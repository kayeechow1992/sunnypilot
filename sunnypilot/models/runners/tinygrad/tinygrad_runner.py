import pickle
from typing import cast

import numpy as np
from openpilot.sunnypilot.modeld_v2.runners.tinygrad_helpers import qcom_tensor_from_opencl_address
from openpilot.sunnypilot.models.runners.constants import CLMemDict, FrameDict, NumpyDict, ModelType, ShapeDict, CUSTOM_MODEL_PATH
from openpilot.sunnypilot.models.runners.model_runner import ModelRunner
from openpilot.sunnypilot.models.runners.tinygrad.model_types import PolicyTinygrad, VisionTinygrad, SupercomboTinygrad
from openpilot.system.hardware import TICI
from openpilot.common.swaglog import cloudlog

from tinygrad.tensor import Tensor


class TinygradRunner(ModelRunner, SupercomboTinygrad, PolicyTinygrad, VisionTinygrad):
  """
  A ModelRunner implementation for executing Tinygrad models.

  Handles loading Tinygrad model artifacts (.pkl), preparing inputs as Tinygrad
  Tensors (potentially using QCOM extensions on TICI), running inference,
  and parsing the outputs.

  :param model_type: The type of model (e.g., supercombo) to load and run.
  """
  def __init__(self, model_type: int = ModelType.supercombo):
    ModelRunner.__init__(self)
    SupercomboTinygrad.__init__(self)
    PolicyTinygrad.__init__(self)
    VisionTinygrad.__init__(self)
    self._model_data = self.models.get(model_type)
    if not self._model_data or not self._model_data.model:
      raise ValueError(f"Model data for type {model_type} not available.")

    artifact_filename = self._model_data.model.artifact.fileName
    assert artifact_filename.endswith('_tinygrad.pkl'), \
      f"Invalid model file {artifact_filename} for TinygradRunner"

    model_pkl_path = f"{CUSTOM_MODEL_PATH}/{artifact_filename}"
    with open(model_pkl_path, "rb") as f:
      try:
        # Load the compiled Tinygrad model runner function
        self.model_run = pickle.load(f)
      except FileNotFoundError as e:
        # Provide a helpful error message if the model was built for a different platform
        assert "/dev/kgsl-3d0" not in str(e), "Model was built on C3 or C3X, but is being loaded on PC"
        raise

    # Map input names to their required dtype and device from the loaded model
    self.input_to_dtype = {}
    self.input_to_device = {}
    for idx, name in enumerate(self.model_run.captured.expected_names):
      info = self.model_run.captured.expected_st_vars_dtype_device[idx]
      self.input_to_dtype[name] = info[2]  # dtype
      self.input_to_device[name] = info[3]  # device

  def prepare_vision_inputs(self, imgs_cl: CLMemDict, frames: FrameDict):
    """Prepares vision (image) inputs as Tinygrad Tensors."""
    for key in imgs_cl:
      if TICI and key not in self.inputs:
        # On TICI, directly use OpenCL memory address for efficiency via QCOM extensions
        self.inputs[key] = qcom_tensor_from_opencl_address(imgs_cl[key].mem_address, self.input_shapes[key], dtype=self.input_to_dtype[key])
      elif not TICI:
        # On other platforms, copy data from CL buffer to a numpy array first
        shape = frames[key].buffer_from_cl(imgs_cl[key]).reshape(self.input_shapes[key])
        self.inputs[key] = Tensor(shape, device=self.input_to_device[key], dtype=self.input_to_dtype[key]).realize()

  def prepare_policy_inputs(self, numpy_inputs: NumpyDict):
    """Prepares non-image (policy) inputs as Tinygrad Tensors."""
    for key, value in numpy_inputs.items():
      self.inputs[key] = Tensor(value, device=self.input_to_device[key], dtype=self.input_to_dtype[key]).realize()

  def prepare_inputs(self, imgs_cl: CLMemDict, numpy_inputs: NumpyDict, frames: FrameDict) -> dict:
    """Prepares all vision and policy inputs for the model."""
    self.prepare_vision_inputs(imgs_cl, frames)
    self.prepare_policy_inputs(numpy_inputs)
    return self.inputs

  def _run_model(self) -> NumpyDict:
    """Runs the Tinygrad model inference and parses the outputs."""
    outputs = self.model_run(**self.inputs).numpy().flatten()
    return self._parse_outputs(outputs)

  def _parse_outputs(self, model_outputs: np.ndarray) -> NumpyDict:
    """Parses the raw model outputs using the standard Parser."""
    if self._model_data is None:
      raise ValueError("Model data is not available. Ensure the model is loaded correctly.")

    result: NumpyDict = self.parser_method_dict[self._model_data.model.type.raw](model_outputs)
    return result

class TinygradSplitRunner(ModelRunner):
  """
  A ModelRunner that coordinates separate TinygradVisionRunner and TinygradPolicyRunner instances.

  Manages sequential execution of policy then vision models, using policy outputs as vision inputs.
  """
  def __init__(self):
    super().__init__()
    self.is_20hz_3d = True
    self.vision_runner = TinygradRunner(ModelType.vision)
    self.policy_runner = TinygradRunner(ModelType.policy)

  def _run_model(self) -> NumpyDict:
    """
    Runs models in sequence: policy -> vision, with policy outputs feeding into vision inputs.

    First runs policy model, then transfers applicable outputs to vision model inputs,
    then runs vision model, and finally combines all results.
    """
    # Run the policy model first
    policy_output = self.policy_runner.run_model()

    # Pass policy outputs to vision inputs if they match expected input names
    for key, output_array in policy_output.items():
      if key in self.vision_runner.input_shapes and key in self.vision_runner.input_to_device and key in self.vision_runner.input_to_dtype:
        expected_shape = self.vision_runner.input_shapes[key]

        # Check for shape compatibility
        if output_array.shape != expected_shape:
          cloudlog.warning(f"Shape mismatch for policy->vision key '{key}': policy output {output_array.shape}, vision expects {expected_shape}")

        # Convert the numpy array to a tensor with the expected device and dtype
        device = self.vision_runner.input_to_device[key]
        dtype = self.vision_runner.input_to_dtype[key]
        try:
          tensor = Tensor(output_array, device=device, dtype=dtype).realize()
          self.vision_runner.inputs[key] = tensor
        except Exception as e:
          cloudlog.exception(f"Error transferring policy output '{key}' to vision: {e}")

    # Now run the vision model
    vision_output = self.vision_runner.run_model()

    # Combine results - policy outputs take precedence for any overlapping keys
    # This ensures policy outputs (like recurrent state) are preserved
    combined_output = vision_output.copy()
    combined_output.update(policy_output)
    return cast(NumpyDict, combined_output)

  @property
  def input_shapes(self) -> ShapeDict:
    """
    Returns combined external input shapes from both models.

    Excludes vision inputs that come from policy outputs since those are internal to the split model.
    """
    vision_shapes = self.vision_runner.input_shapes.copy()
    policy_shapes = self.policy_runner.input_shapes.copy()

    # If policy model provides outputs that match vision inputs, those are internal connections
    # and not part of the external input interface of the split model
    if self.policy_runner._model_data and self.policy_runner._model_data.output_slices:
      policy_output_keys = self.policy_runner._model_data.output_slices.keys()

      # Remove from vision_shapes any keys that match policy outputs
      for key in policy_output_keys:
        if key in vision_shapes:
          del vision_shapes[key]

    # Combine the shapes, with policy shapes taking precedence if there are duplicate keys
    combined_shapes = vision_shapes.copy()
    combined_shapes.update(policy_shapes)
    return cast(ShapeDict, combined_shapes)

  def prepare_inputs(self, imgs_cl: CLMemDict, numpy_inputs: NumpyDict, frames: FrameDict) -> dict:
    """
    Prepares only the initial external inputs for both models.

    The policy -> vision internal data transfer happens during model execution in _run_model.
    """
    # Prepare policy's numpy inputs
    self.policy_runner.prepare_policy_inputs(numpy_inputs)

    # Prepare vision's image inputs
    self.vision_runner.prepare_vision_inputs(imgs_cl, frames)

    # Prepare any additional numpy inputs for vision that aren't provided by policy
    if self.policy_runner._model_data and self.policy_runner._model_data.output_slices:
      policy_output_keys = self.policy_runner._model_data.output_slices.keys()

      # Only add numpy inputs to vision that aren't going to be provided by policy outputs
      for key, value in numpy_inputs.items():
        if key not in policy_output_keys and key in self.vision_runner.input_shapes:
          if key in self.vision_runner.input_to_device and key in self.vision_runner.input_to_dtype:
            device = self.vision_runner.input_to_device[key]
            dtype = self.vision_runner.input_to_dtype[key]
            self.vision_runner.inputs[key] = Tensor(value, device=device, dtype=dtype).realize()

    # Return combined inputs dictionary (even though they're stored in separate runners)
    return {**self.policy_runner.inputs, **self.vision_runner.inputs}
