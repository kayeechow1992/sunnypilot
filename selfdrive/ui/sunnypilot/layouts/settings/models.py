from openpilot.system.ui.lib.list_view import ListView, ButtonItem, ToggleItem
from openpilot.system.ui.lib.widget import Widget
from openpilot.common.params import Params


class ModelsLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    items = [
      ButtonItem("Current Model", "SELECT"),
      ToggleItem("Live Learning Steer Delay"),
    ]

    self._list_widget = ListView(items)

  def _render(self, rect):
    self._list_widget.render(rect)
