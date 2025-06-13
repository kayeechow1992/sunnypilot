from openpilot.system.ui.lib.list_view import ListView, ButtonItem, ToggleItem
from openpilot.system.ui.lib.widget import Widget
from openpilot.common.params import Params


class CruiseLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    items = [
    ]

    self._list_widget = ListView(items)

  def _render(self, rect):
    self._list_widget.render(rect)
