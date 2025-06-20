from openpilot.system.ui.lib.list_view import ListView, button_item, toggle_item
from openpilot.system.ui.lib.widget import Widget
from openpilot.common.params import Params


class TripsLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    items = [
    ]

    self._list_widget = ListView(items)

  def _render(self, rect):
    self._list_widget.render(rect)
