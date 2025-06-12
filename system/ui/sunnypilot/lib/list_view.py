import pyray as rl
from dataclasses import dataclass
from system.ui.lib.list_view import *

@dataclass
class ListItemSP(ListItem):

  def is_toggle_action(self) -> bool:
    return isinstance(self.action_item, ToggleAction)
  
  def get_right_item_rect(self, item_rect: rl.Rectangle) -> rl.Rectangle:
    if not self.action_item:
      return rl.Rectangle(0, 0, 0, 0)
  
    action_width = self.action_item.get_width()
    
    if self.is_toggle_action():
      # Position toggle on the left
      action_x = item_rect.x + ITEM_PADDING
      action_y = item_rect.y
    else:
      # Position other actions on the right
      action_x = item_rect.x + item_rect.width - action_width
      action_y = item_rect.y
      
    return rl.Rectangle(action_x, action_y, action_width, ITEM_BASE_HEIGHT)


class ListViewSP(ListView):
  def __init__(self, items: list[ListItemSP]):
    super().__init__(items)
    self._items: list[ListItemSP] = items

  def _render_item(self, item: ListItemSP, y: int):
    content_x = item.rect.x + ITEM_PADDING
    text_x = content_x

    # Draw toggle on left if it's a toggle item
    if item.action_item and item.is_toggle_action():
      # Get toggle rectangle positioned on the left
      toggle_rect = item.get_right_item_rect(item.rect)
      toggle_rect.y = y

      # Draw the toggle and handle its activation
      if item.action_item.draw(toggle_rect) and item.action_item.enabled:
        if item.callback:
          item.callback()

      # Adjust text position to be after the toggle
      text_x = toggle_rect.x + toggle_rect.width + ITEM_PADDING


    # Draw main text
    text_size = measure_text_cached(self._font, item.title, ITEM_TEXT_FONT_SIZE)
    item_y = y + (ITEM_BASE_HEIGHT - text_size.y) // 2
    rl.draw_text_ex(self._font, item.title, rl.Vector2(text_x, item_y), ITEM_TEXT_FONT_SIZE, 0, ITEM_TEXT_COLOR)

    # Draw description if visible
    current_description = item.get_description()
    if item.description_visible and current_description and item._wrapped_description:
      rl.draw_text_ex(
        self._font,
        item._wrapped_description,
        (text_x, y + ITEM_DESC_V_OFFSET),
        ITEM_DESC_FONT_SIZE,
        0,
        ITEM_DESC_TEXT_COLOR,
      )

    # Draw non-toggle action items (buttons, text) on the right
    if item.action_item and not item.is_toggle_action():
      action_rect = item.get_right_item_rect(item.rect)
      action_rect.y = y
      if item.action_item.draw(action_rect) and item.action_item.enabled:
        if item.callback:
          item.callback()

def toggle_item(title: str, description: str | Callable[[], str] | None = None, initial_state: bool = False,
                callback: Callable | None = None, icon: str = "", enabled: bool | Callable[[], bool] = True) -> ListItemSP:
  action = ToggleAction(initial_state=initial_state, enabled=enabled)
  return ListItemSP(title=title, description=description, action_item=action, icon=icon, callback=callback)