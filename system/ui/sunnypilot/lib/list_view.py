import pyray as rl
from dataclasses import dataclass
from system.ui.lib.list_view import *

@dataclass
class ListItemSP(ListItem):

  def is_toggle_action(self) -> bool:
    return isinstance(self.action_item, ToggleAction)
  
  def get_content_width(self, total_width: int) -> int:
    if self.action_item:
      if self.is_toggle_action():
        # For toggles on left, content width isn't reduced by toggle width
        return total_width
      else:
        # Other actions (buttons, text) stay on the right
        return total_width - self.action_item.get_width() - RIGHT_ITEM_PADDING
    return total_width
  
  def get_action_item_rect(self, item_rect: rl.Rectangle) -> rl.Rectangle:
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

  def render(self, rect: rl.Rectangle):
    total_height = self._update_item_rects(rect)

    # Update layout and handle scrolling
    content_rect = rl.Rectangle(rect.x, rect.y, rect.width, total_height)
    scroll_offset = self.scroll_panel.handle_scroll(rect, content_rect)

    # Handle mouse interaction
    if self.scroll_panel.is_click_valid():
      self._handle_mouse_interaction(rect, scroll_offset)

    # Set scissor mode for clipping
    rl.begin_scissor_mode(int(rect.x), int(rect.y), int(rect.width), int(rect.height))

    for i, item in enumerate(self._items):
      y = int(item.rect.y  + scroll_offset.y)
      if y + item.rect.height <= rect.y or y >= rect.y + rect.height:
        continue

      self._render_item(item, y)

      if i < len(self._items) - 1:
        line_y = int(y + item.rect.height - 1)
        rl.draw_line(int(item.rect.x) + LINE_PADDING, line_y, int(item.rect.x + item.rect.width) - LINE_PADDING * 2, line_y, LINE_COLOR)
    rl.end_scissor_mode()

  def _render_item(self, item: ListItemSP, y: int):
    content_x = item.rect.x + ITEM_PADDING
    text_x = content_x

    # Draw toggle on left if it's a toggle item
    if item.action_item and item.is_toggle_action():
      # Get toggle rectangle positioned on the left
      toggle_rect = item.get_action_item_rect(item.rect)
      toggle_rect.y = y

      # Draw the toggle and handle its activation
      if item.action_item.draw(toggle_rect) and item.action_item.enabled:
        if item.callback:
          item.callback()

      # Adjust text position to be after the toggle
      text_x = toggle_rect.x + toggle_rect.width + ITEM_PADDING

    # Draw icon if present
    if item.icon:
      icon_texture = gui_app.texture(os.path.join("icons", item.icon), ICON_SIZE, ICON_SIZE)
      rl.draw_texture(icon_texture, int(text_x), int(y + (ITEM_BASE_HEIGHT - icon_texture.width) // 2), rl.WHITE)
      text_x += ICON_SIZE + ITEM_PADDING

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
      action_rect = item.get_action_item_rect(item.rect)
      action_rect.y = y
      if item.action_item.draw(action_rect) and item.action_item.enabled:
        if item.callback:
          item.callback()

def toggle_item(title: str, description: str | Callable[[], str] | None = None, initial_state: bool = False,
                callback: Callable | None = None, icon: str = "", enabled: bool | Callable[[], bool] = True) -> ListItemSP:
  action = ToggleAction(initial_state=initial_state, enabled=enabled)
  return ListItemSP(title=title, description=description, action_item=action, icon=icon, callback=callback)