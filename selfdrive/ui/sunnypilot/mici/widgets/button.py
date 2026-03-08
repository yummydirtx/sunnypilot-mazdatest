"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

"""SP-specific mici widget extensions — keeps upstream button.py clean.

Classes:
  _BadgeMixin       — Badge pill rendering (flow layout with row wrapping)
  BigButtonSP       — BigButton + badges + active-state green tint + link_sub_panel helper
  BigMultiParamToggleSP — BigMultiParamToggle + bounds-checked refresh + dynamic pill spacing
  BigParamOption    — Numeric param button that opens a NumberPickerScreen on tap

Design notes:
  - BigButtonSP duplicates BigButton._draw_content to integrate badge rendering in the
    subtitle area. This is intentional to avoid modifying upstream BigButton.
  - Upstream BigParamControl is used directly for toggles (no SP subclass needed).
  - lambda-based set_enabled (e.g. `set_enabled(lambda: toggle._checked)`) is used so
    dependent widgets respond on the same frame as the toggle tap, since mouse events
    process during rendering after _update_state runs.
"""

from collections.abc import Callable

import pyray as rl

from openpilot.selfdrive.ui.mici.widgets.button import (
  BigButton,
  BigMultiParamToggle,
  LABEL_COLOR,
)
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached

try:
  from openpilot.common.params import Params
except ImportError:
  Params = None

BADGE_GREEN_BG = rl.Color(51, 171, 76, 50)
BADGE_GREEN_FG = rl.Color(140, 210, 150, 200)
CARD_ACTIVE_TINT = rl.Color(140, 230, 150, 255)


class _BadgeMixin:
  """Badge pill rendering and active-state tracking for BigButton subclasses."""

  def _init_badges(self):
    self._badge_labels: list[str] | None = None
    self._active: bool = True

  @property
  def active(self) -> bool:
    return self._active

  def set_active(self, active: bool) -> None:
    """Set whether the setting is logically in effect (controls badge dimming, not interactivity)."""
    self._active = active

  def set_badges(self, entries: list[tuple[str, str]]):
    """Set badge pill chips from (key, value) pairs.

    - 'off' values are hidden
    - 'on' values show just the key
    - Other values show just the value
    """
    labels = []
    for key, val in entries:
      if val == 'off':
        continue
      labels.append(key if val == 'on' else val)
    new_labels = labels or None
    if new_labels == self._badge_labels:
      return
    self._badge_labels = new_labels
    self.value = ""
    self._update_label_layout()

  def _draw_badges(self, rect: rl.Rectangle):
    """Render cached badge labels as outlined pill chips in a uniform-width flow layout."""
    font = gui_app.font(FontWeight.BOLD)
    font_size = 26
    h_pad = 10
    gap = 8
    alpha_mult = 1.0 if self._active else 0.3
    border_color = rl.Color(BADGE_GREEN_FG.r, BADGE_GREEN_FG.g, BADGE_GREEN_FG.b, int(BADGE_GREEN_FG.a * 0.4 * alpha_mult))
    text_color = rl.Color(BADGE_GREEN_FG.r, BADGE_GREEN_FG.g, BADGE_GREEN_FG.b, int(BADGE_GREEN_FG.a * alpha_mult))

    assert self._badge_labels is not None
    specs = []
    for label in self._badge_labels:
      text_w = measure_text_cached(font, label, font_size).x
      specs.append((label, text_w + h_pad * 2, text_w))

    # Flow layout: wrap into rows, tracking max pill width per row
    rows: list[list] = []
    current_row: list = []
    row_width = 0.0
    for spec in specs:
      w = spec[1]
      needed = w + (gap if current_row else 0)
      if current_row and row_width + needed > rect.width:
        rows.append(current_row)
        current_row = [spec]
        row_width = w
      else:
        current_row.append(spec)
        row_width += needed
    if current_row:
      rows.append(current_row)

    # Fit badge height to available space
    num_rows = len(rows)
    text_h = measure_text_cached(font, "Xg", font_size).y
    max_h = (rect.height - gap * (num_rows - 1)) / num_rows
    badge_h = max(text_h, min(text_h + 10, max_h))

    # Draw rows bottom-up
    avail_w = rect.width
    cy = rect.y + rect.height - badge_h
    for row in reversed(rows):
      total_badge_w = sum(bw for _, bw, _ in row)
      row_gap = gap if len(row) <= 1 else (avail_w - total_badge_w) / (len(row) - 1)
      cx = rect.x
      for label, badge_w, text_w in row:
        pill_rect = rl.Rectangle(cx, cy, badge_w, badge_h)
        rl.draw_rectangle_rounded_lines_ex(pill_rect, 0.5, 6, 2, border_color)
        ty = cy + (badge_h - text_h) / 2 - 2
        rl.draw_text_ex(font, label, rl.Vector2(cx + (badge_w - text_w) / 2, ty), font_size, 0, text_color)
        cx += badge_w + row_gap
      cy -= badge_h + gap


class BigButtonSP(_BadgeMixin, BigButton):
  """BigButton extended with badge pills, active-state tinting, and subtitle font size."""

  def __init__(self, text: str, value: str = "", icon="", icon_size: tuple[int, int] = (64, 64), scroll: bool = False):
    self._init_badges()
    BigButton.__init__(self, text, value, icon, icon_size, scroll)

  def set_subtitle_font_size(self, size: int):
    self._sub_label.set_font_size(size)

  def _update_label_layout(self):
    self._label.set_font_size(self._get_label_font_size())
    if self.value or self._badge_labels:
      self._label.set_alignment_vertical(rl.GuiTextAlignmentVertical.TEXT_ALIGN_TOP)
    else:
      self._label.set_alignment_vertical(rl.GuiTextAlignmentVertical.TEXT_ALIGN_BOTTOM)

  def set_value(self, value: str):
    """Set plain text subtitle, clearing any badges."""
    if value == self.value and self._badge_labels is None:
      return
    self.value = value
    self._badge_labels = None
    self._sub_label.set_text(value)
    self._update_label_layout()

  def _draw_content(self, btn_y: float):
    # LABEL
    label_x = self._rect.x + self.LABEL_HORIZONTAL_PADDING
    label_color = LABEL_COLOR if self.enabled else rl.Color(255, 255, 255, int(255 * 0.35))
    self._label.set_color(label_color)
    label_rect = rl.Rectangle(label_x, btn_y + self.LABEL_VERTICAL_PADDING, self._width_hint(),
                              self._rect.height - self.LABEL_VERTICAL_PADDING * 2)
    self._label.render(label_rect)

    if self.value or self._badge_labels:
      label_y = btn_y + self.LABEL_VERTICAL_PADDING + self._label.get_content_height(self._width_hint())
      sub_label_height = btn_y + self._rect.height - self.LABEL_VERTICAL_PADDING - label_y
      sub_label_rect = rl.Rectangle(label_x, label_y, self._width_hint(), sub_label_height)
      if self._badge_labels:
        # Add top margin so pills don't touch title descenders
        badge_margin = 4
        self._draw_badges(rl.Rectangle(label_x, label_y + badge_margin, self._width_hint(), sub_label_height - badge_margin))
      else:
        self._sub_label.render(sub_label_rect)

    # ICON
    if self._txt_icon:
      rotation = 0
      if self._rotate_icon_t is not None:
        rotation = (rl.get_time() - self._rotate_icon_t) * 180
      x = self._rect.x + self._rect.width - 30 - self._txt_icon.width / 2
      y = btn_y + 30 + self._txt_icon.height / 2
      source_rec = rl.Rectangle(0, 0, self._txt_icon.width, self._txt_icon.height)
      dest_rec = rl.Rectangle(x, y, self._txt_icon.width, self._txt_icon.height)
      origin = rl.Vector2(self._txt_icon.width / 2, self._txt_icon.height / 2)
      rl.draw_texture_pro(self._txt_icon, source_rec, dest_rec, origin, rotation, rl.Color(255, 255, 255, int(255 * 0.9)))

  def link_sub_panel(self, items):
    """Create a sub-panel NavScroller with the given items, linked to this button's click."""
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.scroller import NavScroller

    view = NavScroller()
    view.add_widgets(items)
    self.set_click_callback(lambda: gui_app.push_widget(view))
    return view

  def _render(self, _):
    txt_bg, btn_x, btn_y, scale = self._handle_background()
    bg_tint = CARD_ACTIVE_TINT if self._badge_labels and self._active else rl.WHITE

    if self._scroll:
      scaled_rect = rl.Rectangle(btn_x, btn_y, self._rect.width * scale, self._rect.height * scale)
      rl.draw_rectangle_rounded(scaled_rect, 0.4, 7, rl.Color(0, 0, 0, int(255 * 0.5)))
      self._draw_content(btn_y)
      rl.draw_texture_ex(txt_bg, (btn_x, btn_y), 0, scale, bg_tint)
    else:
      rl.draw_texture_ex(txt_bg, (btn_x, btn_y), 0, scale, bg_tint)
      self._draw_content(btn_y)


class BigMultiParamToggleSP(BigMultiParamToggle):
  """BigMultiParamToggle with bounds-checked param reading, refresh, and dynamic pill spacing."""

  def _draw_content(self, btn_y: float):
    # Override upstream's hardcoded 35px pill spacing to fit within button height
    BigButton._draw_content(self, btn_y)
    checked_idx = self._options.index(self.value)
    n = len(self._options)
    pill_h = self._txt_enabled_toggle.height
    step = min(35, (self._rect.height - pill_h) / max(n - 1, 1))
    x = self._rect.x + self._rect.width - self._txt_enabled_toggle.width
    y = btn_y
    for i in range(n):
      self._draw_pill(x, y, checked_idx == i)
      y += step

  def _get_param_index(self) -> int:
    idx = self._params.get(self._param, return_default=True) or 0
    return max(0, min(int(idx), len(self._options) - 1))

  def _load_value(self):
    self.set_value(self._options[self._get_param_index()])

  def refresh(self):
    new_value = self._options[self._get_param_index()]
    if new_value != self.value:
      self.set_value(new_value)


class BigParamOption(BigButton):
  """A BigButton that shows a numeric param value; opens a picker screen on tap."""

  def __init__(
    self,
    text: str,
    param: str,
    min_value: int,
    max_value: int,
    value_change_step: int = 1,
    label_callback: Callable | None = None,
    value_map: dict[int, int] | None = None,
    float_param: bool = False,
    picker_label_callback: Callable | None = None,
    picker_unit: str | Callable[[], str] = "",
    picker_item_width: int = 0,
  ):
    super().__init__(text, "")
    self._param = param
    self._min_value = min_value
    self._max_value = max_value
    self._step = value_change_step
    self._label_callback = label_callback
    self._value_map = value_map
    self._float_param = float_param
    self._picker_label_callback = picker_label_callback
    self._picker_unit = picker_unit
    self._picker_item_width = picker_item_width
    self._params = Params()
    self._current = self._read_value()
    self._update_display()
    self.set_click_callback(self._open_picker)

  def _read_value(self) -> int:
    val = self._params.get(self._param, return_default=True)
    try:
      return int(float(val)) if val is not None else self._min_value
    except (ValueError, TypeError):
      return self._min_value

  def _display_value(self) -> int:
    """Return the mapped display value if value_map exists, otherwise the raw value."""
    if self._value_map and self._current in self._value_map:
      return self._value_map[self._current]
    return self._current

  def _update_display(self):
    display_val = self._display_value()
    if self._label_callback:
      self.set_value(self._label_callback(display_val))
    else:
      self.set_value(str(display_val))

  def refresh(self):
    new = self._read_value()
    if new != self._current:
      self._current = new
      self._update_display()
    elif self._label_callback:
      # Label may depend on external state (e.g. offset type), re-render
      self._update_display()

  def _open_picker(self):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.scroller import NavScroller

    picker = self.create_picker_screen()
    view = NavScroller()
    view._scroller._show_scroll_indicator = False
    view._scroller._pad = 0
    view.set_back_callback(lambda: self.refresh())
    view.add_widgets([picker])
    view.set_scrolling_enabled(False)
    gui_app.push_widget(view)

  def create_picker_screen(self):
    """Factory: create a NumberPickerScreen from this option's config."""
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.number_picker import NumberPickerScreen

    kwargs = {}
    if self._picker_item_width:
      kwargs['item_width'] = self._picker_item_width
    return NumberPickerScreen(
      title=self.text,
      param=self._param,
      min_value=self._min_value,
      max_value=self._max_value,
      step=self._step,
      label_callback=self._picker_label_callback,
      value_map=self._value_map,
      float_param=self._float_param,
      unit=self._picker_unit,
      **kwargs,
    )
