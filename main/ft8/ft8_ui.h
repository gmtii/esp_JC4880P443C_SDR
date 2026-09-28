#ifndef FT8_UI_H
#define FT8_UI_H

#include <stdbool.h>
#include "lvgl.h"

/*
 * FT8 panel for the ESP32-P4 SDR UI (LVGL 9). When FT8 mode is on it covers
 * the spectrum + waterfall area (x=0, y=top_y, 1024 x 320) with:
 *   - a status line (clock + source, band-sync state and DT error, symbol
 *     progress, last-slot stats) and a TIME button: a keypad to set the UTC
 *     time of day by eye (plus a "Sync slot" key for :00/:15/:30/:45) - the band-sync loop does
 *     the fine alignment,
 *   - an audio-frequency scale (0-1600 Hz, the FT8 search window),
 *   - a scrolling cascade of that window (4 px per 6.25 Hz bin = full width,
 *     one row every 320 ms, ~30 s of history),
 *   - the decoded-message list (newest at the bottom).
 * Tapping the cascade hides it and gives its space to the message list (same
 * toggle the DeepSDR 101 has); tapping the list brings the cascade back.
 *
 * All functions must be called from the LVGL context (event callbacks,
 * lv_timer callbacks, or with the display lock held).
 */

/* Creates the (hidden) panel as a child of `parent`. Call at the END of
 * init_ui(), so the panel sits above the spectrum/waterfall canvases and the
 * passband overlay. */
void ft8_ui_create(lv_obj_t *parent, int top_y);

void ft8_ui_show(bool show);
bool ft8_ui_is_visible(void);

/* Called by the panel itself when it has to leave FT8 mode on its own (the
 * demod mode was changed away from USB elsewhere, e.g. from the menu), so the
 * owner (ui.c) can update its FT8 button. */
typedef void (*ft8_ui_exit_cb_t)(void);
void ft8_ui_set_exit_callback(ft8_ui_exit_cb_t cb);

#endif /* FT8_UI_H */
