#ifndef AIS_UI_H
#define AIS_UI_H

#include <stdbool.h>
#include "lvgl.h"

/*
 * AIS panel (LVGL 9): covers the spectrum + waterfall area (1024 x 320) with a
 * 320 x 320 radar view centred on the own QTH (grid from FT8 / the PC tool;
 * tap it to step the range AUTO -> 2 -> 5 -> 10 -> 20 -> 50 -> 100 NM; AUTO fits
 * ~80 % of the vessels and out-of-range ones show as ticks on the rim; class A green / class B cyan triangles along COG, base
 * stations yellow and aids to navigation magenta diamonds), the most recently
 * heard vessels (name or MMSI, class, SOG, COG, distance, bearing, age) and a
 * status line (frames per channel, last base-station UTC).
 * Call from the LVGL context only.
 */
void ais_ui_create(lv_obj_t *parent, int top_y);
void ais_ui_show(bool show);
bool ais_ui_is_visible(void);

typedef void (*ais_ui_exit_cb_t)(void);
void ais_ui_set_exit_callback(ais_ui_exit_cb_t cb);

#endif /* AIS_UI_H */
