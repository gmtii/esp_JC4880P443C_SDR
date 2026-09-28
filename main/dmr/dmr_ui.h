#ifndef DMR_UI_H
#define DMR_UI_H

#include <stdbool.h>
#include "lvgl.h"

/*
 * DMR panel (LVGL 9). While DMR mode is on it covers the spectrum + waterfall
 * area (1024 x 320) with: a status line (lock, repeater/simplex, polarity,
 * measured frequency offset and deviation), one card per timeslot (activity,
 * colour code, talkgroup / destination, source ID, talker alias, call time)
 * and the recent-call log. With voice (mbelib) compiled in, tapping a card
 * fixes playback to that slot and tapping it again returns to auto; the
 * card being heard shows a speaker symbol. Call from the LVGL context only.
 */
void dmr_ui_create(lv_obj_t *parent, int top_y);
void dmr_ui_show(bool show);
bool dmr_ui_is_visible(void);

/* Called when the panel leaves DMR on its own (demod mode changed elsewhere). */
typedef void (*dmr_ui_exit_cb_t)(void);
void dmr_ui_set_exit_callback(dmr_ui_exit_cb_t cb);

#endif /* DMR_UI_H */
