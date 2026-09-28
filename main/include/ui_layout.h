#pragma once

/*
 * Screen layout profiles - the single source of UI geometry.
 *
 * Select in menuconfig: "SDR: Display" -> "Screen layout". Everything that
 * depends on the screen size is derived from here (spectrum/waterfall canvases,
 * button row, header, S-meter, menu, and the FT8 / DMR / AIS panels), so one
 * code base serves both panels. The DSP side is unaffected: the FFT stays at
 * SAMPLE_BUFFER_SIZE (1024) bins and the display maps bins to its own width
 * (see spec_col() in ui.c).
 */

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

#if defined(CONFIG_UI_LAYOUT_800x480)
/* ---------------- 800 x 480 ---------------- */
#define UI_SCREEN_W 800
#define UI_SCREEN_H 480

#define UI_SPEC_W 800         /* spectrum/waterfall width in px (1024 FFT bins mapped onto it) */
#define UI_WAVEFORM_H 140     /* spectrum height */
#define UI_WATERFALL_H 100    /* waterfall height */

#define UI_BUTTON_ROW_H 64
#define UI_CTRL_BTN_W 124     /* 6 buttons: 10 + 6*124 + 5*8 = 794 */
#define UI_CTRL_BTN_H 56
#define UI_CTRL_BTN_GAP 8

#define UI_FREQ_X 285
#define UI_FREQ_Y 8
#define UI_MODE_X 285
#define UI_MODE_Y 68
#define UI_BADGE_ROW_Y 122
#define UI_BADGE_SMALL_PITCH 56  /* 10 small badges */
#define UI_BADGE_SMALL_W 50
#define UI_BADGE_BIG_X 570       /* 2 wide badges */
#define UI_BADGE_BIG_PITCH 112
#define UI_BADGE_BIG_W 106

#define UI_SMETER_PANEL_W 270
#define UI_SMETER_PANEL_H 112
#define UI_SMETER_CANVAS_W 246   /* multiple of the 6 px segment pitch */
#define UI_SMETER_S_Y 66         /* big "S9" label */
#define UI_SMETER_DBM_Y 68
#define UI_SMETER_PEAK_Y 88

#define UI_MENU_W 800
#define UI_MENU_H 420
#define UI_MENU_SLIDERS_H 150
#define UI_MENU_SLIDER_BOX_W 180
#define UI_MENU_SLIDER_BOX_H 100
#define UI_MENU_BTN_W 118
#define UI_MENU_BTN_H 52
#define UI_MENU_GRID_H 180       /* 3 rows x 52 + 2 x 8 */
#define UI_MENU_GRID_ROW_GAP 8
#define UI_MENU_PALETTE_W 150
#define UI_FREQ_POPUP_W 460
#define UI_FREQ_POPUP_H 280
#define UI_FREQ_POPUP_BTNM_H 200

#define UI_FT8_CASCADE_H 64      /* rows (~20 s of history) */
#define UI_FT8_PX_PER_BIN 3      /* 256 bins x 3 = 768 px */

#define UI_DMR_CARD_H 122
#define UI_DMR_Y_TITLE 6
#define UI_DMR_Y_BIG 28
#define UI_DMR_Y_SRC 78
#define UI_DMR_Y_ALIAS 99

#define UI_AIS_LIST_ROWS 9
#define UI_AIS_COL_W {170, 44, 56, 50, 84, 50, 56} /* = 510 px */

#else
/* ---------------- 1024 x 600 (default, the original design) ---------------- */
#define UI_SCREEN_W 1024
#define UI_SCREEN_H 600

#define UI_SPEC_W 1024
#define UI_WAVEFORM_H 192
#define UI_WATERFALL_H 128

#define UI_BUTTON_ROW_H 80
#define UI_CTRL_BTN_W 150
#define UI_CTRL_BTN_H 70
#define UI_CTRL_BTN_GAP 20

#define UI_FREQ_X 360
#define UI_FREQ_Y 15
#define UI_MODE_X 360
#define UI_MODE_Y 90
#define UI_BADGE_ROW_Y 140
#define UI_BADGE_SMALL_PITCH 70
#define UI_BADGE_SMALL_W 60
#define UI_BADGE_BIG_X 710
#define UI_BADGE_BIG_PITCH 110
#define UI_BADGE_BIG_W 100

#define UI_SMETER_PANEL_W 345
#define UI_SMETER_PANEL_H 128
#define UI_SMETER_CANVAS_W 324
#define UI_SMETER_S_Y 74
#define UI_SMETER_DBM_Y 80
#define UI_SMETER_PEAK_Y 106

#define UI_MENU_W 1024
#define UI_MENU_H 500
#define UI_MENU_SLIDERS_H 190
#define UI_MENU_SLIDER_BOX_W 225
#define UI_MENU_SLIDER_BOX_H 120
#define UI_MENU_BTN_W 140
#define UI_MENU_BTN_H 60
#define UI_MENU_GRID_H 210
#define UI_MENU_GRID_ROW_GAP 10
#define UI_MENU_PALETTE_W 180
#define UI_FREQ_POPUP_W 520
#define UI_FREQ_POPUP_H 300
#define UI_FREQ_POPUP_BTNM_H 220

#define UI_FT8_CASCADE_H 96
#define UI_FT8_PX_PER_BIN 4      /* 256 bins x 4 = 1024 px */

#define UI_DMR_CARD_H 150
#define UI_DMR_Y_TITLE 8
#define UI_DMR_Y_BIG 34
#define UI_DMR_Y_SRC 90
#define UI_DMR_Y_ALIAS 116

#define UI_AIS_LIST_ROWS 12
#define UI_AIS_COL_W {230, 56, 70, 64, 100, 64, 70} /* = 654 px */
#endif

/* ---------------- derived ---------------- */
#define UI_PANEL_H (UI_WAVEFORM_H + UI_WATERFALL_H) /* FT8 / DMR / AIS panels cover spectrum + waterfall */
#define UI_SPECTRUM_TOP_Y (UI_SCREEN_H - UI_BUTTON_ROW_H - UI_WATERFALL_H - UI_WAVEFORM_H)
