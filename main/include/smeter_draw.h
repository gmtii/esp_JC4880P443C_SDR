/*
 * smeter_draw.h - S-meter bar rendering, pure C (no LVGL, no ESP-IDF).
 *
 * Kept LVGL-free on purpose so the exact same drawing code can be compiled on a
 * host PC to render a preview image before it ever runs on the board.
 *
 * Scale (standard HF IARU convention): S9 = -73 dBm, 6 dB per S-unit, so
 * S0 = -127 dBm. Above S9 the scale runs to S9+60 = -13 dBm.
 * Horizontal layout is piecewise-linear in dB: S0..S9 takes the first
 * SMETER_S9_FRAC of the width, S9..S9+60 the rest (like most real rigs, the
 * S-unit part is stretched, the "+dB" part compressed).
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SMETER_DBM_S0      (-127.0f)
#define SMETER_DBM_S9      (-73.0f)
#define SMETER_DBM_TOP     (-13.0f)  /* S9+60 */
#define SMETER_S9_FRAC     0.55f  /* 0.62 made the +40/+60 labels collide */

/* Canvas geometry (pixels) */
#define SMETER_CANVAS_W    324
#define SMETER_CANVAS_H    30
#define SMETER_SEG_PITCH   6         /* 5 px segment + 1 px gap -> 54 segments */
#define SMETER_SEG_COUNT   (SMETER_CANVAS_W / SMETER_SEG_PITCH)
#define SMETER_TICK_Y      0         /* tick marks: rows 0..7 */
#define SMETER_BAR_Y       10        /* bar: rows 10..27 */
#define SMETER_BAR_H       18

/* dBm -> x position on the canvas (0..SMETER_CANVAS_W), clamped. */
int smeter_dbm_to_x(float dbm);

/* "S7", "S9", "S9+10" ... into buf (>= 8 bytes). */
void smeter_format_s(float dbm, char *buf, int buflen);

/* Zone colour for text readouts, RGB888 (0xRRGGBB): green below S9, amber
 * S9..S9+20, red above - same zones as the bar itself. */
uint32_t smeter_zone_rgb(float dbm);

/*
 * Render the full canvas (ticks + bar + peak marker) into an RGB565 buffer of
 * SMETER_CANVAS_W * SMETER_CANVAS_H pixels, same packing the spectrum canvas
 * already uses. level_dbm is the (already smoothed) bar level, peak_dbm the
 * peak-hold marker (pass a value <= SMETER_DBM_S0 to hide it).
 */
void smeter_render(uint16_t *buf, float level_dbm, float peak_dbm);

/* Tick/label positions for the scale text above the bar. */
typedef struct {
    float dbm;
    const char *text;
    uint8_t red;   /* 1 = "+dB" part (drawn in red) */
} smeter_label_t;

extern const smeter_label_t smeter_labels[];
extern const int smeter_label_count;

#ifdef __cplusplus
}
#endif
