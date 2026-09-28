/*
 * smeter_draw.c - see smeter_draw.h. Pure C: builds on the host for previews.
 */
#include "smeter_draw.h"

#include <stdio.h>

/* ------------------------------------------------------------------------- */

const smeter_label_t smeter_labels[] = {
    { -121.0f, "1",   0 },
    { -109.0f, "3",   0 },
    {  -97.0f, "5",   0 },
    {  -85.0f, "7",   0 },
    {  -73.0f, "9",   0 },
    {  -53.0f, "+20", 1 },
    {  -33.0f, "+40", 1 },
    {  -13.0f, "+60", 1 },
};
const int smeter_label_count = (int)(sizeof(smeter_labels) / sizeof(smeter_labels[0]));

/* ------------------------------------------------------------------------- */

typedef struct { uint8_t r, g, b; } rgb_t;

static inline uint16_t pack565(rgb_t c)
{
    return (uint16_t)(((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3));
}

static inline rgb_t mix(rgb_t a, rgb_t b, int t256) /* t256: 0 = a, 256 = b */
{
    rgb_t o;
    o.r = (uint8_t)(a.r + (((int)b.r - (int)a.r) * t256 >> 8));
    o.g = (uint8_t)(a.g + (((int)b.g - (int)a.g) * t256 >> 8));
    o.b = (uint8_t)(a.b + (((int)b.b - (int)a.b) * t256 >> 8));
    return o;
}

static const rgb_t k_bg        = { 12, 15, 20 };
static const rgb_t k_white     = { 255, 255, 255 };
static const rgb_t k_black     = { 0, 0, 0 };
static const rgb_t k_green_lo  = { 0, 160, 70 };
static const rgb_t k_green_hi  = { 110, 235, 60 };
static const rgb_t k_amber     = { 245, 190, 0 };
static const rgb_t k_red       = { 245, 55, 40 };
static const rgb_t k_tick      = { 130, 140, 150 };
static const rgb_t k_tick_red  = { 200, 80, 70 };

int smeter_dbm_to_x(float dbm)
{
    float f;
    if (dbm <= SMETER_DBM_S0) return 0;
    if (dbm >= SMETER_DBM_TOP) return SMETER_CANVAS_W;
    if (dbm <= SMETER_DBM_S9)
        f = (dbm - SMETER_DBM_S0) / (SMETER_DBM_S9 - SMETER_DBM_S0) * SMETER_S9_FRAC;
    else
        f = SMETER_S9_FRAC + (dbm - SMETER_DBM_S9) / (SMETER_DBM_TOP - SMETER_DBM_S9) * (1.0f - SMETER_S9_FRAC);
    return (int)(f * (float)SMETER_CANVAS_W + 0.5f);
}

static float x_to_dbm(int x)
{
    const float f = (float)x / (float)SMETER_CANVAS_W;
    if (f <= SMETER_S9_FRAC)
        return SMETER_DBM_S0 + f / SMETER_S9_FRAC * (SMETER_DBM_S9 - SMETER_DBM_S0);
    return SMETER_DBM_S9 + (f - SMETER_S9_FRAC) / (1.0f - SMETER_S9_FRAC) * (SMETER_DBM_TOP - SMETER_DBM_S9);
}

void smeter_format_s(float dbm, char *buf, int buflen)
{
    if (dbm < SMETER_DBM_S9 - 3.0f) {
        int s = (int)((dbm - SMETER_DBM_S0) / 6.0f + 0.5f);
        if (s < 0) s = 0;
        if (s > 9) s = 9;
        snprintf(buf, (size_t)buflen, "S%d", s);
    } else {
        int over = (int)((dbm - SMETER_DBM_S9) / 5.0f + 0.5f) * 5; /* nearest 5 dB */
        if (over <= 0) snprintf(buf, (size_t)buflen, "S9");
        else snprintf(buf, (size_t)buflen, "S9+%d", over > 60 ? 60 : over);
    }
}

uint32_t smeter_zone_rgb(float dbm)
{
    if (dbm < SMETER_DBM_S9 - 3.0f) return 0x6EEB3C;
    if (dbm < SMETER_DBM_S9 + 20.0f) return 0xF5BE00;
    return 0xF53728;
}

/* Lit colour of the segment whose centre sits at this dBm */
static rgb_t zone_color(float dbm)
{
    if (dbm < SMETER_DBM_S9) {
        int t = (int)((dbm - SMETER_DBM_S0) / (SMETER_DBM_S9 - SMETER_DBM_S0) * 256.0f);
        if (t < 0) t = 0;
        if (t > 256) t = 256;
        return mix(k_green_lo, k_green_hi, t);
    }
    if (dbm < SMETER_DBM_S9 + 20.0f) return k_amber;
    return k_red;
}

static void fill_rect(uint16_t *buf, int x, int y, int w, int h, uint16_t c)
{
    for (int j = y; j < y + h; j++) {
        if (j < 0 || j >= SMETER_CANVAS_H) continue;
        uint16_t *row = &buf[j * SMETER_CANVAS_W];
        for (int i = x; i < x + w; i++) {
            if (i >= 0 && i < SMETER_CANVAS_W) row[i] = c;
        }
    }
}

static void draw_tick(uint16_t *buf, float dbm, int h, rgb_t c)
{
    int x = smeter_dbm_to_x(dbm);
    if (x >= SMETER_CANVAS_W) x = SMETER_CANVAS_W - 1;
    fill_rect(buf, x, SMETER_TICK_Y + (8 - h), 1, h, pack565(c));
}

void smeter_render(uint16_t *buf, float level_dbm, float peak_dbm)
{
    const int lit_x = smeter_dbm_to_x(level_dbm);
    const int peak_x = (peak_dbm > SMETER_DBM_S0) ? smeter_dbm_to_x(peak_dbm) : -1000;

    fill_rect(buf, 0, 0, SMETER_CANVAS_W, SMETER_CANVAS_H, pack565(k_bg));

    /* Scale ticks: every S-unit (major on the labelled odd ones), every 10 dB above S9 */
    for (int s = 1; s <= 9; s++) {
        draw_tick(buf, SMETER_DBM_S0 + 6.0f * (float)s, (s & 1) ? 7 : 4, k_tick);
    }
    for (int d = 10; d <= 60; d += 10) {
        draw_tick(buf, SMETER_DBM_S9 + (float)d, (d % 20 == 0) ? 7 : 4, k_tick_red);
    }

    /* Bar: LED-style segments. Lit = zone colour with a light top edge and a
     * darker bottom edge; unlit = same hue at ~15% so the scale's colour zones
     * stay readable even with no signal. */
    for (int k = 0; k < SMETER_SEG_COUNT; k++) {
        const int x0 = k * SMETER_SEG_PITCH;
        const int w = SMETER_SEG_PITCH - 1;
        const float centre_dbm = x_to_dbm(x0 + w / 2);
        const rgb_t base = zone_color(centre_dbm);
        const int is_peak = (peak_x >= x0 && peak_x < x0 + SMETER_SEG_PITCH);

        if (x0 + w / 2 < lit_x || is_peak) {
            const rgb_t body = is_peak ? mix(base, k_white, 150) : base;
            fill_rect(buf, x0, SMETER_BAR_Y, w, SMETER_BAR_H, pack565(body));
            fill_rect(buf, x0, SMETER_BAR_Y, w, 2, pack565(mix(body, k_white, 110)));
            fill_rect(buf, x0, SMETER_BAR_Y + SMETER_BAR_H - 2, w, 2, pack565(mix(body, k_black, 90)));
        } else {
            fill_rect(buf, x0, SMETER_BAR_Y, w, SMETER_BAR_H, pack565(mix(k_bg, base, 40)));
        }
    }
}
