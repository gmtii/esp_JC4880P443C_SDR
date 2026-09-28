#include "ais_ui.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "sdr.h"
#include "ais_app.h"
#include "ais_demod.h"
#include "ais_msg.h"
#include "ft8_decoder.h" /* own QTH + distance (shared with FT8 / DMR) */

extern int demod_modo;

#define PANEL_W 1024
#define PANEL_H (WAVEFORM_HEIGHT + WATERFALL_HEIGHT) /* 320 */
#define MAP 320
#define LIST_X (MAP + 12)
#define LIST_ROWS 12
#define STALE_MS 600000u /* vessels not heard for 10 min are dropped from the view */

/* Vessel list as an lv_table with fixed column widths (px): columns line up
 * whatever the font - padded printf columns only worked with a monospaced one. */
enum { COL_NAME, COL_CL, COL_SOG, COL_COG, COL_DIST, COL_BRG, COL_AGE, NCOLS };
static const int16_t k_col_w[NCOLS] = {230, 56, 70, 64, 100, 64, 70}; /* = 654 px */
static const char *const k_col_title[NCOLS] = {"NAME / MMSI", "CL", "SOG", "COG", "DIST", "BRG", "AGE"};

#define LIST_FONT (&lv_font_montserrat_14)

static lv_obj_t *s_panel, *s_status, *s_list, *s_canvas, *s_range_lbl;
static lv_timer_t *s_timer;
static uint16_t *s_map; /* MAP x MAP RGB565, PSRAM */
static ais_ui_exit_cb_t s_exit_cb;
/* Radar range: -1 = AUTO, otherwise an index into k_ranges. Tap the radar to
 * step AUTO -> 2 -> 5 -> ... -> 100 NM -> AUTO. */
static const float k_ranges[] = {2, 5, 10, 20, 50, 100};
#define N_RANGES ((int)(sizeof(k_ranges) / sizeof(k_ranges[0])))
static int s_zoom = -1;
static uint32_t s_now_ms;

void ais_ui_set_exit_callback(ais_ui_exit_cb_t cb) { s_exit_cb = cb; }

/* ---- tiny RGB565 drawing ------------------------------------------------------ */
static inline void px(int x, int y, uint16_t c)
{
    if (x >= 0 && y >= 0 && x < MAP && y < MAP)
    {
        s_map[y * MAP + x] = c;
    }
}

static void line(int x0, int y0, int x1, int y1, uint16_t c)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1, dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1, e = dx + dy;
    for (;;)
    {
        px(x0, y0, c);
        if (x0 == x1 && y0 == y1)
        {
            break;
        }
        int e2 = 2 * e;
        if (e2 >= dy) { e += dy; x0 += sx; }
        if (e2 <= dx) { e += dx; y0 += sy; }
    }
}

static void circle(int cx, int cy, int r, uint16_t c)
{
    for (int a = 0; a < 360; a += 2)
    {
        float t = a * 0.0174533f;
        px(cx + (int)lroundf(r * cosf(t)), cy + (int)lroundf(r * sinf(t)), c);
    }
}

#define C_BG 0x0841
#define C_RING 0x061f
#define C_OWN 0xFFFF
#define C_A 0x07E0
#define C_B 0x07FF
#define C_BASE 0xFFE0
#define C_ATON 0xF81F

/* distance (NM) and true bearing (deg) from own QTH */
static bool dist_brg(const ais_vessel_t *v, float mlat, float mlon, float *nm, float *brg)
{
    if (!v->pos_valid)
    {
        return false;
    }
    *nm = ft8_distance_km(mlat, mlon, v->lat, v->lon) / 1.852f;
    {
        const float d2r = 0.0174533f;
        float y = sinf((v->lon - mlon) * d2r) * cosf(v->lat * d2r);
        float x = cosf(mlat * d2r) * sinf(v->lat * d2r) - sinf(mlat * d2r) * cosf(v->lat * d2r) * cosf((v->lon - mlon) * d2r);
        *brg = fmodf(atan2f(y, x) / d2r + 360.0f, 360.0f);
    }
    return true;
}

static int cmp_float(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

static void draw_map(const ais_vessel_t *v, int n, float mlat, float mlon)
{
    float dists[AIS_MAX_VESSELS];
    int nd = 0;
    float range = k_ranges[N_RANGES - 1];
    const int c = MAP / 2, R = MAP / 2 - 6;

    for (int i = 0; i < n; i++)
    {
        float nm, brg;
        if (s_now_ms - v[i].last_ms < STALE_MS && dist_brg(&v[i], mlat, mlon, &nm, &brg))
        {
            dists[nd++] = nm;
        }
    }
    if (s_zoom >= 0)
    {
        range = k_ranges[s_zoom];
    }
    else if (nd > 0)
    {
        /* AUTO: fit ~80 % of the vessels, not the farthest one - a single
         * distant ship used to squeeze everything nearby into the centre */
        float target;
        qsort(dists, (size_t)nd, sizeof(float), cmp_float);
        target = dists[(nd * 8 + 9) / 10 - 1];
        range = k_ranges[N_RANGES - 1];
        for (int k = 0; k < N_RANGES; k++)
        {
            if (target <= k_ranges[k] * 0.95f)
            {
                range = k_ranges[k];
                break;
            }
        }
    }
    else
    {
        range = k_ranges[1];
    }

    for (int i = 0; i < MAP * MAP; i++)
    {
        s_map[i] = C_BG;
    }
    circle(c, c, R, C_RING);
    circle(c, c, R / 2, C_RING);
    line(c, c - R, c, c - R + 8, C_OWN); /* north tick */
    line(c - 4, c, c + 4, c, C_OWN);     /* own position */
    line(c, c - 4, c, c + 4, C_OWN);

    for (int i = 0; i < n; i++)
    {
        float nm, brg, sb, cb;
        uint16_t col;
        int x, y;
        if (s_now_ms - v[i].last_ms >= STALE_MS || !dist_brg(&v[i], mlat, mlon, &nm, &brg))
        {
            continue;
        }
        col = v[i].kind == AIS_KIND_CLASS_A ? C_A : v[i].kind == AIS_KIND_CLASS_B ? C_B : v[i].kind == AIS_KIND_BASE ? C_BASE : C_ATON;
        sb = sinf(brg * 0.0174533f);
        cb = cosf(brg * 0.0174533f);
        if (nm > range)
        {
            /* out of range: a short tick on the outer ring, at its bearing */
            line(c + (int)lroundf(sb * (R - 6)), c - (int)lroundf(cb * (R - 6)),
                 c + (int)lroundf(sb * (R + 4)), c - (int)lroundf(cb * (R + 4)), col);
            continue;
        }
        x = c + (int)lroundf(sb * nm / range * R);
        y = c - (int)lroundf(cb * nm / range * R);
        if (v[i].kind == AIS_KIND_BASE || v[i].kind == AIS_KIND_ATON || v[i].cog_deg < 0.0f)
        {
            for (int d = -3; d <= 3; d++) /* diamond for fixed stations / no course */
            {
                px(x + d, y - (3 - abs(d)), col);
                px(x + d, y + (3 - abs(d)), col);
            }
        }
        else
        {
            /* triangle pointing along the course over ground */
            float a = v[i].cog_deg * 0.0174533f;
            int tx = x + (int)lroundf(sinf(a) * 8), ty = y - (int)lroundf(cosf(a) * 8);
            int lx = x + (int)lroundf(sinf(a + 2.5f) * 5), ly = y - (int)lroundf(cosf(a + 2.5f) * 5);
            int rx = x + (int)lroundf(sinf(a - 2.5f) * 5), ry = y - (int)lroundf(cosf(a - 2.5f) * 5);
            line(tx, ty, lx, ly, col);
            line(lx, ly, rx, ry, col);
            line(rx, ry, tx, ty, col);
        }
    }
    lv_label_set_text_fmt(s_range_lbl, "%s ring %.0f NM (tap: zoom)", s_zoom < 0 ? "AUTO" : "", (double)range);
    lv_obj_invalidate(s_canvas);
}

/* tap on the radar: next range */
static void map_click_cb(lv_event_t *e)
{
    (void)e;
    s_zoom = (s_zoom + 2 > N_RANGES) ? -1 : s_zoom + 1;
    lv_timer_ready(s_timer); /* redraw now, not at the next 1 s tick */
}

static const char *kind_str(const ais_vessel_t *v)
{
    switch (v->kind)
    {
    case AIS_KIND_CLASS_A: return "A";
    case AIS_KIND_CLASS_B: return "B";
    case AIS_KIND_BASE: return "BS";
    default: return "AtoN";
    }
}

static ais_vessel_t *s_snap; /* [AIS_MAX_VESSELS] UI copy, PSRAM (ais_ui_create) */

static void timer_cb(lv_timer_t *t)
{
    ais_vessel_t *const v = s_snap;
    ais_demod_stats_t ds;
    ais_msg_stats_t ms;
    float mlat = 0.0f, mlon = 0.0f;
    int n, shown = 0;
    bool own;
    (void)t;

    if (s_panel == NULL || v == NULL || lv_obj_has_flag(s_panel, LV_OBJ_FLAG_HIDDEN))
    {
        return;
    }
    if (demod_modo != DEMOD_WFM)
    {
        ais_app_set_active(false);
        ais_ui_show(false);
        if (s_exit_cb != NULL)
        {
            s_exit_cb();
        }
        return;
    }

    n = ais_msg_get_vessels(v, AIS_MAX_VESSELS);
    ais_demod_get_stats(&ds);
    ais_msg_get_stats(&ms);
    s_now_ms = ais_demod_now_ms(); /* same clock as the vessels' last_ms */
    own = ft8_decoder_get_own_latlon(&mlat, &mlon);

    if (ms.utc_valid)
    {
        lv_label_set_text_fmt(s_status, "AIS 162.000 MHz  |  frames A %lu  B %lu  |  vessels %d  |  base station UTC %02d:%02d:%02d",
                              (unsigned long)ds.frames_ok[0], (unsigned long)ds.frames_ok[1], n, ms.hour, ms.minute, ms.second);
    }
    else
    {
        lv_label_set_text_fmt(s_status, "AIS 162.000 MHz  |  frames A %lu  B %lu  |  vessels %d  |  %s",
                              (unsigned long)ds.frames_ok[0], (unsigned long)ds.frames_ok[1], n,
                              own ? "no base station heard yet" : "set your grid (FT8/PC tool) for range/bearing");
    }

    for (int i = 0; i < n && shown < LIST_ROWS; i++)
    {
        char name[24], sog[8], cog[8], dist[12], brg[8], age[12];
        float nm = 0.0f, b = 0.0f;
        const int row = shown + 1; /* row 0 is the header */
        unsigned a = (s_now_ms - v[i].last_ms) / 1000u;
        if (s_now_ms - v[i].last_ms >= STALE_MS)
        {
            continue;
        }
        if (v[i].name[0])
        {
            snprintf(name, sizeof(name), "%.20s", v[i].name);
        }
        else
        {
            snprintf(name, sizeof(name), "%09u", (unsigned)v[i].mmsi);
        }
        snprintf(sog, sizeof(sog), v[i].sog_kn >= 0.0f ? "%.1f" : "-", (double)v[i].sog_kn);
        snprintf(cog, sizeof(cog), v[i].cog_deg >= 0.0f ? "%.0f" : "-", (double)v[i].cog_deg);
        if (own && dist_brg(&v[i], mlat, mlon, &nm, &b))
        {
            snprintf(dist, sizeof(dist), "%.1f NM", (double)nm);
            snprintf(brg, sizeof(brg), "%03.0f", (double)b);
        }
        else
        {
            snprintf(dist, sizeof(dist), "-");
            snprintf(brg, sizeof(brg), "-");
        }
        if (a < 60)
        {
            snprintf(age, sizeof(age), "%us", a);
        }
        else
        {
            snprintf(age, sizeof(age), "%um", a / 60u);
        }
        lv_table_set_cell_value(s_list, row, COL_NAME, name);
        lv_table_set_cell_value(s_list, row, COL_CL, kind_str(&v[i]));
        lv_table_set_cell_value(s_list, row, COL_SOG, sog);
        lv_table_set_cell_value(s_list, row, COL_COG, cog);
        lv_table_set_cell_value(s_list, row, COL_DIST, dist);
        lv_table_set_cell_value(s_list, row, COL_BRG, brg);
        lv_table_set_cell_value(s_list, row, COL_AGE, age);
        shown++;
    }
    /* blank the rows no longer used */
    for (int row = shown + 1; row <= LIST_ROWS; row++)
    {
        for (int col = 0; col < NCOLS; col++)
        {
            lv_table_set_cell_value(s_list, row, col, "");
        }
    }

    if (s_map != NULL && own)
    {
        draw_map(v, n, mlat, mlon);
    }
}

void ais_ui_create(lv_obj_t *parent, int top_y)
{
    s_panel = lv_obj_create(parent);
    lv_obj_remove_style_all(s_panel);
    lv_obj_set_pos(s_panel, 0, top_y);
    lv_obj_set_size(s_panel, PANEL_W, PANEL_H);
    lv_obj_set_style_bg_color(s_panel, lv_color_hex(0x05080C), 0);
    lv_obj_set_style_bg_opa(s_panel, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(s_panel, LV_OBJ_FLAG_SCROLLABLE);

    s_map = heap_caps_calloc(MAP * MAP, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    s_snap = heap_caps_calloc(AIS_MAX_VESSELS, sizeof(ais_vessel_t), MALLOC_CAP_SPIRAM);
    s_canvas = lv_canvas_create(s_panel);
    lv_obj_set_pos(s_canvas, 0, 0);
    if (s_map != NULL)
    {
        lv_canvas_set_buffer(s_canvas, s_map, MAP, MAP, LV_COLOR_FORMAT_RGB565);
    }
    lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_canvas, map_click_cb, LV_EVENT_CLICKED, NULL);
    s_range_lbl = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_range_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_range_lbl, lv_color_hex(0x8090A0), 0);
    lv_obj_set_pos(s_range_lbl, 6, MAP - 18);
    lv_label_set_text(s_range_lbl, "");

    s_status = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_status, lv_color_hex(0x60FF90), 0);
    lv_obj_set_pos(s_status, LIST_X, 4);
    lv_obj_set_width(s_status, PANEL_W - LIST_X - 8);
    lv_label_set_long_mode(s_status, LV_LABEL_LONG_CLIP);

    s_list = lv_table_create(s_panel);
    lv_obj_set_pos(s_list, LIST_X, 26);
    lv_obj_set_size(s_list, PANEL_W - LIST_X - 8, PANEL_H - 28);
    lv_obj_remove_flag(s_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_list, LV_OBJ_FLAG_CLICKABLE);
    /* flat look: no table background or borders, compact rows */
    lv_obj_set_style_bg_opa(s_list, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_list, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_list, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_list, LV_OPA_TRANSP, LV_PART_ITEMS);
    lv_obj_set_style_border_width(s_list, 0, LV_PART_ITEMS);
    lv_obj_set_style_pad_ver(s_list, 2, LV_PART_ITEMS);
    lv_obj_set_style_pad_hor(s_list, 4, LV_PART_ITEMS);
    lv_obj_set_style_text_font(s_list, LIST_FONT, LV_PART_ITEMS);
    lv_obj_set_style_text_color(s_list, lv_color_hex(0xC8D2DC), LV_PART_ITEMS);
    lv_table_set_column_count(s_list, NCOLS);
    lv_table_set_row_count(s_list, LIST_ROWS + 1);
    for (int col = 0; col < NCOLS; col++)
    {
        lv_table_set_column_width(s_list, col, k_col_w[col]);
        lv_table_set_cell_value(s_list, 0, col, k_col_title[col]);
    }
    lv_table_set_cell_value(s_list, 1, COL_NAME, "Listening...");

    lv_obj_add_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
    s_timer = lv_timer_create(timer_cb, 1000, NULL);
}

void ais_ui_show(bool show)
{
    if (s_panel == NULL)
    {
        return;
    }
    if (show)
    {
        lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_panel);
    }
    else
    {
        lv_obj_add_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

bool ais_ui_is_visible(void)
{
    return s_panel != NULL && !lv_obj_has_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
}
