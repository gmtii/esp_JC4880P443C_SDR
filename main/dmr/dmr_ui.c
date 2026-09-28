#include "dmr_ui.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "sdr.h"
#include "dmr_app.h"
#include "dmr_demod.h"
#include "dmr_proto.h"
#include "dmr_voice.h"
#include "ft8_time.h" /* shared UTC clock for the call log */
#include "ft8_decoder.h" /* own QTH (grid) and distance, shared with FT8 */

extern int demod_modo;

#define PANEL_W 1024
#define PANEL_H (WAVEFORM_HEIGHT + WATERFALL_HEIGHT) /* 320 */
#define CARD_W 500
#define CARD_H 150
#define CARD_Y 30
#define LOG_Y (CARD_Y + CARD_H + 8)
#define STALE_MS 1500 /* a slot with no valid burst for this long is shown as quiet */

#if defined(LV_FONT_MONTSERRAT_46) && LV_FONT_MONTSERRAT_46
#define BIG_FONT (&lv_font_montserrat_46)
#else
#define BIG_FONT (&lv_font_montserrat_18)
#endif
#if defined(LV_FONT_UNSCII_16) && LV_FONT_UNSCII_16
#define LOG_FONT (&lv_font_unscii_16)
#else
#define LOG_FONT (&lv_font_montserrat_14)
#endif

typedef struct
{
    lv_obj_t *card, *title, *big, *src, *alias;
} card_t;

static lv_obj_t *s_panel;
static lv_obj_t *s_status;
static lv_obj_t *s_log;
static card_t s_card[2];
static uint32_t s_log_seq_shown;
static dmr_ui_exit_cb_t s_exit_cb;

void dmr_ui_set_exit_callback(dmr_ui_exit_cb_t cb)
{
    s_exit_cb = cb;
}

/* "IL18qg 23 km" (distance only when the own grid is set) */
static void fmt_gps(bool valid, float lat, float lon, const char *grid, char *out, size_t len)
{
    float mlat, mlon;
    if (!valid)
    {
        out[0] = '\0';
        return;
    }
    if (ft8_decoder_get_own_latlon(&mlat, &mlon))
    {
        snprintf(out, len, "%s %.0f km", grid, ft8_distance_km(mlat, mlon, lat, lon));
    }
    else
    {
        snprintf(out, len, "%s", grid);
    }
}

static const char *act_name(dmr_activity_t a)
{
    switch (a)
    {
    case DMR_ACT_VOICE: return "VOICE";
    case DMR_ACT_IDLE: return "IDLE";
    case DMR_ACT_DATA: return "DATA";
    case DMR_ACT_CSBK: return "CSBK";
    default: return "---";
    }
}

static lv_color_t act_color(dmr_activity_t a, bool stale)
{
    if (stale)
    {
        return lv_color_hex(0x303840);
    }
    switch (a)
    {
    case DMR_ACT_VOICE: return lv_color_hex(0x30D060);
    case DMR_ACT_DATA:
    case DMR_ACT_CSBK: return lv_color_hex(0x3080E0);
    case DMR_ACT_IDLE: return lv_color_hex(0x607080);
    default: return lv_color_hex(0x303840);
    }
}

/* stream time (demodulator sample clock) -> "HH:MM:SS" on the shared clock */
static void fmt_time(uint32_t t_ms, uint32_t now_stream_ms, char *out, size_t len)
{
    int64_t wall = ft8_time_now_ms() - (int64_t)(now_stream_ms - t_ms);
    time_t s = (time_t)(wall / 1000);
    struct tm tm;
    gmtime_r(&s, &tm);
    snprintf(out, len, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
}

static void card_update(int idx, const dmr_demod_status_t *ds, bool ms)
{
    dmr_slot_info_t si;
    card_t *c = &s_card[idx];
    bool stale;
    char buf[64];

    dmr_proto_get_slot(idx, &si);
    stale = !ds->locked || si.last_ms == 0 || (ds->now_ms - si.last_ms) > STALE_MS;

    if (ms && idx == 1)
    {
        lv_label_set_text(c->title, "TS2  (not used in simplex)");
        lv_label_set_text(c->big, "");
        lv_label_set_text(c->src, "");
        lv_label_set_text(c->alias, "");
        lv_obj_set_style_border_color(c->card, lv_color_hex(0x202830), 0);
        return;
    }

    if (si.cc >= 0)
    {
        snprintf(buf, sizeof(buf), "%s   %s   CC %d%s%s%s", ms ? "MS simplex" : (idx ? "TS2" : "TS1"),
                 stale ? "---" : act_name(si.activity), si.cc,
                 si.encrypted ? "   ENC" : "",
                 dmr_voice_get_playing() == idx ? "   " LV_SYMBOL_VOLUME_MAX : "",
                 dmr_voice_get_pref() == idx ? "   (fixed)" : "");
    }
    else
    {
        snprintf(buf, sizeof(buf), "%s   ---", ms ? "MS simplex" : (idx ? "TS2" : "TS1"));
    }
    lv_label_set_text(c->title, buf);
    lv_obj_set_style_border_color(c->card, act_color(si.activity, stale), 0);

    if (si.ids_valid)
    {
        lv_label_set_text_fmt(c->big, "%s %u", si.group ? "TG" : "ID", (unsigned)si.dst);
        if (!stale && si.activity == DMR_ACT_VOICE)
        {
            lv_label_set_text_fmt(c->src, "SRC %u   %lu s", (unsigned)si.src,
                                  (unsigned long)((ds->now_ms - si.call_start_ms) / 1000u));
        }
        else
        {
            lv_label_set_text_fmt(c->src, "SRC %u   (last)", (unsigned)si.src);
        }
        if (si.gps_valid)
        {
            char g[32];
            fmt_gps(true, si.gps_lat, si.gps_lon, si.gps_grid, g, sizeof(g));
            lv_label_set_text_fmt(c->alias, "%s%s" LV_SYMBOL_GPS " %s", si.alias, si.alias[0] ? "   " : "", g);
        }
        else
        {
            lv_label_set_text(c->alias, si.alias);
        }
    }
    else
    {
        lv_label_set_text(c->big, "");
        lv_label_set_text(c->src, "");
        lv_label_set_text(c->alias, "");
    }
    lv_obj_set_style_text_color(c->big, stale ? lv_color_hex(0x708090) : lv_color_white(), 0);
}

static void log_update(const dmr_demod_status_t *ds)
{
    dmr_log_entry_t e[DMR_LOG_LEN];
    int n = dmr_proto_get_log(e);
    static char buf[DMR_LOG_LEN * 96];
    int pos = 0;
    const int show = 6;

    if (n == 0 || e[n - 1].seq == s_log_seq_shown)
    {
        if (n == 0)
        {
            lv_label_set_text(s_log, "Call log: no calls heard yet.");
        }
        return;
    }
    s_log_seq_shown = e[n - 1].seq;
    for (int i = n - 1; i >= 0 && i >= n - show; i--) /* newest first */
    {
        char t[12];
        fmt_time(e[i].t_ms, ds->now_ms, t, sizeof(t));
        char g[32];
        fmt_gps(e[i].gps_valid, e[i].gps_lat, e[i].gps_lon, e[i].gps_grid, g, sizeof(g));
        pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "%s  %s  %s %-8u  SRC %-8u %s%s%s%s", t,
                        e[i].slot ? (e[i].slot == 1 ? "TS1" : "TS2") : "MS ", e[i].group ? "TG" : "ID",
                        (unsigned)e[i].dst, (unsigned)e[i].src, e[i].alias, g[0] ? "  " : "", g,
                        i > n - show && i > 0 ? "\n" : "");
    }
    lv_label_set_text(s_log, buf);
}

static void status_update(const dmr_demod_status_t *ds)
{
    char buf[160];
    if (ds->locked)
    {
        int pref = dmr_voice_get_pref();
        uint32_t drops, und, frames, us;
        dmr_app_get_stats(&drops, &und);
        dmr_voice_get_timing(&frames, &us);
        if (dmr_voice_available())
        {
            snprintf(buf, sizeof(buf), "DMR %s%s | off %+.0f Hz dev %.0f | voice %s | voc %.1f ms drop %lu und %lu conc %lu",
                     ds->bs ? "BS" : ds->ms ? "MS" : "DM", ds->inverted ? " INV" : "", ds->dc, ds->level,
                     pref == 0 ? "TS1" : pref == 1 ? "TS2" : "auto", us / 1000.0f, (unsigned long)drops, (unsigned long)und,
                     (unsigned long)dmr_voice_get_concealed());
        }
        else
        {
            snprintf(buf, sizeof(buf), "DMR   LOCKED %s%s   |   offset %+.0f Hz   dev %.0f Hz   |   voice: off (no mbelib)",
                     ds->bs ? "repeater (BS)" : ds->ms ? "simplex (MS)" : "direct mode", ds->inverted ? "  INVERTED" : "",
                     ds->dc, ds->level);
        }
        lv_obj_set_style_text_color(s_status, lv_color_hex(0x60FF90), 0);
    }
    else
    {
        snprintf(buf, sizeof(buf), "DMR   searching...   (tune to the repeater output / simplex frequency, NFM)");
        lv_obj_set_style_text_color(s_status, lv_color_hex(0xFFA040), 0);
    }
    lv_label_set_text(s_status, buf);
}

static void timer_cb(lv_timer_t *t)
{
    dmr_demod_status_t ds;
    bool ms;
    (void)t;

    if (!dmr_ui_is_visible())
    {
        return;
    }
    if (demod_modo != DEMOD_FM)
    {
        /* mode changed elsewhere (menu, MODE button): leave DMR */
        dmr_app_set_active(false);
        dmr_ui_show(false);
        if (s_exit_cb != NULL)
        {
            s_exit_cb();
        }
        return;
    }
    dmr_demod_get_status(&ds);
    ms = dmr_proto_is_ms_mode();
    status_update(&ds);
    card_update(0, &ds, ms);
    card_update(1, &ds, ms);
    log_update(&ds);
}

/* Tap a slot card: listen to that slot only; tap it again: back to auto. */
static void card_click_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    dmr_voice_set_pref(dmr_voice_get_pref() == idx ? DMR_VOICE_PREF_AUTO : idx);
}

static lv_obj_t *mk_label(lv_obj_t *parent, const lv_font_t *font, int x, int y, lv_color_t col)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, col, 0);
    lv_obj_set_pos(l, x, y);
    lv_label_set_text(l, "");
    return l;
}

void dmr_ui_create(lv_obj_t *parent, int top_y)
{
    s_panel = lv_obj_create(parent);
    lv_obj_remove_style_all(s_panel);
    lv_obj_set_pos(s_panel, 0, top_y);
    lv_obj_set_size(s_panel, PANEL_W, PANEL_H);
    lv_obj_set_style_bg_color(s_panel, lv_color_hex(0x05080C), 0);
    lv_obj_set_style_bg_opa(s_panel, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(s_panel, LV_OBJ_FLAG_SCROLLABLE);

    s_status = mk_label(s_panel, &lv_font_montserrat_14, 8, 6, lv_color_hex(0xC8D2DC));

    for (int i = 0; i < 2; i++)
    {
        card_t *c = &s_card[i];
        c->card = lv_obj_create(s_panel);
        lv_obj_remove_style_all(c->card);
        lv_obj_set_pos(c->card, 8 + i * (CARD_W + 8), CARD_Y);
        lv_obj_set_size(c->card, CARD_W, CARD_H);
        lv_obj_set_style_bg_color(c->card, lv_color_hex(0x0C1218), 0);
        lv_obj_set_style_bg_opa(c->card, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(c->card, 3, 0);
        lv_obj_set_style_border_color(c->card, lv_color_hex(0x303840), 0);
        lv_obj_set_style_radius(c->card, 8, 0);
        lv_obj_remove_flag(c->card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(c->card, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(c->card, card_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        c->title = mk_label(c->card, &lv_font_montserrat_18, 12, 8, lv_color_hex(0xC8D2DC));
        c->big = mk_label(c->card, BIG_FONT, 12, 34, lv_color_white());
        c->src = mk_label(c->card, &lv_font_montserrat_18, 12, 90, lv_color_hex(0xC8D2DC));
        c->alias = mk_label(c->card, &lv_font_montserrat_18, 12, 116, lv_color_hex(0x60FF90));
    }

    s_log = mk_label(s_panel, LOG_FONT, 8, LOG_Y, lv_color_hex(0xB0C0D0));
    lv_obj_set_size(s_log, PANEL_W - 16, PANEL_H - LOG_Y - 2);
    lv_label_set_long_mode(s_log, LV_LABEL_LONG_CLIP);

    lv_obj_add_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
    lv_timer_create(timer_cb, 250, NULL);
}

void dmr_ui_show(bool show)
{
    if (s_panel == NULL)
    {
        return;
    }
    if (show)
    {
        s_log_seq_shown = 0;
        lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_panel);
    }
    else
    {
        lv_obj_add_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

bool dmr_ui_is_visible(void)
{
    return s_panel != NULL && !lv_obj_has_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
}
