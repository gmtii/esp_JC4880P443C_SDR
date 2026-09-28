#include "ft8_ui.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "sdr.h"
#include "ft8_app.h"
#include "ft8_decoder.h"
#include "ft8_time.h"
#include "ft8_waterfall_adapter.h"
#include "ft8_ram.h" /* FT8_WF_HISTORY_ROWS */
#include "ft8_band_sync.h"

extern int demod_modo;
uint16_t fft_color_map(uint8_t v); /* ui.c - SDR++ "classic" palette LUT */

/* ---- Geometry (panel-relative) ---------------------------------------- */
#define PANEL_W 1024
#define PANEL_H (WAVEFORM_HEIGHT + WATERFALL_HEIGHT) /* 320: exactly the area it covers */

#define STATUS_Y 4
#define SCALE_Y 32
#define CASCADE_Y 50
#define CASCADE_W 1024 /* = FT8_ADAPTER_NUM_BINS (256) * 4 px */
#define CASCADE_H 96   /* rows; one row per 320 ms -> ~30 s (two FT8 slots) */
#define CASCADE_PX_PER_BIN (CASCADE_W / FT8_ADAPTER_NUM_BINS)
#define TEXT_Y_NORMAL (CASCADE_Y + CASCADE_H + 4)
#define TEXT_Y_EXPANDED CASCADE_Y
#define TEXT_X 8

/* Message list: a fixed-width font keeps the decoder's zero-padded columns
 * aligned. unscii_16 is LVGL's monospace font but is off by default - enable
 * it in menuconfig (Component config -> LVGL -> Font usage -> "UNSCII 16")
 * for aligned columns; otherwise a proportional font is used. */
#if defined(LV_FONT_UNSCII_16) && LV_FONT_UNSCII_16
#define FT8_TEXT_FONT (&lv_font_unscii_16)
#define FT8_LINE_PX 18
#elif defined(LV_FONT_MONTSERRAT_16) && LV_FONT_MONTSERRAT_16
#define FT8_TEXT_FONT (&lv_font_montserrat_16)
#define FT8_LINE_PX 20
#else
#define FT8_TEXT_FONT LV_FONT_DEFAULT
#define FT8_LINE_PX 18
#endif

#define LINES_MAX 16

/* Cascade intensity mapping: mag bytes are 2*dB+240 (see decode.h); with the
 * FT8 AGC the noise floor sits around 30-50 and strong signals near 240.
 * Stretch that span over the palette. */
#define CASCADE_V_LOW 30
#define CASCADE_V_HIGH 230

static lv_obj_t *s_panel;
static lv_obj_t *s_status;
static lv_obj_t *s_btn_sync;
static lv_obj_t *s_cascade;
static lv_obj_t *s_text;
static lv_timer_t *s_timer;

static uint16_t *s_casc_buf; /* 2*CASCADE_H rows ring, same trick as ui.c's waterfall */
static int s_casc_head;
static uint32_t s_last_row_counter;

static char s_lines[LINES_MAX][FT8_DECODER_LINE_LEN];
static int s_line_count;       /* valid lines (<= LINES_MAX) */
static int s_line_next;        /* ring write index */
static bool s_text_dirty;
static bool s_expanded;        /* cascade hidden, text uses its space */
static int s_status_div;

static ft8_ui_exit_cb_t s_exit_cb;

void ft8_ui_set_exit_callback(ft8_ui_exit_cb_t cb)
{
    s_exit_cb = cb;
}

/* ---- Message list ------------------------------------------------------- */

static void lines_push(const char *line)
{
    /* Decoder lines are always NUL-terminated within FT8_DECODER_LINE_LEN;
     * snprintf still truncates safely and, unlike strncpy, doesn't trip
     * GCC's -Werror=stringop-truncation. */
    snprintf(s_lines[s_line_next], FT8_DECODER_LINE_LEN, "%s", line);
    s_line_next = (s_line_next + 1) % LINES_MAX;
    if (s_line_count < LINES_MAX)
    {
        s_line_count++;
    }
    s_text_dirty = true;
}

static int visible_lines(void)
{
    int h = PANEL_H - (s_expanded ? TEXT_Y_EXPANDED : TEXT_Y_NORMAL) - 2;
    int n = h / FT8_LINE_PX;
    return (n > LINES_MAX) ? LINES_MAX : n;
}

static void text_redraw(void)
{
    static char buf[LINES_MAX * (FT8_DECODER_LINE_LEN + 1) + 1];
    int n = visible_lines();
    int pos = 0;
    int first;

    if (n > s_line_count)
    {
        n = s_line_count;
    }
    first = (s_line_next - n + LINES_MAX) % LINES_MAX;

    buf[0] = '\0';
    for (int i = 0; i < n; i++)
    {
        const char *l = s_lines[(first + i) % LINES_MAX];
        pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "%s%s", l, (i + 1 < n) ? "\n" : "");
    }
    if (s_line_count == 0)
    {
        snprintf(buf, sizeof(buf), "No decodes yet. Messages appear ~0.5 s after each 15 s slot ends.");
    }
    lv_label_set_text(s_text, buf);
    s_text_dirty = false;
}

static void layout_apply(void)
{
    if (s_expanded)
    {
        lv_obj_add_flag(s_cascade, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(s_text, TEXT_X, TEXT_Y_EXPANDED);
        lv_obj_set_size(s_text, PANEL_W - 2 * TEXT_X, PANEL_H - TEXT_Y_EXPANDED - 2);
    }
    else
    {
        lv_obj_clear_flag(s_cascade, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(s_text, TEXT_X, TEXT_Y_NORMAL);
        lv_obj_set_size(s_text, PANEL_W - 2 * TEXT_X, PANEL_H - TEXT_Y_NORMAL - 2);
        s_last_row_counter = ft8_waterfall_get_row_counter(); /* don't replay stale rows */
    }
    text_redraw();
}

static void toggle_expand_cb(lv_event_t *e)
{
    (void)e;
    s_expanded = !s_expanded;
    layout_apply();
}

/* ---- Cascade ------------------------------------------------------------- */

static uint16_t casc_color(uint8_t v)
{
    int x = ((int)v - CASCADE_V_LOW) * 255 / (CASCADE_V_HIGH - CASCADE_V_LOW);
    if (x < 0)
    {
        x = 0;
    }
    if (x > 255)
    {
        x = 255;
    }
    return fft_color_map((uint8_t)x);
}

static void cascade_push_row(const uint8_t *bins)
{
    s_casc_head = (s_casc_head == 0) ? (CASCADE_H - 1) : (s_casc_head - 1);
    uint16_t *row_a = &s_casc_buf[s_casc_head * CASCADE_W];
    uint16_t *row_b = &s_casc_buf[(s_casc_head + CASCADE_H) * CASCADE_W];

    for (int b = 0; b < FT8_ADAPTER_NUM_BINS; b++)
    {
        uint16_t c = casc_color(bins[b]);
        for (int k = 0; k < CASCADE_PX_PER_BIN; k++)
        {
            row_a[b * CASCADE_PX_PER_BIN + k] = c;
        }
    }
    /* faint 200 Hz grid ticks (every 32 bins) */
    for (int b = 32; b < FT8_ADAPTER_NUM_BINS; b += 32)
    {
        row_a[b * CASCADE_PX_PER_BIN] = 0x4208;
    }
    memcpy(row_b, row_a, CASCADE_W * sizeof(uint16_t));
}

static void cascade_update(void)
{
    uint32_t now = ft8_waterfall_get_row_counter();
    uint32_t diff = now - s_last_row_counter;
    const uint8_t *hist;
    int rows, cols;

    if (diff == 0)
    {
        return;
    }
    s_last_row_counter = now;
    if (diff > FT8_WF_HISTORY_ROWS)
    {
        diff = FT8_WF_HISTORY_ROWS; /* fell far behind: show what history still has */
    }

    ft8_waterfall_get_history(&hist, &rows, &cols);
    for (int r = (int)diff - 1; r >= 0; r--) /* oldest missed row first */
    {
        cascade_push_row(&hist[r * cols]);
    }

    lv_canvas_set_buffer(s_cascade, &s_casc_buf[s_casc_head * CASCADE_W], CASCADE_W, CASCADE_H, LV_COLOR_FORMAT_RGB565);
    lv_obj_invalidate(s_cascade);
}

/* ---- Status line ---------------------------------------------------------- */

static void status_update(void)
{
    ft8_status_t st;
    struct tm t;
    char clk[40];
    char sync[48];
    char buf[220];
    lv_color_t col = lv_color_hex(0xC8D2DC);

    ft8_app_get_status(&st);
    ft8_time_get_utc(&t);

    if (ft8_time_is_synced())
    {
        snprintf(clk, sizeof(clk), "%02d:%02d:%02d UTC (%s)", t.tm_hour, t.tm_min, t.tm_sec,
                 ft8_time_source_name(ft8_time_get_source()));
    }
    else
    {
        snprintf(clk, sizeof(clk), "clock not set");
    }

    switch ((ft8_bsync_state_t)st.sync_state)
    {
    case FT8_BSYNC_LOCKED:
        snprintf(sync, sizeof(sync), "LOCKED dt%+.2fs", st.sync_err_s);
        col = lv_color_hex(0x60FF90);
        break;
    case FT8_BSYNC_SYNCING:
        snprintf(sync, sizeof(sync), "SYNCING dt%+.2fs", st.sync_err_s);
        col = lv_color_hex(0xFFE060);
        break;
    case FT8_BSYNC_HEARD:
        snprintf(sync, sizeof(sync), "HEARD, NO DECODE - searching slot");
        col = lv_color_hex(0xFFA040);
        break;
    default:
        snprintf(sync, sizeof(sync), "%s", ft8_time_is_synced() ? "SEARCHING (band quiet?)" : "SEARCHING - sweeping slot");
        col = lv_color_hex(0xFFA040);
        break;
    }

    if (st.capturing)
    {
        snprintf(buf, sizeof(buf), "FT8  %s  |  %s  |  sym %2d/%d  |  last %d dec %d cand %lu ms  |  total %lu%s",
                 clk, sync, st.blocks, FT8_ADAPTER_MAX_BLOCKS,
                 st.last_decoded, st.last_candidates, (unsigned long)st.last_proc_ms,
                 (unsigned long)st.total_decodes, st.overruns ? "  AUDIO OVERRUN" : "");
    }
    else
    {
        snprintf(buf, sizeof(buf), "FT8  %s  |  %s  |  waiting for the next slot boundary...", clk, sync);
    }
    lv_obj_set_style_text_color(s_status, col, 0);
    lv_label_set_text(s_status, buf);
}

/* ---- TIME popup: set the clock "by eye" -------------------------------------
 * Type HHMMSS (or HHMM) in UTC and press OK when a reference clock reaches it;
 * a second or two off is fine - the band-sync loop removes the rest.
 * "Sync slot" snaps to the nearest 15 s boundary: press it when a reference
 * clock shows :00, :15, :30 or :45 (the band-sync loop needs nothing more). */
static lv_obj_t *s_time_popup;
static lv_obj_t *s_time_ta;

static const char *k_time_map[] = {"1", "2", "3", "\n",
                                   "4", "5", "6", "\n",
                                   "7", "8", "9", "\n",
                                   LV_SYMBOL_BACKSPACE, "0", LV_SYMBOL_OK, "\n",
                                   "Sync slot", LV_SYMBOL_CLOSE, ""};

static void time_popup_close(void)
{
    if (s_time_popup != NULL)
    {
        lv_obj_delete(s_time_popup);
        s_time_popup = NULL;
        s_time_ta = NULL;
    }
}

static bool time_popup_apply(const char *s)
{
    size_t n = strlen(s);
    int h, m, sec = 0;

    if (n != 4 && n != 6)
    {
        return false;
    }
    h = (s[0] - '0') * 10 + (s[1] - '0');
    m = (s[2] - '0') * 10 + (s[3] - '0');
    if (n == 6)
    {
        sec = (s[4] - '0') * 10 + (s[5] - '0');
    }
    if (h > 23 || m > 59 || sec > 59)
    {
        return false;
    }
    ft8_time_set_time_of_day(h, m, sec);
    return true;
}

static void time_btnm_cb(lv_event_t *e)
{
    lv_obj_t *bm = lv_event_get_target_obj(e);
    const char *txt = lv_buttonmatrix_get_button_text(bm, lv_buttonmatrix_get_selected_button(bm));

    if (txt == NULL || s_time_ta == NULL)
    {
        return;
    }
    if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0)
    {
        lv_textarea_delete_char(s_time_ta);
    }
    else if (strcmp(txt, LV_SYMBOL_CLOSE) == 0)
    {
        time_popup_close();
    }
    else if (strcmp(txt, "Sync slot") == 0)
    {
        ft8_time_manual_sync_slot();
        time_popup_close();
        status_update();
    }
    else if (strcmp(txt, LV_SYMBOL_OK) == 0)
    {
        if (time_popup_apply(lv_textarea_get_text(s_time_ta)))
        {
            time_popup_close();
            status_update();
        }
        else
        {
            lv_textarea_set_text(s_time_ta, ""); /* invalid: clear and let the user retype */
        }
    }
    else if (txt[0] >= '0' && txt[0] <= '9' && txt[1] == '\0')
    {
        lv_textarea_add_text(s_time_ta, txt);
    }
}

static void time_popup_open(void)
{
    lv_obj_t *title, *bm;

    if (s_time_popup != NULL)
    {
        return;
    }
    s_time_popup = lv_obj_create(lv_screen_active());
    lv_obj_set_size(s_time_popup, 420, 440);
    lv_obj_center(s_time_popup);
    lv_obj_set_style_bg_color(s_time_popup, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_time_popup, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_time_popup, 12, 0);
    lv_obj_remove_flag(s_time_popup, LV_OBJ_FLAG_SCROLLABLE);

    title = lv_label_create(s_time_popup);
    lv_label_set_text(title, "HHMMSS UTC + OK, or Sync slot at :00/:15/:30/:45");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

    s_time_ta = lv_textarea_create(s_time_popup);
    lv_textarea_set_one_line(s_time_ta, true);
    lv_textarea_set_max_length(s_time_ta, 6);
    lv_textarea_set_accepted_chars(s_time_ta, "0123456789");
    lv_textarea_set_placeholder_text(s_time_ta, "HHMMSS");
    lv_obj_set_width(s_time_ta, lv_pct(100));
    lv_obj_align(s_time_ta, LV_ALIGN_TOP_MID, 0, 28);

    bm = lv_buttonmatrix_create(s_time_popup);
    lv_buttonmatrix_set_map(bm, k_time_map);
    lv_obj_set_size(bm, lv_pct(100), 300);
    lv_obj_align(bm, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_event_cb(bm, time_btnm_cb, LV_EVENT_VALUE_CHANGED, NULL);
}

static void time_btn_cb(lv_event_t *e)
{
    (void)e;
    time_popup_open();
}

/* ---- Periodic timer -------------------------------------------------------- */

static void ft8_ui_timer_cb(lv_timer_t *t)
{
    (void)t;
    ft8_decoded_msg_t msg;

    if (!ft8_ui_is_visible())
    {
        return;
    }

    /* Mode changed away from USB somewhere else (menu, MODE button): leave FT8. */
    if (demod_modo != DEMOD_USB)
    {
        ft8_app_set_active(false);
        ft8_ui_show(false);
        if (s_exit_cb != NULL)
        {
            s_exit_cb();
        }
        return;
    }

    while (ft8_decoder_get_message(&msg))
    {
        lines_push(msg.line);
    }
    if (s_text_dirty)
    {
        text_redraw();
    }

    if (!s_expanded)
    {
        cascade_update();
    }

    if (++s_status_div >= 5) /* 500 ms */
    {
        s_status_div = 0;
        status_update();
    }
}

/* ---- Creation --------------------------------------------------------------- */

static void make_scale(void)
{
    for (int f = 0; f <= 1600; f += 200)
    {
        lv_obj_t *l = lv_label_create(s_panel);
        int x = (int)((float)f / 6.25f) * CASCADE_PX_PER_BIN;
        lv_obj_set_style_text_color(l, lv_color_hex(0x8090A0), 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
        lv_label_set_text_fmt(l, "%d", f);
        if (f == 1600)
        {
            lv_label_set_text(l, "1600 Hz");
            x -= 48;
        }
        else if (f > 0)
        {
            x -= 10;
        }
        lv_obj_set_pos(l, x < 2 ? 2 : x, SCALE_Y);
    }
}

void ft8_ui_create(lv_obj_t *parent, int top_y)
{
    s_panel = lv_obj_create(parent);
    lv_obj_remove_style_all(s_panel);
    lv_obj_set_pos(s_panel, 0, top_y);
    lv_obj_set_size(s_panel, PANEL_W, PANEL_H);
    lv_obj_set_style_bg_color(s_panel, lv_color_hex(0x05080C), 0);
    lv_obj_set_style_bg_opa(s_panel, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_panel, LV_OBJ_FLAG_CLICKABLE); /* swallow taps meant for the canvases below */
    lv_obj_remove_flag(s_panel, LV_OBJ_FLAG_SCROLLABLE);

    s_status = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_status, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(s_status, 8, STATUS_Y + 4);
    lv_obj_set_width(s_status, PANEL_W - 140);
    lv_label_set_long_mode(s_status, LV_LABEL_LONG_CLIP);
    lv_label_set_text(s_status, "FT8");

    s_btn_sync = lv_btn_create(s_panel);
    lv_obj_set_size(s_btn_sync, 110, 26);
    lv_obj_set_pos(s_btn_sync, PANEL_W - 118, STATUS_Y);
    lv_obj_set_style_bg_color(s_btn_sync, lv_color_hex(0x0D4A8F), 0);
    lv_obj_add_event_cb(s_btn_sync, time_btn_cb, LV_EVENT_CLICKED, NULL);
    {
        lv_obj_t *l = lv_label_create(s_btn_sync);
        lv_label_set_text(l, "TIME");
        lv_obj_center(l);
    }

    make_scale();

    s_casc_buf = heap_caps_calloc(2u * CASCADE_W * CASCADE_H, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    s_cascade = lv_canvas_create(s_panel);
    lv_obj_remove_style_all(s_cascade);
    lv_obj_set_pos(s_cascade, 0, CASCADE_Y);
    if (s_casc_buf != NULL)
    {
        lv_canvas_set_buffer(s_cascade, s_casc_buf, CASCADE_W, CASCADE_H, LV_COLOR_FORMAT_RGB565);
    }
    else
    {
        ESP_LOGE("FT8UI", "cascade buffer allocation failed");
    }
    lv_obj_add_flag(s_cascade, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(s_cascade, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_cascade, toggle_expand_cb, LV_EVENT_CLICKED, NULL);

    s_text = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_text, FT8_TEXT_FONT, 0);
    lv_obj_set_style_text_color(s_text, lv_color_hex(0x00FF60), 0);
    lv_obj_set_style_text_line_space(s_text, FT8_LINE_PX - lv_font_get_line_height(FT8_TEXT_FONT), 0);
    lv_label_set_long_mode(s_text, LV_LABEL_LONG_CLIP);
    lv_obj_add_flag(s_text, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_text, toggle_expand_cb, LV_EVENT_CLICKED, NULL);

    layout_apply();

    lv_obj_add_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
    s_timer = lv_timer_create(ft8_ui_timer_cb, 100, NULL);
}

void ft8_ui_show(bool show)
{
    if (s_panel == NULL || s_casc_buf == NULL)
    {
        return;
    }
    if (show)
    {
        s_last_row_counter = ft8_waterfall_get_row_counter();
        s_status_div = 5; /* refresh status on the first tick */
        lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_panel);
        text_redraw();
    }
    else
    {
        time_popup_close();
        lv_obj_add_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

bool ft8_ui_is_visible(void)
{
    return s_panel != NULL && !lv_obj_has_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
}
