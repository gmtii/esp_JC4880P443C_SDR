#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "bsp_board_extra.h"

#include "sdr.h"
#include "ui.h"
#include "agc.h"
#include "audio_out.h"
#include "smeter.h"
#include "math.h"

#include "lvgl.h"

#include "rtl_source.h"

#include "palettes.h"
#include "menu.h"

#include "lvgl.h"
#include <stdio.h>
#include <stdbool.h>

extern int demod_modo;
extern int f_nrss;
extern agc_wdsp_params_t agc_wdsp_conf;
extern bool screen_update;

extern int pasos[6];
extern int pasos_indice;

extern bool f_actualiza;
extern int filtro_indice;
extern char *filtros_texto[5];
extern uint8_t spec_smooth_passes; /* "SPT" - see sdr_priv.h's comment */
#define SPECTRUM_LINE_SMOOTH_MAX 5
extern char *agc_texto[6];

/* =========================================================
 * VARIABLES DE LA APLICACIÓN
 * ========================================================= */
static int32_t var_slider1 = 50; /* output volume, percent */
static int32_t var_slider2 = 30; /* RTL tuner gain, dB */

int menu_get_rtl_gain_db(void) { return (int)var_slider2; }
static int32_t var_slider4 = 5;

static bool var_btn1 = false;
static bool var_btn2 = false;
static bool var_btn3 = false;
static bool var_btn4 = false;
static bool var_btn5 = false;
static bool var_btn6 = false;
static bool var_btn7 = false;
static bool var_btn8 = false;
static bool var_btn9 = false;
static bool var_btn10 = false;
static bool var_btn11 = false;
static bool var_btn12 = false;

/* Labels para mostrar valores de sliders */
static lv_obj_t *lbl_s1;
static lv_obj_t *lbl_s2;
static lv_obj_t *lbl_s4;

/* ===== Objetos LVGL del menú (separados) ===== */
static lv_obj_t *cont_menu = NULL; /* contenedor principal */
static lv_obj_t *menu_scr = NULL;  /* pantalla activa cuando se crea el menú */

static lv_obj_t *row_sliders = NULL;
static lv_obj_t *grid_btns = NULL;

/* Bloques/Sliders */
static lv_obj_t *box_s1 = NULL;
static lv_obj_t *sl_s1 = NULL;
static lv_obj_t *box_s2 = NULL;
static lv_obj_t *sl_s2 = NULL;
static lv_obj_t *box_s4 = NULL;
static lv_obj_t *sl_s4 = NULL;

/* Botones (si necesitas manipularlos después) */
static lv_obj_t *btn1_usb = NULL;
static lv_obj_t *btn2_lsb = NULL;
static lv_obj_t *btn3_am = NULL;
static lv_obj_t *btn4_sam = NULL;
static lv_obj_t *btn5_samu = NULL;
static lv_obj_t *btn6_saml = NULL;
static lv_obj_t *btn7_nr = NULL;
static lv_obj_t *btn8_filtro = NULL;
static lv_obj_t *btn9_agc = NULL;
static lv_obj_t *btn10 = NULL;
static lv_obj_t *btn11 = NULL;
static lv_obj_t *btn12_close = NULL;

void inicia_timers(void)
{
    /* Evita crear timers duplicados (causa típica de ralentización) */
    if (!timer_pantalla)
        timer_pantalla = lv_timer_create(timer_dibuja_pantalla, 33, NULL);
    else
        lv_timer_resume(timer_pantalla);

    if (!timer_smeter)
        timer_smeter = lv_timer_create(timer_smeter_update, 33, NULL);
    else
        lv_timer_resume(timer_smeter);
}

/* =========================================================
 * CALLBACKS SLIDERS
 * ========================================================= */
static void slicer_dac_volume(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED)
        return;
    lv_obj_t *sl = lv_event_get_target(e);

    var_slider1 = lv_slider_get_value(sl);
    lv_label_set_text_fmt(lbl_s1, "Slider 1: %ld", (long)var_slider1);

    audio_out_set_volume((int)var_slider1);
}

static void slider_rtl_gain(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED)
        return;
    lv_obj_t *sl = lv_event_get_target(e);

    var_slider2 = lv_slider_get_value(sl);
    lv_label_set_text_fmt(lbl_s2, "Slider 2: %ld", (long)var_slider2);

    rtl_source_set_gain_db(var_slider2);
}

static void slider_brightness(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED)
        return;
    lv_obj_t *sl = lv_event_get_target(e);

    var_slider4 = lv_slider_get_value(sl);
    lv_label_set_text_fmt(lbl_s4, "Slider 4: %ld", (long)var_slider4);

    bsp_display_brightness_set(var_slider4);

    // hw_set_param3(var_slider3);
}

/* =========================================================
 * CALLBACKS BOTONES
 * ========================================================= */
static void btn1_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn1 = !var_btn1;
    demod_modo = DEMOD_USB;
    dibuja_pasabanda();
}

static void btn2_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn2 = !var_btn2;
    demod_modo = DEMOD_LSB;
    dibuja_pasabanda();
}

static void btn3_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn3 = !var_btn3;
    demod_modo = DEMOD_AM;
    dibuja_pasabanda();
}

static void btn4_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn4 = !var_btn4;
    demod_modo = DEMOD_SAM;
    dibuja_pasabanda();
}

static void btn5_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn5 = !var_btn5;
    demod_modo = DEMOD_SAMU;
    dibuja_pasabanda();
}

static void btn6_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn6 = !var_btn6;
    demod_modo = DEMOD_SAML;
    dibuja_pasabanda();
}

static void btn7_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn7 = !var_btn7;

    if (!f_nrss)
        f_nrss = true;
    else
        f_nrss = false;
}

/*
 * "SPT" - spectrum spatial-line smoothing cycle (0..SPECTRUM_LINE_SMOOTH_MAX), per
 * Jorge, ported from a sibling GD32F450 SDR project (see sdr_priv.h's
 * spec_smooth_passes comment for the DSP side). Repurposed from this slot's old
 * FILTRO job: main screen's own btn_filtros (ui.c) already does the exact same
 * filtro_indice cycling, making this menu-grid copy redundant.
 */
static void btn8_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn8 = !var_btn8;

    spec_smooth_passes = (uint8_t)((spec_smooth_passes + 1U) % (SPECTRUM_LINE_SMOOTH_MAX + 1U));

    lv_obj_t *lbl = lv_obj_get_child(btn8_filtro, 0);
    if (lbl != NULL)
    {
        lv_label_set_text_fmt(lbl, "SPT %d", (int)spec_smooth_passes);
    }
}

static void btn9_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn9 = !var_btn9;

    agc_wdsp_conf.AGC_mode++;
    if (agc_wdsp_conf.AGC_mode > 5)
        agc_wdsp_conf.AGC_mode = 0;

    agc_wdsp_conf.agc_switch_mode = 1;
    AGC_prep();
}

/*
 * NFM / WFM. Mirrors the dedicated FM button on the main screen (ui.c's btn4_cb):
 * WFM tunes on-frequency (no 12 kHz offset) and defaults the tuner to AGC, since
 * broadcast FM carriers are always strong; leaving it restores the RTL gain
 * slider's manual value. Kept as two separate buttons here (rather than one
 * toggle like ui.c's) since the menu has room and it avoids a relabelling step.
 */
static void btn10_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn10 = !var_btn10;

    demod_modo = DEMOD_FM;
    currentVFO.demod_modo = demod_modo;
    rtl_source_set_freq(currentVFO.Frec - lo_offset_for_mode(demod_modo));
    rtl_source_set_gain_db(var_slider2);

    dibuja_pasabanda();
    refresca_indicadores();
}

static void btn11_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    var_btn11 = !var_btn11;

    demod_modo = DEMOD_WFM;
    currentVFO.demod_modo = demod_modo;
    rtl_source_set_freq(currentVFO.Frec - lo_offset_for_mode(demod_modo));
    rtl_source_set_gain_auto(true);

    dibuja_pasabanda();
    refresca_indicadores();
}

/*
 * PALETTE: cycles the waterfall / FT8-cascade color palette through the full
 * SDR++ set (palettes.c). Applied live and saved in NVS by ui_set_palette().
 */
static lv_obj_t *btn13_palette = NULL;

static void palette_btn_label(void)
{
    lv_obj_t *lbl = (btn13_palette != NULL) ? lv_obj_get_child(btn13_palette, 0) : NULL;
    if (lbl != NULL)
    {
        lv_label_set_text_fmt(lbl, "PALETTE\n%s", palette_name(ui_get_palette()));
    }
}

/* AIS: enter/leave the AIS receiver (ui.c: 162.000 MHz, WFM wide path). */
static lv_obj_t *btn14_ais = NULL;

static void ais_btn_label(void)
{
    lv_obj_t *lbl = (btn14_ais != NULL) ? lv_obj_get_child(btn14_ais, 0) : NULL;
    if (lbl != NULL)
    {
        lv_label_set_text(lbl, ui_ais_is_active() ? "AIS\nON" : "AIS\nOFF");
    }
}

static void btn14_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    ui_ais_toggle();
}

static void btn13_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    ui_set_palette((ui_get_palette() + 1) % PALETTE_COUNT);
    palette_btn_label();
}

/* =========================================================
 * BOTÓN CERRAR (DESTRUYE EL MENÚ)
 * ========================================================= */

static void btn12_cb(lv_event_t *e)
{

    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;

    if (cont_menu && lv_obj_is_valid(cont_menu))
    {
        lv_obj_del_async(cont_menu);

        cont_menu = NULL;

        /* Limpia referencias para evitar punteros colgantes */
        menu_scr = NULL;
        row_sliders = NULL;
        grid_btns = NULL;
        btn13_palette = NULL;
        btn14_ais = NULL;
        ui_ais_set_state_callback(NULL);

        box_s1 = sl_s1 = NULL;
        box_s2 = sl_s2 = NULL;
        box_s4 = sl_s4 = NULL;

        /* Labels quedan invalidados al borrar cont_menu, pero los punteros se limpian igualmente */
        lbl_s1 = lbl_s2 = lbl_s4 = NULL;

        btn1_usb = btn2_lsb = btn3_am = btn4_sam = NULL;
        btn5_samu = btn6_saml = btn7_nr = btn8_filtro = NULL;
        btn9_agc = btn10 = btn11 = btn12_close = NULL;

        inicia_timers();
        refresca_indicadores();
    }
}

/* =========================================================
 * HELPERS UI
 * ========================================================= */
static void create_slider_block(lv_obj_t *parent,
                                const char *title,
                                int32_t min, int32_t max, int32_t init,
                                lv_event_cb_t cb,
                                lv_obj_t **out_box,
                                lv_obj_t **out_slider,
                                lv_obj_t **out_label)
{
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_set_size(box, 225, 120);
    lv_obj_set_style_pad_all(box, 12, 0);
    lv_obj_set_flex_flow(box, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(box,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_t *t = lv_label_create(box);
    lv_label_set_text(t, title);

    lv_obj_t *sl = lv_slider_create(box);
    lv_obj_set_width(sl, lv_pct(100));
    lv_slider_set_range(sl, min, max);
    lv_slider_set_value(sl, init, LV_ANIM_OFF);
    lv_obj_add_event_cb(sl, cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *val = lv_label_create(box);
    lv_label_set_text_fmt(val, "Slider: %ld", (long)init);

    if (out_box)
        *out_box = box;
    if (out_slider)
        *out_slider = sl;
    if (out_label)
        *out_label = val;
}

static lv_obj_t *create_button(lv_obj_t *parent,
                               const char *txt,
                               lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 140, 60);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_set_style_bg_color(btn, lv_color_hex(0x202020), LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x404040), LV_PART_MAIN);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, txt);
    lv_obj_center(lbl);

    return btn;
}

/* =========================================================
 * FUNCIÓN PRINCIPAL PARA CREAR EL MENÚ
 * ========================================================= */
void ui_create_control_panel(void)
{
    if (cont_menu)
        return; /* evita crear dos veces */

    menu_scr = lv_scr_act();

    /* ---------- Contenedor principal ---------- */
    cont_menu = lv_obj_create(menu_scr);
    lv_obj_set_size(cont_menu, 1024, 500);
    lv_obj_center(cont_menu);
    lv_obj_set_style_pad_all(cont_menu, 16, 0);

    lv_obj_set_style_bg_color(cont_menu, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(cont_menu, LV_OPA_COVER, 0);

    lv_obj_remove_flag(cont_menu, LV_OBJ_FLAG_SCROLLABLE);

    /* ---------- Layout general ---------- */
    lv_obj_set_flex_flow(cont_menu, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont_menu,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);

    /* ---------- Fila de sliders ---------- */
    row_sliders = lv_obj_create(cont_menu);
    lv_obj_set_width(row_sliders, lv_pct(100));
    lv_obj_set_height(row_sliders, 190);
    lv_obj_set_style_pad_all(row_sliders, 0, 0);
    lv_obj_set_style_border_width(row_sliders, 0, 0);

    lv_obj_set_flex_flow(row_sliders, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row_sliders,
                          LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    create_slider_block(row_sliders, "Volume", 0, 100,
                        var_slider1, slicer_dac_volume, &box_s1, &sl_s1, &lbl_s1);
    create_slider_block(row_sliders, "RTL gain", 0, 50,
                        var_slider2, slider_rtl_gain, &box_s2, &sl_s2, &lbl_s2);
    create_slider_block(row_sliders, "Brightness", 1, 200,
                        var_slider4, slider_brightness, &box_s4, &sl_s4, &lbl_s4);

    /* ---------- Botonera (3 filas: 6 + 6 + PALETTE) ----------
     * Height and row gap set explicitly so the third row fits:
     * 3 x 60 px buttons + 2 x 10 px gaps = 200 px, inside the 500 px panel
     * (16 px padding x 2 + 190 px sliders + column gap + this grid). */
    grid_btns = lv_obj_create(cont_menu);
    lv_obj_set_width(grid_btns, lv_pct(100));
    lv_obj_set_height(grid_btns, 210);
    lv_obj_set_style_pad_all(grid_btns, 0, 0);
    lv_obj_set_style_pad_row(grid_btns, 10, 0);
    lv_obj_remove_flag(grid_btns, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(grid_btns, 0, 0);

    /* Flex: filas con wrap */
    lv_obj_set_flex_flow(grid_btns, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(grid_btns,
                          LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    /* Espacio vertical entre filas */
    btn1_usb = create_button(grid_btns, "USB", btn1_cb);
    btn2_lsb = create_button(grid_btns, "LSB", btn2_cb);
    btn3_am = create_button(grid_btns, "AM", btn3_cb);
    btn4_sam = create_button(grid_btns, "SAM", btn4_cb);
    btn5_samu = create_button(grid_btns, "SAMU", btn5_cb);
    btn6_saml = create_button(grid_btns, "SAML", btn6_cb);

    btn7_nr = create_button(grid_btns, "NR", btn7_cb);
    {
        /* The menu grid is destroyed and rebuilt from scratch every time it's
         * closed/reopened (see btn12_cb: lv_obj_del_async(cont_menu) + all these
         * pointers set to NULL) - create_button() only ever takes a fixed literal
         * for its initial label, so this must be formatted from the LIVE value
         * every time, or the button keeps showing "SPT 0" on reopen regardless of
         * what spec_smooth_passes actually holds (the value itself is a plain
         * global and does persist correctly across that rebuild - only the label
         * was wrong). */
        char spt_label[8];
        snprintf(spt_label, sizeof(spt_label), "SPT %d", (int)spec_smooth_passes);
        btn8_filtro = create_button(grid_btns, spt_label, btn8_cb);
    }
    btn9_agc = create_button(grid_btns, "AGC", btn9_cb);
    btn10 = create_button(grid_btns, "NFM", btn10_cb);
    btn11 = create_button(grid_btns, "WFM", btn11_cb);
    btn12_close = create_button(grid_btns, LV_SYMBOL_CLOSE, btn12_cb);

    btn13_palette = create_button(grid_btns, "PALETTE", btn13_cb);
    lv_obj_set_width(btn13_palette, 180); /* room for "Temper Colors" */
    lv_obj_set_style_text_align(lv_obj_get_child(btn13_palette, 0), LV_TEXT_ALIGN_CENTER, 0);
    palette_btn_label();
    btn14_ais = create_button(grid_btns, "AIS", btn14_cb);
    lv_obj_set_style_text_align(lv_obj_get_child(btn14_ais, 0), LV_TEXT_ALIGN_CENTER, 0);
    ais_btn_label();
    ui_ais_set_state_callback(ais_btn_label);
}

/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */

static lv_obj_t *freq_popup = NULL;
static lv_obj_t *ta_freq = NULL;
static lv_obj_t *btnm_freq = NULL;

static uint32_t parse_freq(const char *s, double mul)
{
    char *end;
    double v = strtod(s, &end);
    if (end == s || v <= 0)
        return 0;
    return (uint32_t)(v * mul + 0.5);
}

static void freq_popup_close(void)
{
    if (freq_popup)
    {
        lv_obj_del(freq_popup);
        freq_popup = NULL;
        ta_freq = NULL;
        btnm_freq = NULL;
    }
}

static void freq_btnm_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED)
        return;

    const char *txt = lv_btnmatrix_get_btn_text(
        lv_event_get_target(e),
        lv_btnmatrix_get_selected_btn(lv_event_get_target(e)));

    if (!txt)
        return;

    if (strcmp(txt, LV_SYMBOL_LEFT) == 0)
    {
        lv_textarea_delete_char(ta_freq);
        return;
    }

    if (strcmp(txt, "kHz") == 0 || strcmp(txt, "MHz") == 0)
    {
        double mul = (txt[0] == 'k') ? 1e3 : 1e6;
        uint32_t hz = parse_freq(lv_textarea_get_text(ta_freq), mul);
        if (hz)
        {
            currentVFO.Frec = hz;
            refresca_VFO();
            rtl_source_set_freq(currentVFO.Frec - lo_offset_for_mode(demod_modo));
            freq_popup_close();

            inicia_timers();
        }
        return;
    }

    if (strcmp(txt, LV_SYMBOL_OK) == 0)
    {
        //uint32_t hz = parse_freq(lv_textarea_get_text(ta_freq), 1e6);
        //if (hz)
        //    currentVFO.Frec = hz;
        refresca_VFO();
        rtl_source_set_freq(currentVFO.Frec - lo_offset_for_mode(demod_modo));
        freq_popup_close();

        inicia_timers();

        return;
    }

    /* dígitos y punto */
    if ((txt[0] >= '0' && txt[0] <= '9' && txt[1] == '\0') || strcmp(txt, ".") == 0)
    {
        if (strcmp(txt, ".") == 0)
        {
            const char *s = lv_textarea_get_text(ta_freq);
            if (strchr(s, '.'))
                return;
            if (!s[0])
                lv_textarea_add_text(ta_freq, "0");
        }
        lv_textarea_add_text(ta_freq, txt);
    }
}

void freq_label_event_cb(lv_event_t *e)
{
    if (freq_popup)
        return;

    if (timer_pantalla)
        lv_timer_pause(timer_pantalla);
    if (timer_smeter)
        lv_timer_pause(timer_smeter);

    freq_popup = lv_obj_create(lv_scr_act());
    lv_obj_set_size(freq_popup, 520, 300);
    lv_obj_center(freq_popup);
    lv_obj_set_style_pad_all(freq_popup, 12, 0);

    lv_obj_set_style_bg_color(freq_popup, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(freq_popup, LV_OPA_COVER, 0);

    ta_freq = lv_textarea_create(freq_popup);
    lv_obj_set_width(ta_freq, lv_pct(100));
    lv_textarea_set_one_line(ta_freq, true);

    lv_textarea_set_text(ta_freq, "");

    static const char *map[] = {
        "7", "8", "9", LV_SYMBOL_LEFT, "\n",
        "4", "5", "6", "kHz", "\n",
        "1", "2", "3", "MHz", "\n",
        "0", ".", LV_SYMBOL_OK, ""};

    btnm_freq = lv_btnmatrix_create(freq_popup);
    lv_btnmatrix_set_map(btnm_freq, map);
    lv_obj_set_size(btnm_freq, lv_pct(100), 220);
    lv_obj_align(btnm_freq, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_add_event_cb(btnm_freq, freq_btnm_event_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);
}

/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------- */