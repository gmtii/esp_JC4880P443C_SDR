#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#include "sdr.h"

#include "lvgl.h"

    static lv_obj_t *waveform_canvas;
    static lv_obj_t *waterfall_canvas;

    lv_obj_t *screen;

    lv_obj_t *cont;
    lv_obj_t *cont_ind;
    lv_obj_t *cont_principal_botones;
    static lv_obj_t *meter_cont;

    lv_obj_t *btn_menu;
    lv_obj_t *btn_step;
    lv_obj_t *btn_filtros;
    lv_obj_t *btn_modos;
    lv_obj_t *btn2;
    lv_obj_t *btn3;
    lv_obj_t *btn4;
    lv_obj_t *btn5;
    lv_obj_t *btn6;
    lv_obj_t *btn_vol;
    lv_obj_t *slider_vol;

    lv_obj_t *box_pasabanda;
    lv_obj_t *needle;

    lv_obj_t *smeter_obj;
    lv_obj_t *freq_label;
    lv_obj_t *label_modo_info; /* top-left info block: current demod mode, plain text (not a button) */
    lv_obj_t *label_clock;     /* top-right corner: placeholder, uptime since boot until real RTC/NTP lands */

    lv_obj_t *label_modos;
    lv_obj_t *label_step;
    lv_obj_t *label_menu;
    lv_obj_t *label_filtros;
    lv_obj_t *label2;
    lv_obj_t *label3;
    lv_obj_t *label4;
    lv_obj_t *label5;
    lv_obj_t *label6;
    lv_obj_t *label_vol;
    lv_obj_t *label_msi_gain;

    lv_obj_t *value_label;


    static lv_obj_t *indicador;

    lv_timer_t *timer_pantalla;
    lv_timer_t *timer_smeter;
    lv_timer_t *timer_clock;
    lv_timer_t *timer_cpu;
    lv_timer_t *timer_debounce;

    typedef enum
    {
        BTN_AM, // boton 0
        BTN_FM,
        BTN_USB,
        BTN_SSB,
        BTN_SAM,
        BTN_SAMU,
        BTN_SAML,
        BTN_PASO,
        BTN_FILTRO,
        BTN_USO_CPU,
        BTN_USO_MEM,
        BTN_MENU
    } button_id_t;

    typedef enum
    {
        IND_AM, // Indicador 0
        IND_FM,
        IND_USB,
        IND_SSB,
        IND_SAM,
        IND_SAMU,
        IND_SAML,
        IND_PASO,
        IND_F_CW,
        IND_F_1K8,
        IND_F_2K3,
        IND_F_3K6,
        IND_F_4K0,
        IND_13,
        IND_14,
        IND_15, // indicador 15

    } indicadores_id_t;

    extern int16_t pixelnew[SAMPLE_BUFFER_SIZE];
    extern int16_t pixelold[SAMPLE_BUFFER_SIZE];

    // Offset y máxima altura. Para controlar el espectro.
    int spectrum_y = 0; // upper edge
    int spectrum_x = 0;
    int spectrum_height = WAVEFORM_HEIGHT;

#define W WAVEFORM_WIDTH
#define H WAVEFORM_HEIGHT

#define CANVAS_W WAVEFORM_WIDTH
#define CANVAS_H WAVEFORM_HEIGHT

#define ROW_BYTES (CANVAS_W / 2)
#define BUF_SIZE (ROW_BYTES * CANVAS_H)

    // static uint8_t waveformbuffer[BUF_SIZE] __attribute__((aligned(32)));

    uint16_t *waveformbuffer;
    /*
     * Waterfall ring buffer: 2*WATERFALL_HEIGHT rows, each row duplicated WATERFALL_HEIGHT
     * rows apart (buf[r] == buf[r + WATERFALL_HEIGHT] at all times). Any WATERFALL_HEIGHT-row
     * window starting at a row in [0, WATERFALL_HEIGHT) is then contiguous memory, so scrolling
     * needs no memmove: waterfall_head just steps back by one slot per frame (see waterfall_update()).
     */
    uint16_t *waterfallbuffer;
    int waterfall_head;

#define SMETER_PIVOT_X 168
#define SMETER_PIVOT_Y 200
#define SMETER_NEEDLE_LEN 200

#define SMETER_DBM_MIN -121
#define SMETER_DBM_MAX -60

#define SMETER_ANGLE_MIN 40  // grados
#define SMETER_ANGLE_MAX 140 // grados

/*
 * S-meter panel (top-left). The bar itself (scale ticks, 54 LED-style segments,
 * peak marker) is rendered by smeter_draw.c into a small RGB565 canvas; the
 * scale numbers and readouts are LVGL labels. Geometry below matches the host
 * preview rendered from the same smeter_draw.c code.
 */
#define SMETER_PANEL_X 5
#define SMETER_PANEL_Y 4
#define SMETER_PANEL_W 345
#define SMETER_PANEL_H 128
#define SMETER_CANVAS_X 10 /* inside the panel */
#define SMETER_CANVAS_Y 30
#define SMETER_LABEL_Y 8

    lv_obj_t *smeter_canvas;
    uint16_t *smeter_canvas_buf;
    lv_obj_t *smeter_lbl_s;
    lv_obj_t *smeter_lbl_dbm;
    lv_obj_t *smeter_lbl_peak;

#ifdef __cplusplus
}
#endif