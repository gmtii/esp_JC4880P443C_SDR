#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"

    void spectrum(void);
    void init_ui(void);
    void nuevo(void);
    void dibuja_botones(void);
    lv_obj_t *smeter_create(lv_obj_t *parent, int size);
    lv_indev_t *lvgl_encoder_init(void);
    void tarea_encoder(void *arg);
    void waterfall_update(void);
    void dibuja_pasabanda(void);
    void inicia_smeter_ui(void);
    void smeter_set_dbm(float dbm);
    void crear_label_con_estilo(lv_obj_t *parent, char *texto, int x, int y, int ancho, bool color_verde);
    void refresca_VFO(void);
    void timer_dibuja_pantalla(lv_timer_t *timer);
    void timer_smeter_update(lv_timer_t *timer);
    void timer_uso_cpu(lv_timer_t *timer);
    void refresca_indicadores(void);
    void indicadores_create(lv_obj_t *parent);
    void indicador_update(int idx, const char *texto, bool verde);

    /* Waterfall / FT8-cascade color palette (palette_id_t in palettes.h).
     * ui_set_palette() applies it immediately and saves it in NVS. Call from
     * the LVGL context. */
    /* AIS mode (menu button): enter/leave, state, and a callback the menu uses
     * to refresh its button when the state changes. LVGL context only. */
    void ui_ais_toggle(void);
    bool ui_ais_is_active(void);
    void ui_ais_set_state_callback(void (*cb)(void));

    int ui_get_palette(void);
    void ui_set_palette(int id);
    static void init_styles_once(void);

    extern lv_timer_t *timer_pantalla;
    extern lv_timer_t *timer_cpu;
    extern lv_timer_t *timer_smeter;

#ifdef __cplusplus
}
#endif