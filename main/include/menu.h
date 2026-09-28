#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    extern lv_obj_t *freq_label;

    void ui_create_control_panel(void);
    void ui_create_menu_lvgl92(void);
    void freq_label_event_cb(lv_event_t *e);

    /* The RTL gain slider's current value (dB), so leaving WFM can restore the
     * manual gain it had before WFM switched the tuner to AGC. */
    int menu_get_rtl_gain_db(void);

#ifdef __cplusplus
}
#endif