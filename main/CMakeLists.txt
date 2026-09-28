set(INCLUDE_DIR ./include)

idf_component_register(
    SRCS 
    main.c
    audio_out.c
    sdr.c
    agc.c
    nr.c
    sam.c
    ui.c
    sdr_math.c
    dsp_example.c
    nr_ss.c
    smeter.c
    smeter_draw.c
    rtl_dsp.c
    rtl_source.c
    uart_commands.c
    menu.c
    palettes.c
    ft8/ft8_app.c
    ft8/ft8_ui.c
    ft8/ft8_decimator.c
    ft8/ft8_waterfall_adapter.c
    ft8/ft8_fft1024.c
    ft8/ft8_ram.c
    ft8/ft8_decoder.c
    ft8/ft8_time.c
    ft8/time_sync.c
    ft8/ft8_band_sync.c
    dmr/dmr_app.c
    dmr/dmr_ui.c
    dmr/dmr_demod.c
    dmr/dmr_proto.c
    dmr/dmr_fec.c
    dmr/dmr_voice.c
    ais/ais_app.c
    ais/ais_ui.c
    ais/ais_demod.c
    ais/ais_msg.c
    INCLUDE_DIRS .  ${INCLUDE_DIR} ft8 dmr ais
    ) 

idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
target_compile_options(
    ${LVGL_LIB} 
    PRIVATE
        -DLV_LVGL_H_INCLUDE_SIMPLE
)

