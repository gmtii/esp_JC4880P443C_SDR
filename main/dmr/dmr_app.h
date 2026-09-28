#ifndef DMR_APP_H
#define DMR_APP_H

#include <stdbool.h>
#include <stddef.h>

/*
 * DMR receive mode (phase 1: metadata) for the ESP32-P4 SDR.
 *
 * Threading, same pattern as ft8/ft8_app.h:
 *  - sdrTask (core 1) calls dmr_app_feed_iq() with the raw 48 kS/s IQ of the
 *    NFM branch (before its discriminator). That only copies into a PSRAM
 *    stream buffer; no DSP runs in sdrTask.
 *  - The DMR task (core 0, low priority) runs the channel filter,
 *    discriminator, 4FSK demodulator and burst decoder (dmr_demod.c,
 *    dmr_proto.c).
 *  - The LVGL task reads snapshots (dmr_ui.c).
 */

bool dmr_app_init(void);
bool dmr_app_available(void);

void dmr_app_set_active(bool on);
bool dmr_app_is_active(void);

/* From sdrTask: n complex samples at 48 kS/s, channel 12 kHz off centre. */
void dmr_app_feed_iq(const float *i, const float *q, size_t n);

/* From sdrTask, NFM branch: fills n samples at 48 kHz with decoded DMR voice
 * (silence when there is none, or when voice is not compiled in). */
void dmr_app_read_audio(float *out, size_t n);

#include <stdint.h>
/* Diagnostics: IQ blocks dropped because the DMR task fell behind, and audio
 * underruns (gaps heard as choppy voice). */
void dmr_app_get_stats(uint32_t *iq_drops, uint32_t *underruns);

#endif /* DMR_APP_H */
