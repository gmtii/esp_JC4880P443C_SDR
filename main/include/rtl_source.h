/*
 * rtl_source.h - RTL-SDR (esp_rtl_sdr USB Host driver) as the I/Q source of the
 * receiver. Replaces the I2S RX path + MSI001 tuner: it delivers 48 kSps I/Q
 * frames, the same domain the DSP chain in sdr.c already works in.
 *
 * Threading:
 *   - rtl_source_init()        once, from app_main (does not block on USB)
 *   - rtl_source_read_float()  from the SDR task only (single consumer)
 *   - rtl_source_set_*()       from any task, including LVGL callbacks: they only
 *                              post a request; a control task talks to the dongle
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Spectrum orientation. sdr.c shifts the spectrum by +fs/4 (multiplying by j^n)
 * and expects the wanted signal at -12 kHz when the LO is at f_vfo - 12 kHz,
 * i.e. the orientation the codec path delivered. A standard RTL-SDR puts a
 * signal above the LO at +12 kHz, so Q is negated by default. If USB and LSB
 * come out swapped, or the carrier of a known station is not at the centre
 * marker of the spectrum, set this to 0.
 */
#ifndef RTL_SOURCE_CONJUGATE_IQ
#define RTL_SOURCE_CONJUGATE_IQ 1
#endif

typedef struct {
    bool     streaming;        /* dongle attached and bulk stream running            */
    bool     wide;             /* true: fifo_frames and step_ppm are in wide (192 kSps) units */
    uint32_t fifo_frames;      /* frames waiting for the SDR task (rate per `wide`)  */
    float    step_ppm;         /* clock-drift correction currently applied           */
    uint32_t fifo_overruns;    /* frames dropped because the FIFO was full           */
    uint32_t underruns;        /* SDR task waited for data and timed out             */
    uint32_t skipped_frames;   /* frames discarded by the consumer above high water  */
    uint32_t usb_overruns;     /* driver: USB side could not keep up                 */
    uint32_t usb_drops;        /* driver: consumer too slow inside the driver        */
    uint32_t effective_sps;    /* driver: measured dongle rate                       */
} rtl_source_stats_t;

/* initial_lo_hz: LO frequency (VFO - 12 kHz). initial_gain_db: manual tuner gain. */
esp_err_t rtl_source_init(uint32_t initial_lo_hz, int initial_gain_db);

/*
 * Request a new LO frequency; the latest request wins. Cheap and non-blocking.
 * Within RTL_SOURCE_DIGITAL_WINDOW_HZ (24 kHz) of the dongle's physical LO the
 * change is made by an NCO in rtl_dsp.c: instant, the USB I/Q stream never
 * stops (dragging the spectrum or spinning the encoder no longer freezes the
 * display). Beyond it, or in WFM mode, the dongle is retuned physically, which
 * briefly pauses the stream. After tuning has been idle for
 * RTL_SOURCE_RECENTRE_MS the dongle is re-centred once and the offset dropped.
 */
void rtl_source_set_freq(uint32_t lo_hz);

/* Manual tuner gain (0..49 dB, nearest step) or the tuner's own AGC. */
void rtl_source_set_gain_db(int gain_db);
void rtl_source_set_gain_auto(bool enable);

/*
 * Block until `frames` 48 kSps I/Q frames are available (or timeout) and return
 * them as floats in [-1, 1), same scaling the I2S path used (int16 / 32767).
 * ESP_ERR_TIMEOUT: no dongle, or it stalled. The caller should output silence.
 */
esp_err_t rtl_source_read_float(float *i, float *q, size_t frames, uint32_t timeout_ms);

/*
 * Switch between the narrow (48 kSps, default) and wide (192 kSps, for WFM) I/Q
 * rate. A no-op if the stream is already in the requested mode. Otherwise this
 * restarts the USB stream (a fresh ring, re-primed at the new rate) rather than
 * changing the rate under a FIFO that may still hold frames at the old rate's
 * duration - mode changes are a deliberate, infrequent user action (switching
 * demod mode), so the ~1 s restart this costs is a fair trade for never mixing
 * two frame durations in the same drift-control calculation. Safe to call every
 * loop iteration: only an actual change triggers a restart.
 */
void rtl_source_set_wide(bool wide);

bool rtl_source_is_streaming(void);
void rtl_source_get_stats(rtl_source_stats_t *out);

#ifdef __cplusplus
}
#endif
