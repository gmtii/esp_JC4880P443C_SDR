#ifndef FT8_APP_H
#define FT8_APP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * FT8 receive mode for the ESP32-P4 SDR - glue between sdr.c's audio chain,
 * the ported DeepSDR 101 FT8 pipeline (ft8_decimator -> ft8_waterfall_adapter
 * -> ft8_decoder, all in main/ft8/) and the UI (ft8_ui.c).
 *
 * Threading:
 *  - sdrTask (core 1, prio 20) calls ft8_app_feed_audio() once per loop with
 *    the 12 kHz USB-demodulated block. That call only copies the samples into
 *    a PSRAM stream buffer (non-blocking) - no DSP happens in sdrTask.
 *  - The FT8 task (core 0, low priority) drains that buffer, runs AGC,
 *    resampler, FFT and waterfall encoding, schedules 15 s slots from the
 *    system clock (ft8_time.h), decodes each completed slot, and feeds the
 *    per-decode timing to the band-sync loop (ft8_band_sync.h), which steers
 *    that clock onto the transmissions on the air - so no accurate external
 *    time source is required.
 *  - The LVGL task reads status/messages (ft8_ui.c).
 *
 * This replaces the GD32's split between the audio DMA ISR (resampler) and
 * the main loop (FFT, scheduling, decode), including its critical sections.
 */

typedef struct
{
    bool active;              /* FT8 mode selected */
    bool capturing;           /* a slot capture is armed (false until the first boundary after entry/resync) */
    int blocks;               /* symbol blocks collected in the current slot (0..93) */
    uint32_t total_decodes;   /* since boot */
    uint32_t slots_processed; /* since boot */
    int last_decoded;         /* last slot: messages decoded */
    int last_candidates;      /* last slot: sync candidates tried */
    uint32_t last_proc_ms;    /* last slot: decode wall-clock time */
    float last_db_min;        /* last slot: raw dB range, see FT8_ADAPTER_DB_OFFSET */
    float last_db_max;
    uint32_t overruns;        /* audio blocks dropped because the stream buffer was full */
    int sync_state;           /* ft8_bsync_state_t - band-sync loop state */
    bool sync_have_err;       /* sync_err_s is valid */
    float sync_err_s;         /* median DT of the last slot with decodes */
} ft8_status_t;

/* Allocates the FT8 buffers (PSRAM), loads the own grid from NVS, hooks the
 * serial time-sync grid callback and starts the FT8 task. Call once from
 * app_main() before sdrTask starts feeding. Returns false if FT8 could not be
 * set up (it then stays disabled; the rest of the radio is unaffected). */
bool ft8_app_init(void);

/* True once ft8_app_init() succeeded. */
bool ft8_app_available(void);

/* Enter/leave FT8 mode (DSP side only - the UI side is ft8_ui.c). On entry
 * the task waits for the next 15 s boundary before arming the first capture. */
void ft8_app_set_active(bool on);
bool ft8_app_is_active(void);

/* Called by sdrTask with each block of 12 kHz USB-demodulated audio, in the
 * receiver's native float scale (+-1.0 full scale). Cheap no-op when FT8 is
 * not active. */
void ft8_app_feed_audio(const float *samples, size_t n);

/* Snapshot of the current state for the UI. */
void ft8_app_get_status(ft8_status_t *out);

#endif /* FT8_APP_H */
