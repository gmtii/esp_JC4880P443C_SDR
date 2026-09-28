/*
 * rtl_source.c - glue between esp_rtl_sdr (USB Host) and the receiver DSP.
 *
 *   USB block (CU8 @ 960 kSps, delivery task, callback mode)
 *        |  rtl_dsp_process(): CIC -> drift resampler -> FIR    (rtl_dsp.c)
 *        v
 *   SPSC ring of int16 I/Q frames @ 48 kSps
 *        |  rtl_source_read_float(): SDR task, paced by the codec's I2S TX
 *        v
 *   sdr.c
 *
 * The dongle and the codec run from different crystals. The consumer measures
 * the ring level once per block and steers the resampler step so the level
 * stays constant (see rtl_dsp.h). No samples are ever dropped or repeated in
 * normal operation.
 */
#include "rtl_source.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_rtl_sdr.h"
#include "rtl_dsp.h"

static const char *TAG = "rtl_source";

/* Uncomment to log how long each retune actually blocks (pause bulk + drain + EP0 +
 * resubmit) - see ui.c's SPECTRUM_DRAG_RETUNE_MIN_US comment for why. */
// #define RTL_SOURCE_RETUNE_LOG

/*
 * Sized for the worst case (wide/WFM mode, 192 kSps): RING_FRAMES gives the same
 * ~170 ms of headroom wide mode had at 48 kSps (8192 frames there), scaled by the
 * RTL_DSP_WIDE_RATE/RTL_DSP_OUT_RATE ratio (4x). Narrow mode gets proportionally
 * more headroom in wide mode's frame units, which is harmless. OUT_CHUNK_FRAMES
 * covers RTL_DSP_WIDE_MAX_FRAMES(CHUNK_BYTES) (~421 with today's sizes), well
 * above narrow mode's ~108.
 */
#define RING_FRAMES       32768u                    /* power of two, ~170 ms at wide rate */
#define RING_MASK         (RING_FRAMES - 1u)
#define CHUNK_BYTES       4096u                     /* DSP works in chunks of this size */
#define OUT_CHUNK_FRAMES  512u                      /* >= RTL_DSP_WIDE_MAX_FRAMES(CHUNK_BYTES) */

/* Retry delays after a failed start. Running out of memory cannot fix itself, and every
 * retry makes the driver replay its whole demodulator init table on an already
 * configured dongle (EP0 STALLs), so wait much longer than for a missing device. */
#ifndef RTL_SOURCE_RETRY_MS
/*
 * Digital tuning window: frequency changes within +-this of the hardware LO are
 * done by the NCO in rtl_dsp (instant, the USB stream never stops). Only a
 * change beyond it - or any change in WFM, which uses the full 192 kSps band -
 * retunes the dongle physically, which pauses, drains and resubmits the bulk
 * pipeline (that pause is what froze the spectrum and waterfall while dragging).
 * +-24 kHz keeps the CIC droop under ~2 dB at the channel edge.
 */
#define RTL_SOURCE_DIGITAL_WINDOW_HZ 24000
/*
 * Once tuning has been idle this long with a digital offset in place, retune
 * the dongle to the wanted frequency once and drop the offset: that restores
 * the full CIC alias rejection (about -67 dB instead of ~-40 dB for signals
 * ~140 kHz away at the window edge). One short stream pause, only after you
 * stop tuning. 0 disables it.
 */
#define RTL_SOURCE_RECENTRE_MS 1500u

#define RTL_SOURCE_RETRY_MS       500u
#endif
#ifndef RTL_SOURCE_NOMEM_RETRY_MS
#define RTL_SOURCE_NOMEM_RETRY_MS 10000u
#endif

typedef struct { int16_t i, q; } iq16_t;

static struct {
    esp_rtl_sdr_handle_t sdr;
    rtl_dsp_t dsp;                                  /* owned by the delivery task     */
    rtl_rate_ctl_t rc;                              /* owned by the SDR task          */
    volatile float step_req;                        /* SDR task -> delivery task      */

    iq16_t *ring;
    uint32_t head;                                  /* written by delivery task       */
    uint32_t tail;                                  /* written by SDR task            */
    /*
     * Arrival time of the block that made up the current head, truncated to 32 bits
     * (microseconds since boot). A plain 64-bit int64_t is not atomic on this 32-bit
     * target, and an earlier version paired it with `head` via a seqlock; that let the
     * reader (sdrTask, prio 20) livelock the writer (the driver's delivery task, prio
     * 18 - both pinned to core 1) by spinning on the lock without ever yielding, which
     * starved IDLE1 and tripped the task watchdog. A 32-bit timestamp needs no lock:
     * plain store here, ordered by the release-store of `head` right after it; plain
     * load on the read side, ordered by the acquire-load of `head` right before it.
     * The two can very rarely be one block apart (a torn "snapshot"), which the rate
     * controller's smoothing absorbs without effect; that trade-off is deliberate.
     */
    uint32_t arr_t_us32;
    volatile bool primed;

    /*
     * Narrow (48 kSps, default) vs wide (192 kSps, WFM) tap. `wide`/`rate_mult`/
     * `target_raw`/`high_water_raw` are owned by the delivery task (process_block()
     * applies a pending switch there - see rtl_source_set_wide()); `want_wide` and
     * `wide_pending` are the reader's request, mirroring how `step_req` and the
     * lo/gain `*_pending` flags already cross the same two tasks.
     */
    bool wide;
    uint32_t rate_mult;                             /* RTL_DSP_WIDE_RATE/RTL_DSP_OUT_RATE when wide, else 1 */
    uint32_t target_raw;                            /* RTL_RATE_TARGET_FRAMES, in the active rate's raw frames */
    uint32_t high_water_raw;                        /* likewise, the skip-ahead threshold */
    volatile bool want_wide;
    volatile bool wide_pending;

    volatile TaskHandle_t reader;
    volatile TaskHandle_t ctl;

    volatile bool streaming;
    volatile bool fault;


    volatile uint32_t want_lo_hz;
    volatile bool     lo_pending;
    /* Digital fine tuning (see rtl_source_set_freq()): hw_lo_hz is where the
     * dongle is physically tuned; dig_offset_hz = want - hw is applied by the
     * NCO in rtl_dsp from the delivery callback, without touching the stream. */
    volatile uint32_t hw_lo_hz;
    volatile bool     hw_valid;
    volatile int32_t  dig_offset_hz;
    volatile int64_t  last_tune_us;                 /* time of the last rtl_source_set_freq() */
    volatile int      want_gain_db;
    volatile bool     want_gain_auto;
    volatile bool     gain_pending;

    uint32_t fifo_overruns, underruns, skipped;
    uint32_t start_backoff_ms;                      /* ctl task only: wait before the next start attempt */
} S;

/* ------------------------------------------------------------------------- */
/* Delivery-task side                                                        */
/* ------------------------------------------------------------------------- */

static void process_block(const uint8_t *data, size_t bytes)
{
    int16_t out[2 * OUT_CHUNK_FRAMES];

    /* digital fine tuning: cheap, phase-continuous, no-op when unchanged */
    rtl_dsp_set_offset(&S.dsp, __atomic_load_n(&S.dig_offset_hz, __ATOMIC_ACQUIRE));

    if (__atomic_exchange_n(&S.wide_pending, false, __ATOMIC_ACQUIRE)) {
        const bool wide = S.want_wide;
        rtl_dsp_set_wide(&S.dsp, wide);
        S.wide = wide;
        S.rate_mult = wide ? (RTL_DSP_WIDE_RATE / RTL_DSP_OUT_RATE) : 1u;
        S.target_raw = RTL_RATE_TARGET_FRAMES * S.rate_mult;
        S.high_water_raw = (RTL_RATE_TARGET_FRAMES * 12u / 5u) * S.rate_mult;
        /* Flush: frames already queued are at the OLD rate's duration and would corrupt
         * the drift-level math if left mixed with new-rate frames. Safe to do here even
         * though `tail` belongs to the reader: this only ever moves `head` forward to
         * meet it (never past it), so the reader never sees head-tail go negative. */
        const uint32_t tail_now = __atomic_load_n(&S.tail, __ATOMIC_ACQUIRE);
        __atomic_store_n(&S.head, tail_now, __ATOMIC_RELEASE);
        S.primed = false;
    }

    rtl_dsp_set_step(&S.dsp, S.step_req);

    uint32_t head = S.head;
    const uint32_t tail = __atomic_load_n(&S.tail, __ATOMIC_ACQUIRE);

    for (size_t off = 0; off < bytes; off += CHUNK_BYTES) {
        size_t n = bytes - off;
        if (n > CHUNK_BYTES) n = CHUNK_BYTES;
        const size_t nf = rtl_dsp_process(&S.dsp, data + off, n, out, OUT_CHUNK_FRAMES);
        const uint32_t space = RING_FRAMES - (head - tail);
        if (nf > space) {
            S.fifo_overruns += (uint32_t)nf;
            continue;
        }
        for (size_t k = 0; k < nf; k++) {
            S.ring[(head + k) & RING_MASK] = (iq16_t){ out[2 * k], out[2 * k + 1] };
        }
        head += (uint32_t)nf;
    }

    /* Publish the arrival time before head: the release-store below makes both visible
     * together to any reader that does an acquire-load of head (see the field comment). */
    S.arr_t_us32 = (uint32_t)esp_timer_get_time();
    __atomic_store_n(&S.head, head, __ATOMIC_RELEASE);

    TaskHandle_t r = S.reader;
    if (r != NULL) {
        xTaskNotifyGive(r);
    }
}

static void on_event(esp_rtl_sdr_event_t ev, const void *payload, void *ctx)
{
    (void)ctx;
    switch (ev) {
    case ESP_RTL_SDR_EVT_IQ_BLOCK: {
        const esp_rtl_sdr_iq_block_t *b = (const esp_rtl_sdr_iq_block_t *)payload;
        process_block(b->data, b->bytes);
        break;
    }
    case ESP_RTL_SDR_EVT_DISCONNECTED:
    case ESP_RTL_SDR_EVT_ERROR:
        S.fault = true;
        if (S.ctl != NULL) {
            xTaskNotifyGive(S.ctl);
        }
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Control task: install / start / retune / gain / recovery                  */
/* ------------------------------------------------------------------------- */

static void apply_gain(void)
{
    esp_err_t err;
    if (S.want_gain_auto) {
        err = esp_rtl_sdr_set_tuner_gain_mode(S.sdr, ESP_RTL_SDR_GAIN_MODE_AUTO);
    } else {
        err = esp_rtl_sdr_set_tuner_gain(S.sdr, S.want_gain_db * 10);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "gain request failed: %s", esp_rtl_sdr_err_to_name(err));
    }
}

static bool start_stream(void)
{
    size_t count = 0;
    S.start_backoff_ms = RTL_SOURCE_RETRY_MS;
    (void)esp_rtl_sdr_refresh_device_list(S.sdr);
    (void)esp_rtl_sdr_get_device_count(S.sdr, &count);
    if (count == 0) {
        return false;
    }

    rtl_dsp_init(&S.dsp, RTL_SOURCE_CONJUGATE_IQ != 0);
    rtl_dsp_set_wide(&S.dsp, S.want_wide);
    /* (re)starting tunes the dongle to the wanted frequency: no digital offset */
    S.hw_lo_hz = S.want_lo_hz;
    __atomic_store_n(&S.dig_offset_hz, 0, __ATOMIC_RELEASE);
    S.hw_valid = true;
    S.wide = S.want_wide;
    S.rate_mult = S.wide ? (RTL_DSP_WIDE_RATE / RTL_DSP_OUT_RATE) : 1u;
    S.target_raw = RTL_RATE_TARGET_FRAMES * S.rate_mult;
    S.high_water_raw = (RTL_RATE_TARGET_FRAMES * 12u / 5u) * S.rate_mult;
    __atomic_store_n(&S.wide_pending, false, __ATOMIC_RELAXED);
    S.step_req = 1.0f;
    S.primed = false;

    /* Clear the flags first, then read the values: a request that lands in between is not lost */
    (void)__atomic_exchange_n(&S.lo_pending, false, __ATOMIC_ACQUIRE);
    (void)__atomic_exchange_n(&S.gain_pending, false, __ATOMIC_ACQUIRE);

    esp_rtl_sdr_stream_config_t st;
    esp_rtl_sdr_stream_config_default(&st);
    st.preset = ESP_RTL_SDR_PRESET_CUSTOM_HZ;
    st.frequency_hz = S.want_lo_hz;
    st.sample_rate_sps = RTL_DSP_IN_RATE;

    /* start() needs ~96 KiB of DMA-capable internal RAM for the USB buffers (6 x 16 KiB,
     * one contiguous block each), 6 x 16 KiB for its I/Q slots (PSRAM first) and a 6 KiB
     * task stack. If it fails with ESP_ERR_NO_MEM, this line shows what was available. */
    ESP_LOGI(TAG, "heap before start: internal free %u, largest internal DMA block %u, PSRAM free %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    const esp_err_t err = esp_rtl_sdr_start(S.sdr, &st);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "start failed: %s", esp_rtl_sdr_err_to_name(err));
        if (err == ESP_ERR_NO_MEM) {
            ESP_LOGE(TAG, "out of memory starting the stream: free internal RAM (see the heap line above); "
                          "next attempt in %u ms", (unsigned)RTL_SOURCE_NOMEM_RETRY_MS);
            S.start_backoff_ms = RTL_SOURCE_NOMEM_RETRY_MS;
        }
        return false;
    }
    S.fault = false;
    apply_gain();
    S.streaming = true;
    ESP_LOGI(TAG, "streaming, LO %u Hz, %u kSps, caps 0x%08x",
             (unsigned)st.frequency_hz, (unsigned)(RTL_DSP_IN_RATE / 1000),
             (unsigned)esp_rtl_sdr_get_device_capabilities(S.sdr));
    return true;
}

static void recover(void)
{
    S.streaming = false;
    S.primed = false;
    ESP_LOGW(TAG, "stream lost, restarting");
    for (int attempt = 0; attempt < 5; attempt++) {
        if (esp_rtl_sdr_stop(S.sdr, 1000) == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (esp_rtl_sdr_get_state(S.sdr) == ESP_RTL_SDR_STATE_FAULT) {
        (void)esp_rtl_sdr_reset(S.sdr);
    }
    S.fault = false;
}

static void ctl_task(void *arg)
{
    (void)arg;

    esp_rtl_sdr_config_t cfg;
    esp_rtl_sdr_config_default(&cfg);
    cfg.event_cb = on_event;
    cfg.delivery_mode = ESP_RTL_SDR_DELIVERY_CALLBACK;   /* no pull ring, we keep our own */

    while (esp_rtl_sdr_install(&cfg, &S.sdr) != ESP_OK) {
        ESP_LOGE(TAG, "esp_rtl_sdr_install failed, retrying");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    TickType_t next_start = xTaskGetTickCount();

    for (;;) {
        if (!S.streaming) {
            /* Frequency/gain requests also wake this task; they must not cut the retry delay short */
            const TickType_t now = xTaskGetTickCount();
            if ((int32_t)(next_start - now) > 0) {
                ulTaskNotifyTake(pdTRUE, next_start - now);
                continue;
            }
            if (!start_stream()) {
                next_start = xTaskGetTickCount() + pdMS_TO_TICKS(S.start_backoff_ms);
            }
            continue;
        }

        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));

#if RTL_SOURCE_RECENTRE_MS > 0
        /* Idle with a digital offset in place: one physical retune to recentre. */
        if (__atomic_load_n(&S.dig_offset_hz, __ATOMIC_ACQUIRE) != 0 &&
            (esp_timer_get_time() - S.last_tune_us) > (int64_t)RTL_SOURCE_RECENTRE_MS * 1000) {
            __atomic_store_n(&S.lo_pending, true, __ATOMIC_RELEASE);
        }
#endif

        if (S.fault || esp_rtl_sdr_get_state(S.sdr) != ESP_RTL_SDR_STATE_STREAMING) {
            recover();
            continue;
        }
        if (__atomic_exchange_n(&S.lo_pending, false, __ATOMIC_ACQUIRE)) {
            const uint32_t lo = S.want_lo_hz;
#ifdef RTL_SOURCE_RETUNE_LOG
            /* How long does a real retune (pause bulk + drain + EP0 + resubmit, per
             * esp_rtl_sdr's own source) actually take on this hardware? Needed to pick
             * ui.c's SPECTRUM_DRAG_RETUNE_MIN_US on measurement instead of a guess -
             * see that constant's comment. */
            const int64_t t0 = esp_timer_get_time();
#endif
            const esp_err_t err = esp_rtl_sdr_retune_hz(S.sdr, lo);
#ifdef RTL_SOURCE_RETUNE_LOG
            ESP_LOGI(TAG, "retune to %u Hz took %lld us", (unsigned)lo, (long long)(esp_timer_get_time() - t0));
#endif
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "retune %u Hz failed: %s", (unsigned)lo, esp_rtl_sdr_err_to_name(err));
            } else {
                /* The dongle is now at lo. Whatever was requested meanwhile stays
                 * digital if it is within the window of the new LO. */
                S.hw_lo_hz = lo;
                const int32_t rest = (int32_t)(S.want_lo_hz - lo);
                if (rest == 0) {
                    /* on target: nothing left to do (this is ALWAYS the case in WFM
                     * after its own retune - the first version of this code sent
                     * rest == 0 in wide mode to the "retune again" branch below and
                     * looped forever, freezing the spectrum on entering WFM) */
                    __atomic_store_n(&S.dig_offset_hz, 0, __ATOMIC_RELEASE);
                } else if (!S.want_wide && rest >= -RTL_SOURCE_DIGITAL_WINDOW_HZ &&
                           rest <= RTL_SOURCE_DIGITAL_WINDOW_HZ) {
                    /* moved a little further meanwhile: finish digitally */
                    __atomic_store_n(&S.dig_offset_hz, rest, __ATOMIC_RELEASE);
                } else {
                    /* moved beyond the window (or WFM moved on): one more real retune */
                    __atomic_store_n(&S.dig_offset_hz, 0, __ATOMIC_RELEASE);
                    __atomic_store_n(&S.lo_pending, true, __ATOMIC_RELEASE);
                    xTaskNotifyGive(S.ctl);
                }
            }
        }
        if (__atomic_exchange_n(&S.gain_pending, false, __ATOMIC_ACQUIRE)) {
            apply_gain();
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

esp_err_t rtl_source_init(uint32_t initial_lo_hz, int initial_gain_db)
{
    if (S.ring != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /* The FIFO is touched once per USB block and once per audio block: PSRAM is fast enough and
     * keeps 32 KiB of scarce internal RAM free for the driver's DMA buffers. */
    S.ring = (iq16_t *)heap_caps_malloc(RING_FRAMES * sizeof(iq16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (S.ring == NULL) {
        S.ring = (iq16_t *)heap_caps_malloc(RING_FRAMES * sizeof(iq16_t), MALLOC_CAP_8BIT);
    }
    if (S.ring == NULL) {
        return ESP_ERR_NO_MEM;
    }
    S.want_lo_hz = initial_lo_hz;
    S.want_gain_db = initial_gain_db;
    S.want_gain_auto = false;
    S.step_req = 1.0f;
    S.rate_mult = 1u;
    S.target_raw = RTL_RATE_TARGET_FRAMES;
    S.high_water_raw = RTL_RATE_TARGET_FRAMES * 12u / 5u;   /* == 6144, same as narrow mode always used before */
    rtl_rate_ctl_reset(&S.rc);

    TaskHandle_t ctl = NULL;
    if (xTaskCreatePinnedToCore(ctl_task, "rtl_ctl", 8192, NULL, 5, &ctl, 0) != pdPASS) {
        heap_caps_free(S.ring);
        S.ring = NULL;
        return ESP_ERR_NO_MEM;
    }
    S.ctl = ctl;
    return ESP_OK;
}

void rtl_source_set_freq(uint32_t lo_hz)
{
    S.want_lo_hz = lo_hz;
    S.last_tune_us = esp_timer_get_time();

    /* Within the digital window of the hardware LO (and not in WFM, which needs
     * the whole 192 kSps band): move the NCO only - instant, stream untouched.
     * A physical retune already in flight picks up the final value itself. */
    if (S.hw_valid && !S.want_wide && !__atomic_load_n(&S.lo_pending, __ATOMIC_ACQUIRE)) {
        const int32_t d = (int32_t)(lo_hz - S.hw_lo_hz);
        if (d >= -RTL_SOURCE_DIGITAL_WINDOW_HZ && d <= RTL_SOURCE_DIGITAL_WINDOW_HZ) {
            __atomic_store_n(&S.dig_offset_hz, d, __ATOMIC_RELEASE);
            return;
        }
    }
    __atomic_store_n(&S.lo_pending, true, __ATOMIC_RELEASE);
    if (S.ctl != NULL) {
        xTaskNotifyGive(S.ctl);
    }
}

void rtl_source_set_gain_db(int gain_db)
{
    if (gain_db < 0) gain_db = 0;
    if (gain_db > 50) gain_db = 50;
    S.want_gain_db = gain_db;
    S.want_gain_auto = false;
    __atomic_store_n(&S.gain_pending, true, __ATOMIC_RELEASE);
    if (S.ctl != NULL) {
        xTaskNotifyGive(S.ctl);
    }
}

void rtl_source_set_wide(bool wide)
{
    if (S.want_wide == wide) {
        return;   /* already there, or already requested: nothing to do */
    }
    S.want_wide = wide;
    __atomic_store_n(&S.wide_pending, true, __ATOMIC_RELEASE);
    if (wide && __atomic_load_n(&S.dig_offset_hz, __ATOMIC_ACQUIRE) != 0) {
        /* WFM uses the whole 192 kSps band: centre the dongle, drop the offset */
        __atomic_store_n(&S.lo_pending, true, __ATOMIC_RELEASE);
        if (S.ctl != NULL) {
            xTaskNotifyGive(S.ctl);
        }
    }
    /* No need to wake anyone: process_block() picks this up on its own very next
     * call, which arrives every ~8.5 ms regardless (USB delivery keeps running
     * unconditionally - a mode switch never stops or restarts the dongle). */
}

void rtl_source_set_gain_auto(bool enable)
{
    S.want_gain_auto = enable;
    __atomic_store_n(&S.gain_pending, true, __ATOMIC_RELEASE);
    if (S.ctl != NULL) {
        xTaskNotifyGive(S.ctl);
    }
}

esp_err_t rtl_source_read_float(float *i, float *q, size_t frames, uint32_t timeout_ms)
{
    if (S.ring == NULL || frames == 0 || frames > S.high_water_raw / 2) {
        return ESP_ERR_INVALID_STATE;
    }
    S.reader = xTaskGetCurrentTaskHandle();

    const TickType_t t0 = xTaskGetTickCount();
    const TickType_t tmo = pdMS_TO_TICKS(timeout_ms);
    uint32_t avail;

    for (;;) {
        avail = __atomic_load_n(&S.head, __ATOMIC_ACQUIRE) - S.tail;
        const uint32_t rm = S.rate_mult ? S.rate_mult : 1u;
        /* PRIME_FRAMES is a fixed 48 kSps-equivalent threshold (~75 ms): normalize the
         * raw ring level by the active rate's multiplier before comparing against it. */
        if (!S.primed && (avail / rm) >= RTL_RATE_PRIME_FRAMES) {
            rtl_rate_ctl_reset(&S.rc);
            S.step_req = 1.0f;
            S.primed = true;
        }
        if (S.primed && avail >= frames) {
            break;
        }
        const TickType_t elapsed = xTaskGetTickCount() - t0;
        if (elapsed >= tmo) {
            S.primed = false;
            S.step_req = 1.0f;
            S.underruns++;
            return ESP_ERR_TIMEOUT;
        }
        ulTaskNotifyTake(pdTRUE, tmo - elapsed);
    }

    /* head, then its arrival time: the acquire-load of head orders this after the
     * writer's plain store of arr_t_us32, so the pair is consistent (see the field
     * comment) with no lock and no possibility of spinning. */
    const uint32_t head = __atomic_load_n(&S.head, __ATOMIC_ACQUIRE);
    const uint32_t t_arr32 = S.arr_t_us32;

    uint32_t tail = S.tail;

    /* Far above target: the consumer stalled for a while. Jump to the target level.
     * Both thresholds are pre-scaled to the active rate's raw frames (see process_block()). */
    if (head - tail > S.high_water_raw) {
        const uint32_t skip = (head - tail) - S.target_raw;
        tail += skip;
        S.skipped += skip;
        rtl_rate_ctl_reset(&S.rc);
    }

    const uint32_t rm2 = S.rate_mult ? S.rate_mult : 1u;
    const uint32_t now32 = (uint32_t)esp_timer_get_time();
    /* Normalize back to 48 kSps-equivalent frames: rtl_level_estimate()'s own "since
     * last arrival" extrapolation is hardcoded in those units (RTL_DSP_OUT_RATE), and
     * the drift controller's constants (RTL_RATE_TARGET_FRAMES etc.) are too. */
    const float level = rtl_level_estimate((head - tail) / rm2, (int64_t)(now32 - t_arr32)); /* wraps correctly: both are the low 32 bits of the same clock, and the gap is always well under 2^31 us */
    S.step_req = rtl_rate_ctl_update(&S.rc, level, true);

    const float k = 1.0f / (float)INT16_MAX;
    for (size_t n = 0; n < frames; n++) {
        const iq16_t v = S.ring[(tail + n) & RING_MASK];
        i[n] = (float)v.i * k;
        q[n] = (float)v.q * k;
    }
    __atomic_store_n(&S.tail, tail + (uint32_t)frames, __ATOMIC_RELEASE);
    return ESP_OK;
}

bool rtl_source_is_streaming(void)
{
    return S.streaming;
}

void rtl_source_get_stats(rtl_source_stats_t *out)
{
    memset(out, 0, sizeof(*out));
    out->streaming = S.streaming;
    out->fifo_frames = __atomic_load_n(&S.head, __ATOMIC_ACQUIRE) - __atomic_load_n(&S.tail, __ATOMIC_ACQUIRE);
    out->step_ppm = (S.step_req - 1.0f) * 1e6f;
    out->wide = S.wide;
    out->fifo_overruns = S.fifo_overruns;
    out->underruns = S.underruns;
    out->skipped_frames = S.skipped;
    if (S.streaming) {
        esp_rtl_sdr_metrics_t m;
        if (esp_rtl_sdr_get_metrics(S.sdr, &m) == ESP_OK) {
            out->usb_overruns = m.overruns;
            out->usb_drops = m.consumer_drops;
            out->effective_sps = m.effective_sps;
        }
    }
}
