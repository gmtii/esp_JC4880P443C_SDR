#include "ft8_app.h"

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/idf_additions.h" /* xTaskCreatePinnedToCoreWithCaps(), xStreamBufferCreateWithCaps() */
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "ft8_ram.h"
#include "ft8_decimator.h"
#include "ft8_waterfall_adapter.h"
#include "ft8_decoder.h"
#include "ft8_time.h"
#include "time_sync.h"
#include "ft8_band_sync.h"

static const char *TAG = "FT8";

/* sdr.c works in +-1.0 float; the ported pipeline (AGC target, dB offset) was
 * calibrated in int16 full-scale units on the GD32. */
#define FT8_INPUT_SCALE 32768.0f

/* Audio queued between sdrTask and the FT8 task. It must cover the longest
 * decode (samples keep arriving while a slot is being decoded - they belong
 * to the NEXT slot and are consumed right after). The GD32 needed ~0.3 s with
 * an empty band and more on a busy one; the P4 is faster, 3 s is ample. */
#define FT8_STREAM_SECONDS 3
#define FT8_STREAM_BYTES (FT8_DECIM_INPUT_RATE_HZ * FT8_STREAM_SECONDS * sizeof(float))
#define FT8_RX_CHUNK 256 /* floats per receive - one sdrTask block at 12 kHz */

/* A clock correction larger than this invalidates the running capture (its
 * audio no longer lines up with the slot grid). Smaller ones (routine serial
 * resyncs) are absorbed - FT8's candidate search tolerates small DT offsets. */
#define FT8_RESYNC_THRESHOLD_MS 200

#define FT8_TASK_STACK 16384 /* bp_decode() alone needs ~4.4 KB, see ft8_decoder.h */
#define FT8_TASK_PRIO 3      /* below LVGL, far below sdrTask (20) */
#define FT8_TASK_CORE 0      /* sdrTask owns core 1 */

#define FT8_SLOT_MS 15000

static StreamBufferHandle_t s_sb;
static volatile bool s_active;
static bool s_available;
static volatile uint32_t s_overruns;

static ft8_status_t s_status;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;

/* ------------------------------------------------------------------------- */
/* Own grid persistence (NVS namespace "ft8", key "grid")                      */
/* ------------------------------------------------------------------------- */

static void grid_load_from_nvs(void)
{
    nvs_handle_t h;
    char grid[8];
    size_t len = sizeof(grid);

    if (nvs_open("ft8", NVS_READONLY, &h) != ESP_OK)
    {
        return; /* nothing stored yet */
    }
    if (nvs_get_str(h, "grid", grid, &len) == ESP_OK && len > 1)
    {
        ft8_decoder_set_own_grid(grid, (int)(len - 1));
        ESP_LOGI(TAG, "own grid from NVS: %s", ft8_decoder_get_own_grid());
    }
    nvs_close(h);
}

/* time_sync.c callback, runs in the UART task after MSG_SET_GRID. */
static void grid_save_to_nvs(const char *grid)
{
    nvs_handle_t h;

    if (nvs_open("ft8", NVS_READWRITE, &h) != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS open failed, grid %s not persisted", grid);
        return;
    }
    if (nvs_set_str(h, "grid", grid) == ESP_OK)
    {
        nvs_commit(h);
    }
    nvs_close(h);
    ESP_LOGI(TAG, "own grid set to %s (saved)", grid);
}

/* ------------------------------------------------------------------------- */
/* FT8 task                                                                   */
/* ------------------------------------------------------------------------- */

static void status_update_live(bool capturing)
{
    float err;
    bool have = ft8_bsync_get_last_error(&err);

    portENTER_CRITICAL(&s_status_lock);
    s_status.active = s_active;
    s_status.capturing = capturing;
    s_status.blocks = ft8_waterfall_get()->num_blocks;
    s_status.overruns = s_overruns;
    s_status.sync_state = (int)ft8_bsync_get_state();
    s_status.sync_have_err = have;
    s_status.sync_err_s = err;
    portEXIT_CRITICAL(&s_status_lock);
}

static void start_capture(void)
{
    ft8_decimator_reset();
    ft8_waterfall_reset();
}

/* Snapshot + immediate re-arm + decode - same order as the GD32 fix for the
 * structural "capture (14.88 s) + decode > 15 s" drift: the next capture must
 * start at the boundary no matter how long this slot's decode takes. */
static int32_t finish_slot_and_rearm(void)
{
    float db_min, db_max;
    uint32_t raw = ft8_decimator_get_raw_sample_count();
    int64_t t0;
    uint32_t proc_ms;
    struct tm slot;

    ft8_waterfall_snapshot_mag();
    ft8_waterfall_get_db_range(&db_min, &db_max); /* before reset clears it */

    start_capture(); /* next slot starts counting now */

    t0 = esp_timer_get_time();
    ft8_decoder_process_slot();
    proc_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

    portENTER_CRITICAL(&s_status_lock);
    s_status.slots_processed++;
    s_status.total_decodes = ft8_decoder_get_total_count();
    s_status.last_decoded = ft8_decoder_get_last_num_decoded();
    s_status.last_candidates = ft8_decoder_get_last_num_candidates();
    s_status.last_proc_ms = proc_ms;
    s_status.last_db_min = db_min;
    s_status.last_db_max = db_max;
    portEXIT_CRITICAL(&s_status_lock);

    /* Band-sync loop: time every CRC-valid decode, steer the clock. */
    {
        ft8_bsync_input_t in;
        int32_t delta;
        float err = 0.0f;

        in.n_dt = ft8_decoder_get_last_timing(&in.dt_s);
        in.top_score = ft8_decoder_get_last_top_score();
        delta = ft8_bsync_slot(&in);

        ft8_time_get_slot_start_utc(&slot);
        (void)ft8_bsync_get_last_error(&err);
        ESP_LOGI(TAG, "slot %02d:%02d:%02d  dec=%d cand=%d top=%d proc=%lums dB[%d..%d] in=%lu  sync=%s medDT=%+.3f corr=%+ldms",
                 slot.tm_hour, slot.tm_min, slot.tm_sec,
                 ft8_decoder_get_last_num_decoded(), ft8_decoder_get_last_num_candidates(), in.top_score,
                 (unsigned long)proc_ms, (int)db_min, (int)db_max, (unsigned long)raw,
                 ft8_bsync_state_name(ft8_bsync_get_state()), in.n_dt ? err : 0.0f, (long)delta);
        return delta;
    }
}

static void ft8_task(void *arg)
{
    (void)arg;
    static float buf[FT8_RX_CHUNK];
    bool was_active = false;
    bool capturing = false;
    int64_t cur_slot = 0;
    uint32_t seen_seq = 0;

    for (;;)
    {
        size_t got = xStreamBufferReceive(s_sb, buf, sizeof(buf), pdMS_TO_TICKS(50));
        bool active = s_active;

        if (!active)
        {
            /* Nothing is fed while inactive; just idle. */
            if (was_active)
            {
                was_active = false;
                capturing = false;
                status_update_live(false);
            }
            continue;
        }

        if (!was_active)
        {
            /* Mode entry: drop anything stale, then wait for the NEXT boundary
             * (never arm mid-slot - the capture would be misaligned). */
            was_active = true;
            capturing = false;
            while (xStreamBufferReceive(s_sb, buf, sizeof(buf), 0) > 0)
            {
            }
            got = 0;
            start_capture(); /* keeps the cascade running meanwhile */
            seen_seq = ft8_time_get_sync_seq();
            cur_slot = ft8_time_now_ms() / FT8_SLOT_MS;
            ft8_bsync_reset(ft8_time_is_synced());
            ESP_LOGI(TAG, "FT8 on, waiting for the next slot boundary (clock: %s, band sync will steer it)",
                     ft8_time_source_name(ft8_time_get_source()));
        }

        if (got > 0)
        {
            size_t n = got / sizeof(float);
            for (size_t i = 0; i < n; i++)
            {
                buf[i] *= FT8_INPUT_SCALE;
            }
            ft8_decimator_feed(buf, n); /* also runs FFT/encode per 80 ms subblock */
        }

        /* Clock corrected under us? */
        {
            uint32_t seq = ft8_time_get_sync_seq();
            if (seq != seen_seq)
            {
                int32_t jump = ft8_time_get_last_jump_ms();
                seen_seq = seq;
                ft8_bsync_external_set(); /* serial tool / manual entry: roughly right now */
                if (abs(jump) > FT8_RESYNC_THRESHOLD_MS)
                {
                    capturing = false;
                    cur_slot = ft8_time_now_ms() / FT8_SLOT_MS;
                    ESP_LOGI(TAG, "clock moved %ld ms, re-arming at the next boundary", (long)jump);
                }
            }
        }

        /* Slot boundary: the 15 s slot index changed. Comparing indices (not
         * "second == 0/15/30/45") can't miss an edge even if this loop is late,
         * and carries the sub-second precision of the system clock. */
        {
            int64_t slot = ft8_time_now_ms() / FT8_SLOT_MS;
            if (slot != cur_slot)
            {
                cur_slot = slot;
                if (capturing && ft8_waterfall_is_full())
                {
                    int32_t delta = finish_slot_and_rearm();
                    if (delta != 0)
                    {
                        ft8_time_trim_ms(delta);
                        if (abs(delta) > FT8_BSYNC_RESTART_MS)
                        {
                            /* Too big for the capture already running to
                             * absorb: drop it, re-arm on the new grid. */
                            capturing = false;
                            ft8_bsync_capture_restarted();
                        }
                        /* Re-read the index after moving the clock: a
                         * backward trim can step back across the boundary
                         * just handled (it is then simply handled again,
                         * re-arming on the corrected grid). */
                        cur_slot = ft8_time_now_ms() / FT8_SLOT_MS;
                    }
                }
                else
                {
                    /* First boundary after entry/resync/correction, or a
                     * short capture (audio stalled, e.g. dongle retune):
                     * just (re)arm. The next decoded slot is then fully on
                     * the current grid. */
                    start_capture();
                    capturing = true;
                    ft8_bsync_capture_restarted();
                }
            }
        }

        status_update_live(capturing);
    }
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                 */
/* ------------------------------------------------------------------------- */

bool ft8_app_init(void)
{
    esp_err_t err;
    TaskHandle_t task = NULL;

    if (s_available)
    {
        return true;
    }

    if (!ft8_ram_init())
    {
        ESP_LOGE(TAG, "buffer allocation failed (%u bytes), FT8 disabled", (unsigned)sizeof(ft8_ram_t));
        return false;
    }

    ft8_decoder_init();

    err = nvs_flash_init(); /* ESP_OK if already initialized elsewhere */
    if (err == ESP_OK)
    {
        grid_load_from_nvs();
    }
    else
    {
        ESP_LOGW(TAG, "nvs_flash_init: %s - own grid will not persist", esp_err_to_name(err));
    }
    time_sync_set_grid_callback(grid_save_to_nvs);

    s_sb = xStreamBufferCreateWithCaps(FT8_STREAM_BYTES, FT8_RX_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    if (s_sb == NULL)
    {
        ESP_LOGE(TAG, "stream buffer allocation failed, FT8 disabled");
        return false;
    }

    /* Stack in PSRAM (internal RAM is tight on this board). This task never
     * touches flash, so an external-RAM stack is safe. Fall back to internal
     * RAM if the PSRAM variant is not available in this configuration. */
    if (xTaskCreatePinnedToCoreWithCaps(ft8_task, "ft8", FT8_TASK_STACK, NULL, FT8_TASK_PRIO,
                                        &task, FT8_TASK_CORE, MALLOC_CAP_SPIRAM) != pdPASS)
    {
        ESP_LOGW(TAG, "PSRAM stack not available, using internal RAM");
        if (xTaskCreatePinnedToCore(ft8_task, "ft8", FT8_TASK_STACK, NULL, FT8_TASK_PRIO,
                                    &task, FT8_TASK_CORE) != pdPASS)
        {
            ESP_LOGE(TAG, "task creation failed, FT8 disabled");
            return false;
        }
    }

    s_available = true;
    ft8_bsync_reset(false);
    ESP_LOGI(TAG, "ready: 1600 Hz window (0-1600 Hz audio), own grid %s", ft8_decoder_get_own_grid());
    return true;
}

bool ft8_app_available(void)
{
    return s_available;
}

void ft8_app_set_active(bool on)
{
    if (!s_available)
    {
        return;
    }
    s_active = on;
}

bool ft8_app_is_active(void)
{
    return s_active;
}

void ft8_app_feed_audio(const float *samples, size_t n)
{
    if (!s_active || s_sb == NULL)
    {
        return;
    }
    /* All or nothing: a partial write could end mid-float and desynchronize
     * the reader's sample framing for good. Single writer, so the free space
     * can only grow between this check and the send. */
    if (xStreamBufferSpacesAvailable(s_sb) < n * sizeof(float))
    {
        s_overruns++;
        return;
    }
    (void)xStreamBufferSend(s_sb, samples, n * sizeof(float), 0);
}

void ft8_app_get_status(ft8_status_t *out)
{
    portENTER_CRITICAL(&s_status_lock);
    *out = s_status;
    portEXIT_CRITICAL(&s_status_lock);
    out->active = s_active;
}
