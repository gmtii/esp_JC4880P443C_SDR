#include "ais_app.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "ais_demod.h"
#include "ais_msg.h"
#include "ft8_time.h"

static const char *TAG = "AIS";

#define AIS_CHUNK 4096 /* complex samples per transfer = one wide sdrTask block */
#define AIS_STREAM_BYTES (AIS_IN_RATE / 2 * 2 * sizeof(float)) /* 0.5 s of IQ */
#define AIS_TASK_STACK 8192
#define AIS_TASK_PRIO 4
#define AIS_CLOCK_MIN_INTERVAL_US (600LL * 1000000LL) /* set the clock at most every 10 min */

static StreamBufferHandle_t s_sb;
/* Work buffers live in PSRAM (allocated in ais_app_init): as static arrays
 * they took ~96 KB of internal RAM and made the link fail (DRAM overflow). */
static float *s_buf;      /* 2 * AIS_CHUNK, task side  */
static float *s_bi, *s_bq; /* AIS_CHUNK each, task side */
static float *s_tmp;      /* 2 * AIS_CHUNK, sdrTask side */
static volatile bool s_active;
static bool s_available;
static int64_t s_last_clock_set_us = -AIS_CLOCK_MIN_INTERVAL_US;

static void log_line(const char *l)
{
    ESP_LOGI(TAG, "%s", l);
}

static void on_frame(const ais_frame_t *f, void *ctx)
{
    (void)ctx;
    ais_msg_handle(f);
}

/* Base station UTC: only when nothing better (serial tool, NTP) set the clock */
static void on_utc(int y, int mo, int d, int h, int mi, int s)
{
    const ft8_time_src_t src = ft8_time_get_source();
    const int64_t now = esp_timer_get_time();
    if (src == FT8_TIME_SRC_SERIAL || src == FT8_TIME_SRC_NTP)
    {
        return;
    }
    if (src == FT8_TIME_SRC_AIS && now - s_last_clock_set_us < AIS_CLOCK_MIN_INTERVAL_US)
    {
        return;
    }
    s_last_clock_set_us = now;
    ft8_time_set_utc_fields(y, mo, d, h, mi, s, FT8_TIME_SRC_AIS);
    ESP_LOGI(TAG, "clock set from base station: %04d-%02d-%02d %02d:%02d:%02d UTC", y, mo, d, h, mi, s);
}

static void ais_task(void *arg)
{
    float *const buf = s_buf, *const bi = s_bi, *const bq = s_bq;
    const size_t buf_bytes = 2 * AIS_CHUNK * sizeof(float);
    bool was_active = false;
    (void)arg;

    for (;;)
    {
        size_t got = xStreamBufferReceive(s_sb, buf, buf_bytes, pdMS_TO_TICKS(50));
        if (!s_active)
        {
            was_active = false;
            continue;
        }
        if (!was_active)
        {
            was_active = true;
            while (xStreamBufferReceive(s_sb, buf, buf_bytes, 0) > 0)
            {
            }
            got = 0;
            ais_demod_reset();
            ESP_LOGI(TAG, "AIS on (162.000 MHz, channels 87B/88B)");
        }
        if (got >= 2 * sizeof(float))
        {
            size_t n = got / (2 * sizeof(float));
            for (size_t k = 0; k < n; k++)
            {
                bi[k] = buf[2 * k];
                bq[k] = buf[2 * k + 1];
            }
            ais_demod_feed(bi, bq, (int)n);
        }
    }
}

bool ais_app_init(void)
{
    TaskHandle_t t = NULL;
    if (s_available)
    {
        return true;
    }
    s_buf = heap_caps_malloc(2 * AIS_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    s_bi = heap_caps_malloc(AIS_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    s_bq = heap_caps_malloc(AIS_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    s_tmp = heap_caps_malloc(2 * AIS_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!s_buf || !s_bi || !s_bq || !s_tmp || !ais_demod_alloc() || !ais_msg_alloc())
    {
        ESP_LOGE(TAG, "buffer allocation failed, AIS disabled");
        return false;
    }
    ais_msg_reset();
    ais_msg_set_printer(log_line);
    ais_msg_set_utc_callback(on_utc);
    ais_demod_init(on_frame, NULL);
    s_sb = xStreamBufferCreateWithCaps(AIS_STREAM_BYTES, 2 * AIS_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    if (s_sb == NULL)
    {
        ESP_LOGE(TAG, "stream buffer allocation failed, AIS disabled");
        return false;
    }
    if (xTaskCreatePinnedToCoreWithCaps(ais_task, "ais", AIS_TASK_STACK, NULL, AIS_TASK_PRIO, &t,
                                        tskNO_AFFINITY, MALLOC_CAP_SPIRAM) != pdPASS &&
        xTaskCreatePinnedToCore(ais_task, "ais", AIS_TASK_STACK, NULL, AIS_TASK_PRIO, &t, tskNO_AFFINITY) != pdPASS)
    {
        ESP_LOGE(TAG, "task creation failed, AIS disabled");
        return false;
    }
    s_available = true;
    ESP_LOGI(TAG, "ready");
    return true;
}

bool ais_app_available(void) { return s_available; }
void ais_app_set_active(bool on) { if (s_available) s_active = on; }
bool ais_app_is_active(void) { return s_active; }

void ais_app_feed_iq(const float *i, const float *q, size_t n)
{
    float *const tmp = s_tmp; /* only ever used from sdrTask */
    if (!s_active || s_sb == NULL)
    {
        return;
    }
    while (n > 0)
    {
        size_t m = n > AIS_CHUNK ? AIS_CHUNK : n;
        for (size_t k = 0; k < m; k++)
        {
            tmp[2 * k] = i[k];
            tmp[2 * k + 1] = q[k];
        }
        if (xStreamBufferSpacesAvailable(s_sb) >= m * 2 * sizeof(float))
        {
            (void)xStreamBufferSend(s_sb, tmp, m * 2 * sizeof(float), 0); /* all or nothing */
        }
        i += m;
        q += m;
        n -= m;
    }
}
