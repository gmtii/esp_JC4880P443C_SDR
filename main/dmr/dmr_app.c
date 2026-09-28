#include "dmr_app.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "dmr_demod.h"
#include "dmr_proto.h"
#include "dmr_voice.h"
#include <math.h>

static const char *TAG = "DMR";

#define DMR_CHUNK 1024 /* complex samples per transfer = one sdrTask block */
#define DMR_STREAM_BYTES (48000 * 2 * sizeof(float)) /* 1 s of IQ */
#define DMR_TASK_STACK 8192
/* Priority above LVGL (esp_lvgl_port default 4) and no core affinity: with
 * voice the vocoder needs steady CPU, and whichever core is free takes it.
 * sdrTask (20) still pre-empts it; the UI only yields while a burst decodes. */
#define DMR_TASK_PRIO 5
#define DMR_TASK_CORE tskNO_AFFINITY

static StreamBufferHandle_t s_sb;
/* Work buffers in PSRAM (allocated in dmr_app_init), not static internal RAM */
static float *s_buf, *s_bi, *s_bq, *s_tmp, *s_up_out;

/* ---- Voice audio path (phase 2) -------------------------------------------
 * dmr_voice (DMR task) -> 8 kHz -> x6 polyphase interpolator -> 48 kHz float
 * stream buffer (PSRAM, 1 s) -> sdrTask (dmr_app_read_audio) -> codec.
 * Speech arrives in 60 ms bursts per slot, so playout waits for a 200 ms
 * cushion at the start of each call and re-buffers after an underrun. */
#define AUD_RATE_UP 6
#define AUD_TAPS 48 /* 8 per phase */
#define AUD_STREAM_BYTES (48000 * sizeof(float))
#define AUD_PREBUFFER (48000 / 5)     /* 200 ms before the first word */
#define AUD_REBUFFER (48000 * 12 / 100) /* 120 ms after a mid-call underrun: two bursts, so the
                                         * ~30 ms per-burst vocoder time measured on the P4
                                         * (plus IQ chunking) does not trip it again at once */
static StreamBufferHandle_t s_aud;
static float s_up_taps[AUD_TAPS];
static float s_up_hist[2 * (AUD_TAPS / AUD_RATE_UP)];
static int s_up_idx;
static bool s_playing; /* sdrTask side */
static bool s_underrun_armed;   /* sdrTask side: a call is in progress */
static bool s_fade_in;          /* sdrTask side: ramp the first samples after (re)start */
#define AUD_RAMP 96             /* 2 ms at 48 kHz */
static volatile uint32_t s_iq_drops, s_underruns;
static volatile bool s_active;
static bool s_available;

static void up_design(void)
{
    const float pi = 3.14159265358979f, fc = 3600.0f / 48000.0f;
    for (int i = 0; i < AUD_TAPS; i++)
    {
        float m = (float)i - (AUD_TAPS - 1) / 2.0f;
        float s = (fabsf(m) < 1e-6f) ? 2.0f * fc : sinf(2.0f * pi * fc * m) / (pi * m);
        float w = 0.54f - 0.46f * cosf(2.0f * pi * i / (AUD_TAPS - 1));
        s_up_taps[i] = s * w * AUD_RATE_UP; /* x6: interpolation gain */
    }
}

/* dmr_voice sink (DMR task): 8 kHz in, 48 kHz out to the audio stream. */
#define UP_OUT_LEN (160 * AUD_RATE_UP)
static void voice_sink(const float *pcm, int n)
{
    float *const out = s_up_out; /* UP_OUT_LEN, PSRAM */
    const int per_phase = AUD_TAPS / AUD_RATE_UP;
    int o = 0;

    for (int k = 0; k < n; k++)
    {
        const float *h;
        s_up_hist[s_up_idx] = pcm[k];
        s_up_hist[s_up_idx + per_phase] = pcm[k];
        s_up_idx = (s_up_idx + 1) % per_phase;
        h = &s_up_hist[s_up_idx]; /* oldest first */
        for (int p = 0; p < AUD_RATE_UP; p++)
        {
            float acc = 0.0f;
            for (int j = 0; j < per_phase; j++)
            {
                acc += s_up_taps[p + AUD_RATE_UP * (per_phase - 1 - j)] * h[j];
            }
            out[o++] = acc;
        }
        if (o == UP_OUT_LEN)
        {
            if (xStreamBufferSpacesAvailable(s_aud) >= UP_OUT_LEN * sizeof(float))
            {
                (void)xStreamBufferSend(s_aud, out, UP_OUT_LEN * sizeof(float), 0);
            }
            o = 0;
        }
    }
    if (o > 0 && xStreamBufferSpacesAvailable(s_aud) >= (size_t)o * sizeof(float))
    {
        (void)xStreamBufferSend(s_aud, out, (size_t)o * sizeof(float), 0);
    }
}

static void log_line(const char *line)
{
    ESP_LOGI(TAG, "%s", line);
}

static void dmr_task(void *arg)
{
    (void)arg;
    float *const buf = s_buf, *const bi = s_bi, *const bq = s_bq;
    const size_t buf_bytes = 2 * DMR_CHUNK * sizeof(float);
    bool was_active = false;
    int64_t next_stats = 0;

    for (;;)
    {
        size_t got = xStreamBufferReceive(s_sb, buf, buf_bytes, pdMS_TO_TICKS(50));
        bool active = s_active;

        if (!active)
        {
            was_active = false;
            continue;
        }
        if (!was_active)
        {
            /* mode entry: drop stale IQ, start from a clean demodulator */
            was_active = true;
            while (xStreamBufferReceive(s_sb, buf, buf_bytes, 0) > 0)
            {
            }
            got = 0;
            dmr_demod_reset();
            ESP_LOGI(TAG, "DMR on");
        }
        if (got >= 2 * sizeof(float))
        {
            size_t n = got / (2 * sizeof(float));
            for (size_t k = 0; k < n; k++)
            {
                bi[k] = buf[2 * k];
                bq[k] = buf[2 * k + 1];
            }
            dmr_demod_feed_iq(bi, bq, (int)n);
        }
        if (esp_timer_get_time() >= next_stats)
        {
            uint32_t frames, avg_us;
            next_stats = esp_timer_get_time() + 10000000;
            dmr_voice_get_timing(&frames, &avg_us);
            ESP_LOGI(TAG, "stats: IQ blocks dropped %lu, audio underruns %lu, vocoder %lu frames avg %lu us/frame (budget 20000), concealed %lu",
                     (unsigned long)s_iq_drops, (unsigned long)s_underruns, (unsigned long)frames, (unsigned long)avg_us,
                     (unsigned long)dmr_voice_get_concealed());
        }
    }
}

bool dmr_app_init(void)
{
    TaskHandle_t task = NULL;

    if (s_available)
    {
        return true;
    }
    s_buf = heap_caps_malloc(2 * DMR_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    s_bi = heap_caps_malloc(DMR_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    s_bq = heap_caps_malloc(DMR_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    s_tmp = heap_caps_malloc(2 * DMR_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    s_up_out = heap_caps_malloc(UP_OUT_LEN * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!s_buf || !s_bi || !s_bq || !s_tmp || !s_up_out || !dmr_demod_init())
    {
        ESP_LOGE(TAG, "buffer allocation failed, DMR disabled");
        return false;
    }
    dmr_proto_set_printer(log_line);

    up_design();
    s_aud = xStreamBufferCreateWithCaps(AUD_STREAM_BYTES, sizeof(float), MALLOC_CAP_SPIRAM);
    if (s_aud != NULL && dmr_voice_available())
    {
        dmr_voice_set_sink(voice_sink);
    }

    s_sb = xStreamBufferCreateWithCaps(DMR_STREAM_BYTES, 2 * DMR_CHUNK * sizeof(float), MALLOC_CAP_SPIRAM);
    if (s_sb == NULL)
    {
        ESP_LOGE(TAG, "stream buffer allocation failed, DMR disabled");
        return false;
    }
    if (xTaskCreatePinnedToCoreWithCaps(dmr_task, "dmr", DMR_TASK_STACK, NULL, DMR_TASK_PRIO, &task,
                                        DMR_TASK_CORE, MALLOC_CAP_SPIRAM) != pdPASS &&
        xTaskCreatePinnedToCore(dmr_task, "dmr", DMR_TASK_STACK, NULL, DMR_TASK_PRIO, &task, DMR_TASK_CORE) != pdPASS)
    {
        ESP_LOGE(TAG, "task creation failed, DMR disabled");
        return false;
    }
    s_available = true;
    ESP_LOGI(TAG, "ready: %s", dmr_voice_available() ? "metadata + voice (mbelib)" : "metadata only (voice needs mbelib, see README_DMR.md)");
    return true;
}

bool dmr_app_available(void)
{
    return s_available;
}

void dmr_app_set_active(bool on)
{
    if (s_available)
    {
        s_active = on;
    }
}

bool dmr_app_is_active(void)
{
    return s_active;
}

void dmr_app_feed_iq(const float *i, const float *q, size_t n)
{
    float *const tmp = s_tmp; /* only ever used from sdrTask */

    if (!s_active || s_sb == NULL)
    {
        return;
    }
    while (n > 0)
    {
        size_t m = (n > DMR_CHUNK) ? DMR_CHUNK : n;
        for (size_t k = 0; k < m; k++)
        {
            tmp[2 * k] = i[k];
            tmp[2 * k + 1] = q[k];
        }
        /* all or nothing, so the reader never loses I/Q pairing */
        if (xStreamBufferSpacesAvailable(s_sb) >= m * 2 * sizeof(float))
        {
            (void)xStreamBufferSend(s_sb, tmp, m * 2 * sizeof(float), 0);
        }
        else
        {
            s_iq_drops++; /* the DMR task is not keeping up */
        }
        i += m;
        q += m;
        n -= m;
    }
}

void dmr_app_read_audio(float *out, size_t n)
{
    size_t got = 0;

    if (s_aud != NULL && s_active)
    {
        size_t avail = xStreamBufferBytesAvailable(s_aud) / sizeof(float);
        if (!s_playing && avail >= (s_underrun_armed ? AUD_REBUFFER : AUD_PREBUFFER))
        {
            s_playing = true;
            s_fade_in = true;
            s_underrun_armed = true;
        }
        if (s_playing)
        {
            got = xStreamBufferReceive(s_aud, out, n * sizeof(float), 0) / sizeof(float);
            if (s_fade_in)
            {
                /* 2 ms fade-in: no click when speech (re)starts */
                for (size_t k = 0; k < got && k < AUD_RAMP; k++)
                {
                    out[k] *= (float)k / AUD_RAMP;
                }
                s_fade_in = false;
            }
            if (got < n)
            {
                /* underrun: a gap in the call (short re-buffer) or its end.
                 * 2 ms fade-out on what we have, so it stops without a click. */
                size_t r = got < AUD_RAMP ? got : AUD_RAMP;
                for (size_t k = 0; k < r; k++)
                {
                    out[got - r + k] *= (float)(r - k) / (float)r;
                }
                s_playing = false;
                s_underruns++;
            }
        }
        if (!s_playing && dmr_voice_get_playing() < 0)
        {
            s_underrun_armed = false; /* call over: next one gets the full cushion */
        }
    }
    for (size_t k = got; k < n; k++)
    {
        out[k] = 0.0f;
    }
}

void dmr_app_get_stats(uint32_t *iq_drops, uint32_t *underruns)
{
    *iq_drops = s_iq_drops;
    *underruns = s_underruns;
}
