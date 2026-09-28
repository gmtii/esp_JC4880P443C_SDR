/* audio_out.c - see audio_out.h */
#include "audio_out.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_codec_dev.h"
#include "bsp/esp-bsp.h"

static const char *TAG = "audio_out";

static esp_codec_dev_handle_t s_spk;
static uint32_t s_rate = 48000;
static volatile int s_volume = 50;

/*
 * The BSP's bsp_extra_codec_init() is not used on purpose: it also creates the
 * microphone path (and asserts if that fails) and opens both directions at
 * 44.1 kHz. Here only the speaker handle is needed.
 */
esp_err_t audio_out_init(uint32_t sample_rate)
{
    if (s_spk != NULL) {
        return ESP_OK;
    }

    esp_codec_dev_handle_t spk = bsp_audio_codec_speaker_init();
    if (spk == NULL) {
        ESP_LOGE(TAG, "bsp_audio_codec_speaker_init() failed");
        return ESP_FAIL;
    }

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = sample_rate,
        .channel = 2,
        .bits_per_sample = 16,
    };
    const int ret = esp_codec_dev_open(spk, &fs);
    if (ret != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "esp_codec_dev_open(%u Hz) failed: %d", (unsigned)sample_rate, ret);
        return ESP_FAIL;
    }

    s_rate = sample_rate;
    s_spk = spk;
    (void)esp_codec_dev_set_out_mute(s_spk, false);
    (void)esp_codec_dev_set_out_vol(s_spk, s_volume);
    ESP_LOGI(TAG, "ES8311 playback open: %u Hz, 16-bit stereo, volume %d%%", (unsigned)s_rate, s_volume);
    return ESP_OK;
}

esp_err_t audio_out_write(const int16_t *stereo_frames, size_t frames)
{
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (s_spk != NULL) {
        const int ret = esp_codec_dev_write(s_spk, (void *)stereo_frames, (int)(frames * 2u * sizeof(int16_t)));
        if (ret == ESP_CODEC_DEV_OK) {
            return ESP_OK;
        }
        err = ESP_FAIL;
    }
    /* Not open or write failed: keep the caller's loop paced instead of spinning */
    TickType_t t = pdMS_TO_TICKS((uint32_t)(frames * 1000u / s_rate));
    vTaskDelay(t ? t : 1);
    return err;
}

void audio_out_set_volume(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    s_volume = percent;
    if (s_spk != NULL) {
        (void)esp_codec_dev_set_out_vol(s_spk, percent);
    }
}

int audio_out_get_volume(void)
{
    return s_volume;
}
