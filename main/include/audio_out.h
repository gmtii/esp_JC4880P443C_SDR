/*
 * audio_out.h - audio output through the on-board ES8311 codec (playback only).
 *
 * Replaces the external NAU8822 + custom I2S driver. The codec, its I2S port,
 * MCLK and the power-amplifier pin are set up by the board support package
 * (bsp_audio_codec_speaker_init()); this module only opens the playback path at
 * the SDR audio rate and writes 16-bit stereo frames to it.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the codec and open playback: 16-bit, stereo, sample_rate Hz. */
esp_err_t audio_out_init(uint32_t sample_rate);

/*
 * Write `frames` interleaved stereo int16 frames (L,R,L,R,...). Blocks until the
 * I2S DMA has taken the data, so the caller's loop is paced by the codec clock.
 * On failure it sleeps for the duration of the block, so a caller that loops on
 * it cannot spin the CPU.
 */
esp_err_t audio_out_write(const int16_t *stereo_frames, size_t frames);

/* Output volume, 0..100 (percent of the codec's range). Safe from any task. */
void audio_out_set_volume(int percent);
int  audio_out_get_volume(void);

#ifdef __cplusplus
}
#endif
