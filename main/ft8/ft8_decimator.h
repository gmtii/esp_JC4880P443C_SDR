#ifndef FT8_DECIMATOR_H
#define FT8_DECIMATOR_H

#include <stdint.h>
#include <stddef.h>

/*
 * Rational resampler 12 kHz -> 3200 Hz (interpolate by L=4, decimate by M=15,
 * direct polyphase form) feeding ft8_waterfall_adapter.c, plus the FT8
 * pipeline's own independent AGC. Ported from the DeepSDR 101 (GD32F450)
 * firmware - same 480-tap Blackman-windowed-sinc prototype (120 taps per
 * phase, cutoff 1600 Hz at the 48 kHz virtual rate, gain L=4), same AGC
 * constants, same output bookkeeping.
 *
 * WHY 3200 Hz: with freq_osr=2 kept (A/B tested on the GD32 against the
 * ft8_lib WAV corpus: dropping it lost real decodes, dropping time_osr did
 * not), 3200 Hz gives nfft = 3200*0.16*2 = 1024, a clean power of two, and a
 * 1600 Hz search window. 12000/3200 = 15/4, hence a real rational resampler.
 *
 * ESP32-P4 port differences:
 *  - Input is float (in int16 full-scale units, i.e. +-32768), not int16:
 *    the P4 receive chain is float end to end, and quantizing weak signals to
 *    int16 BEFORE this AGC would throw away dynamic range for nothing.
 *  - Runs entirely inside the FT8 task (ft8_app.c): no ISR, so the GD32's
 *    ISR->main-loop double buffer and the __disable_irq() critical section in
 *    reset are gone. Each completed 512-sample output block is handed to
 *    ft8_waterfall_feed_subblock() directly.
 *  - The 120-tap history is a doubled ring buffer instead of a shift
 *    register (no 120-element memmove per input sample).
 */
#define FT8_RESAMPLER_L          4     /* interpolation factor */
#define FT8_RESAMPLER_M          15    /* decimation factor - 12000*L/M = 3200 Hz exactly */
#define FT8_DECIM_INPUT_RATE_HZ  12000 /* must match sdr.c's post-decimation audio rate (SAMPLE_RATE/DR) */

/* Clears the resampler history, AGC state and output accumulation. Call at the
 * start of every new slot capture, together with ft8_waterfall_reset(). */
void ft8_decimator_reset(void);

/* Feeds n consecutive 12 kHz samples (float, int16 full-scale units). Runs the
 * AGC + polyphase resampler and calls ft8_waterfall_feed_subblock() every time
 * FT8_ADAPTER_SUBBLOCK_SIZE (256) output samples have accumulated (80 ms). */
void ft8_decimator_feed(const float *samples, size_t n);

/* Raw 12 kHz input samples fed since the last reset - independent of the
 * resampler bookkeeping. Divided by the capture's wall-clock time it should
 * read ~12000 Hz (diagnostic kept from the GD32 slot-drift investigation). */
uint32_t ft8_decimator_get_raw_sample_count(void);

#endif /* FT8_DECIMATOR_H */
