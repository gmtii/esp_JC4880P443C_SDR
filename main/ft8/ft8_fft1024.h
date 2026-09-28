#ifndef FT8_FFT1024_H
#define FT8_FFT1024_H

#include <stdint.h>

/*
 * Dedicated 1024-point real-input FFT for FT8's 1600 Hz search window.
 * Ported from the DeepSDR 101 (GD32F450) firmware: same radix-2 Cooley-Tukey,
 * same Hann window, same libm-free fast log2 "dB" formula - which is what keeps
 * FT8_ADAPTER_DB_OFFSET (ft8_waterfall_adapter.h, tuned on real hardware there)
 * meaningful here too.
 *
 * Differences from the GD32 version:
 *  - The twiddle and Hann tables are computed with real libm sinf()/cosf()
 *    instead of the Bhaskara-I approximation (one-time cost at init, slightly
 *    more accurate transform, no change to the output scale).
 *  - The tables live in the PSRAM block from ft8_ram.h, allocated once and
 *    never shared with anything else, so the GD32's "re-run init on every FT8
 *    mode entry because the waterfall overwrote the shared union" workaround is
 *    no longer needed (calling init again is still harmless).
 */
#define FT8_FFT_SIZE        1024U
#define FT8_FFT_BINS_USEFUL (FT8_FFT_SIZE / 2U) /* 512 - real input: bins 0..Nyquist */

/* Precomputes the twiddle/Hann/bit-reversal tables. Requires ft8_ram_init(). */
void ft8_fft1024_init(void);

/*
 * Input: `samples` (FT8_FFT_SIZE real samples). Output: `db_out`
 * (FT8_FFT_BINS_USEFUL values, uncalibrated relative "dB"). Applies a Hann
 * window internally before transforming.
 */
void ft8_fft1024_compute_db(const int16_t *samples, float *db_out);

#endif /* FT8_FFT1024_H */
