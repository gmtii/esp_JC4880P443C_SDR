#ifndef FT8_RAM_H
#define FT8_RAM_H

#include <stdint.h>
#include <stdbool.h>
#include "ft8_fft1024.h"
#include "ft8_waterfall_adapter.h"

/*
 * Every large buffer the FT8 pipeline needs, in ONE block allocated in PSRAM
 * at startup (ft8_ram_init()).
 *
 * Replaces the GD32F450's ft8_shared_ram.h, which overlaid all of this on the
 * main waterfall's pixel buffer (a real C union) because that MCU only has
 * 192 KB of SRAM. The ESP32-P4 board has 32 MB of PSRAM, so none of that RAM
 * lending is needed any more - and with it goes the whole class of bug the
 * GD32 hit (FFT tables silently overwritten by the waterfall's legitimate use
 * of the same memory before FT8 mode was ever entered). Field names and sizes
 * are kept identical so the ported code reads the same.
 *
 * Total: ~214 KB (mag + mag_snapshot = 2 x 95232 B with time_osr=2, the rest ~23 KB).
 */
#define FT8_WF_HISTORY_ROWS 16 /* cascade history depth - see ft8_waterfall_adapter.c */

typedef struct
{
    uint8_t mag[FT8_ADAPTER_MAX_BLOCKS * FT8_ADAPTER_TIME_OSR * FT8_ADAPTER_FREQ_OSR * FT8_ADAPTER_NUM_BINS];
    /* Frozen copy of mag[] taken the instant a slot completes, so the next
     * capture can start immediately while the (slow) decode reads this copy.
     * See ft8_waterfall_snapshot_mag() - this was the fix for the structural
     * "capture + decode > 15 s" drift found on the GD32. */
    uint8_t mag_snapshot[FT8_ADAPTER_MAX_BLOCKS * FT8_ADAPTER_TIME_OSR * FT8_ADAPTER_FREQ_OSR * FT8_ADAPTER_NUM_BINS];
    float fft_re[FT8_FFT_SIZE];
    float fft_im[FT8_FFT_SIZE];
    float fft_twiddle_cos[FT8_FFT_SIZE / 2U];
    float fft_twiddle_sin[FT8_FFT_SIZE / 2U];
    float fft_hann[FT8_FFT_SIZE];
    uint16_t fft_bitrev[FT8_FFT_SIZE];
    uint8_t cascade[FT8_WF_HISTORY_ROWS][FT8_ADAPTER_NUM_BINS];
    int16_t window[FT8_ADAPTER_NFFT];
    float dbout[FT8_FFT_BINS_USEFUL];
} ft8_ram_t;

extern ft8_ram_t *g_ft8_ram;

/* Allocates g_ft8_ram (PSRAM, zeroed) and builds the FFT tables. Returns false
 * if the allocation failed - FT8 must then stay disabled. Safe to call again
 * (no-op once allocated). */
bool ft8_ram_init(void);

#endif /* FT8_RAM_H */
