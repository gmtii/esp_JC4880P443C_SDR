
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "esp_err.h"

#define SAMPLE_BUFFER_SIZE (1024)

#define DR 4 // decimation factor
#define SAMPLE_RATE (48000)
#define FREQ_CONV_OFFSET (SAMPLE_RATE / DR)

#define WAVEFORM_WIDTH SAMPLE_BUFFER_SIZE

/*
 * One sdrTask() iteration's worth of wide-mode I/Q: chosen so it spans exactly
 * the same wall-clock time as SAMPLE_BUFFER_SIZE frames at the normal 48 kSps
 * (1024/48000 == 4096/192000 == 21.3 ms), so the loop's cadence - and the
 * audio_out_write(SAMPLE_BUFFER_SIZE) call at the end of it - is unaffected by
 * which mode is active. Must track RTL_DSP_WIDE_RATE/RTL_DSP_OUT_RATE (4) in
 * rtl_dsp.h; a comment there points back here.
 */
#define WFM_BUFFER_SIZE (SAMPLE_BUFFER_SIZE * 4)

/* Nominal broadcast FM peak deviation, used to scale the discriminator's Hz output
 * to the same +-1.0-ish full-scale convention every other demod_out value uses. */
#define WFM_MAX_DEVIATION_HZ 75000.0f

#define WAVEFORM_HEIGHT 192
#define WATERFALL_HEIGHT 128

#define DEMOD_USB 0
#define DEMOD_LSB 1
#define DEMOD_AM 2
#define DEMOD_SAM 3
#define DEMOD_SAML 4
#define DEMOD_SAMU 5
#define DEMOD_FM 6
/*
 * Broadcast FM (WFM): unlike every other mode, this one reads I/Q at the wide
 * 192 kSps tap (rtl_source_set_wide()) instead of the usual 48 kSps, because a
 * 75 kHz-deviation signal needs a Carson bandwidth (~180 kHz) far beyond what
 * 48 kSps I/Q (+-24 kHz) can carry - that's also why NFM (DEMOD_FM) is capped
 * at a few kHz of deviation, not because of any deliberate design choice there.
 */
#define DEMOD_WFM 7

/*
 * LO offset for the current mode: every mode but WFM tunes FREQ_CONV_OFFSET (12 kHz)
 * below the wanted frequency (see sdr.c's fs/4 shift). WFM's ~180 kHz Carson bandwidth
 * leaves no room for that trick without pushing the station off-centre in the (already
 * full) 192 kHz window, so it tunes directly on-frequency instead.
 */
static inline uint32_t lo_offset_for_mode(int demod_modo)
{
    return (demod_modo == DEMOD_WFM) ? 0u : (uint32_t)FREQ_CONV_OFFSET;
}

#define F_CW 0
#define F_1K8 1
#define F_2K3 2
#define F_3K6 3
#define F_VAR 4

typedef struct
{
    char *VFOName;  // Nombre del VFO
    uint32_t Frec;  // Frecuencia en Hz
    int demod_modo; // Modo de demodulación
    int filtro;
    int step;
    bool AGC;
    bool FLT;
    bool SPLT;
    bool NR_SS;
    bool NR;
    int ANR;
    int f_baja; // Valor filtro Baja
    int f_alta; // Valor filtro Alta
} VFO;

extern VFO currentVFO;
extern int filtro;
extern bool f_actualiza;

void sdrTask(void *args);
void calcula_fft(void);

/* Builds the esp-dsp FFT table. Call from app_main() BEFORE init_ui(): the
 * spectrum timer may run calcula_fft() before sdrTask starts. */
esp_err_t sdr_fft_init(void);

#ifdef __cplusplus
}
#endif