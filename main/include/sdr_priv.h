
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "driver/i2s_std.h"

#include "esp_dsp.h"

#include "filtros.h"
#include "CPLX_filter.h"

#include "sdr.h"

    /*
     * Spectrum/waterfall AGC (auto-tracked dB floor+ceiling, both bounds, not just a
     * vertical offset). Ported from a sibling GD32F450 SDR project's spectrum_agc_apply()
     * (per Jorge, 2026 - same underlying idea: EMA-track the current frame's own min/max
     * dB into a window, with margins and a guaranteed minimum span, so a quiet band still
     * shows visible contrast and a strong signal doesn't clip against the ceiling).
     *
     * Replaces the previous spec_agc/spec_offset/spec_offset_old/spec_rebote scheme,
     * which only ever nudged a single vertical offset up or down a fixed step every few
     * frames when the trace drifted out of a target zone - it could reposition the trace
     * but never widen or narrow how much dB range maps across the panel height, so a
     * weak, low-contrast signal always looked flat regardless of where the offset put it.
     *
     * spec_db_min/spec_db_max bound the RAW per-bin dB value (calcula_fft()'s
     * 20*log10f_fast(...) output, before any display scaling - same uncalibrated,
     * relative-only meaning it always had; this AGC doesn't change that, only how the
     * range is chosen). calcula_fft() maps [spec_db_min, spec_db_max] onto
     * [0, WAVEFORM_HEIGHT-1] for BOTH the bar height and (via the existing uint8_t cast
     * feeding fft_color_map()) the palette index, same as pixelnew already did for
     * either purpose before this change - just via a proper two-sided window now, and
     * clamped, which also removes a latent bug: pixelnew could previously exceed 255 and
     * wrap when cast to uint8_t for the color lookup, since nothing clamped it against
     * the old offset-only scheme.
     *
     * The two starting-point constants below (SPEC_AGC_DB_FLOOR/CEIL) and the four
     * SPEC_AGC_* tuning constants are carried over verbatim from the sibling project,
     * which tuned them against real hardware feedback on ITS OWN dB scale - similar in
     * nature to this project's (both are uncalibrated, roughly-log-magnitude values, not
     * true dBFS/dBm), but not proven identical in absolute numbers on THIS hardware, so
     * treat these as a working starting point to re-check once you see it on the panel,
     * not as a calibrated value.
     */
    float spec_db_min = 0.0f;
    float spec_db_max = 90.0f; /* same starting span as the old code's usable range (WAVEFORM_HEIGHT/2-ish headroom) */
    bool spec_agc_enabled = true; /* no manual-scale UI ported (not asked for) - always on for now */

    /*
     * "SPT" - spatial (bin-to-bin) spectrum line smoothing, ported from the same
     * sibling project (spectrum_set_line_smooth()/s_line_smooth_passes, per Jorge,
     * 2026): a 3-tap (1,2,1)/4 box filter across neighboring columns' already-
     * quantized pixelnew[] heights, run `spec_smooth_passes` times back-to-back
     * (repeating the narrow kernel approximates a wider one cheaply). Purely
     * spatial and per-frame - it never touches the EXISTING temporal smoothing
     * this project already had (the 0.6/0.4 EMA on fft_mag[] in calcula_fft(),
     * which addresses frame-to-frame flicker, not bin-to-bin jaggedness - the two
     * are complementary, not a replacement for each other). Default 0 (off,
     * original bin-sharp look, matching the sibling project's own default); its
     * own comment there notes 2-3 as the sweet spot after real hardware testing,
     * though that was on ITS OWN panel/FFT size, worth re-checking here rather
     * than assuming it transfers unchanged.
     */
    uint8_t spec_smooth_passes = 0;
#define SPECTRUM_LINE_SMOOTH_MAX 5

/*
 * *** Verified only in a host-side simulation of the tracking math, NOT against real
 * hardware dB values (see the host test in the delivery for this feature) - a
 * meaningful gap, since this project's raw_db comes from 20*log10f_fast(fft_mag*32768),
 * a DIFFERENT computation than the sibling project's own (a bit-manipulation log2
 * approximation), so the two are "uncalibrated relative dB-like" in the same *way* but
 * not proven to land in the same *absolute* numeric range. FLOOR/CEIL below were widened
 * from the sibling project's original -30/120 (tried first; a host-side test with a
 * plausible quiet-floor scenario immediately clipped against -30 and pinned the tracker
 * uselessly) to a wide safety backstop that should never clip a real signal - but "should"
 * is a guess, not a measurement. Recommend logging frame_db_min/frame_db_max (the two
 * locals computed each frame in calcula_fft(), just before this AGC block runs) over
 * serial for a few seconds against a real signal, then tightening FLOOR/CEIL/MIN_SPAN_DB
 * to whatever range that shows - tighter, correctly-chosen bounds make the AGC more
 * effective (SPEC_AGC_MIN_GAP still guards against a degenerate collapsed range even
 * with wide bounds, but a backstop this wide does very little useful clamping on its
 * own day-to-day). ***
 */
#define SPEC_AGC_DB_FLOOR       (-150.0f) /* spec_db_min can't go below this */
#define SPEC_AGC_DB_CEIL         150.0f   /* spec_db_max can't go above this */
#define SPEC_AGC_MIN_GAP          10.0f  /* spec_db_max - spec_db_min never allowed below this */
#define SPEC_AGC_FLOOR_MARGIN_DB   3.0f  /* headroom below the frame's own min, so the noise floor isn't clipped at the very bottom */
#define SPEC_AGC_CEIL_MARGIN_DB    6.0f  /* headroom above the frame's own max, so a peak isn't clipped at the very top */
#define SPEC_AGC_MIN_SPAN_DB      50.0f  /* guaranteed floor-to-ceiling headroom even with no strong signal present */
#define SPEC_AGC_SMOOTH_ALPHA      0.05f /* slow EMA: the window drifts, it doesn't jump, frame to frame */

    /* --------------------------------------------------------------------------------- */

    extern bool debug;
    extern int demod_modo;
    extern bool bucle;

    /* --------------------------------------------------------------------------------- */


    /* --------------------------------------------------------------------------------- */

    float history_i[128];
    float history_q[128];

    fir_f32_t fir_i;
    fir_f32_t fir_q;

    /* --------------------------------------------------------------------------------- */

#define FIR_COEFFS_LEN 96

    float fird_delay_i[FIR_COEFFS_LEN];
    float fird_delay_q[FIR_COEFFS_LEN];
    fir_f32_t fird_i;
    fir_f32_t fird_q;

    /* --------------------------------------------------------------------------------- */

    float z_IIR0[2], z_IIR1[2], z_IIR2[2], z_IIR3[2];

    /* --------------------------------------------------------------------------------- */

    fir_f32_t firmr_i;
    float firmr_i_State[FIR_COEFFS_LEN];

    fir_f32_t firmr_q;
    float firmr_q_State[FIR_COEFFS_LEN];

    fir_f32_t firmr_p;
    float firmr_p_State[FIR_COEFFS_LEN];

    /* --------------------------------------------------------------------------------- */

    float audiotmp = 0.0f, w = 0.0f, wold = 0.0f;

    /* --------------------------------------------------------------------------------- */

    typedef struct
    {
        float lpf_prev, hpf_prev_a, hpf_prev_b;
        float i_sample_prev, q_sample_prev;
        float angle;
        float prev_pilot_sample;
        float demod_out_pilot[SAMPLE_BUFFER_SIZE];
        float demod_out_audio[SAMPLE_BUFFER_SIZE];

    } fm_variables_t;

    fm_variables_t fm_variables;

    /* --------------------------------------------------------------------------------- */

    /* WFM (broadcast FM, DEMOD_WFM): separate from fm_variables_t/fm_variables above
     * (NFM's own state) so switching between NFM and WFM never cross-contaminates the
     * other mode's discriminator phase memory. See sdr.c's DEMOD_WFM branch. */
    typedef struct
    {
        float i_sample_prev, q_sample_prev; /* discriminator phase memory, at RTL_DSP_WIDE_RATE */
        float deemph_state;                 /* 50 us de-emphasis IIR state, same rate */
    } wfm_variables_t;

    wfm_variables_t wfm_variables;

    /*
     * PSRAM, not static internal-RAM arrays: 3 * WFM_BUFFER_SIZE * 4 bytes = 48 KiB, and
     * this project's internal RAM is already tight (see rtl_source.c's ESP_ERR_NO_MEM
     * story). Allocated once in sdrTask()'s startup block, alongside fird_wfm.
     */
    float *i_sample_wide;
    float *q_sample_wide;
    float *wfm_discrim;     /* discriminator + de-emphasis output, pre-decimation */

    fir_f32_t fird_wfm;
    float fird_wfm_delay[FIR_COEFFS_LEN];

    /* --------------------------------------------------------------------------------- */

    union
    {
        uint32_t sample;
        int16_t ch[2];
    } sampleData_in[SAMPLE_BUFFER_SIZE];

    union
    {
        uint32_t sample;
        int16_t ch[2];
    } sampleData_out[SAMPLE_BUFFER_SIZE];

    /* --------------------------------------------------------------------------------- */

    float i_fft[SAMPLE_BUFFER_SIZE], q_fft[SAMPLE_BUFFER_SIZE];
    float i_sample[SAMPLE_BUFFER_SIZE], q_sample[SAMPLE_BUFFER_SIZE];
    float i_sample_out[SAMPLE_BUFFER_SIZE], q_sample_out[SAMPLE_BUFFER_SIZE];
    float demod_out[SAMPLE_BUFFER_SIZE];

    float i_sample_out_d[SAMPLE_BUFFER_SIZE / DR], q_sample_out_d[SAMPLE_BUFFER_SIZE / DR];
    float demod_out_d[SAMPLE_BUFFER_SIZE / DR];

    int16_t pixelnew[SAMPLE_BUFFER_SIZE];
    int16_t pixelold[SAMPLE_BUFFER_SIZE];

    float wind[SAMPLE_BUFFER_SIZE];
    float fft_vector[SAMPLE_BUFFER_SIZE * 2];
    float fft_mag[SAMPLE_BUFFER_SIZE];
    float fft_mag_old[SAMPLE_BUFFER_SIZE];

#ifdef __cplusplus
}
#endif