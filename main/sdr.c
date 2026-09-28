
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stdbool.h"

#include "esp_dsp.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "math.h"

#include "sdr_math.h"
#include "ui.h"

#include "sdr.h"
#include "sdr_priv.h"
#include "rtl_source.h"
#include "rtl_dsp.h"
#include "ft8_app.h"
#include "dmr_app.h"
#include "ais_app.h"

/* Uncomment to log the spectrum AGC's actual frame dB range once a second - see
 * sdr_priv.h's SPEC_AGC_DB_FLOOR/CEIL comment for why this is worth running on real
 * hardware before trusting those constants. */
// #define SPEC_AGC_LOG

/*
 * Frame-to-frame (temporal) spectrum smoothing weight for the new frame - the OTHER
 * kind of smoothing, distinct from SPT's spatial (bin-to-bin, single-frame) pass
 * below. This is what actually calms a trace that visibly jumps/flickers frame to
 * frame ("nervous" in that sense) - SPT alone does very little for that, since it
 * only ever operates within one already-computed frame. Lower = slower/heavier
 * smoothing (more weight on the past), higher = snappier/more jittery. Was a fixed
 * 0.6 (fairly fast/light); starting lower here since Jorge reported the default
 * looking "nervous" and 5 passes of SPT barely changing that - not measured against
 * real hardware/signal conditions, so treat 0.25 as a first guess to retune, not a
 * calibrated value. 'f' suffix: the P4 FPU is single precision only, an unsuffixed
 * constant here would make this soft-float. */
#define SPEC_TEMPORAL_ALPHA 0.25f
#include "audio_out.h"

#include "agc.h"
#include "nr.h"
#include "sam.h"
#include "nr_ss.h"

extern int nr_mode;
extern int agc_mode;
extern unsigned int time_sdrtask;
extern bool bucle;

extern bool f_nr;
extern bool f_nrss;
extern int f_autonotch_nr;

extern int filtro_indice;
extern bool f_actualiza;

// Generate Windowed-Sinc filter coefficients
void generate_FIR_coefficients(float *fir_coeffs, const unsigned int fir_len, const float ft)
{

    // Even or odd length of the FIR filter
    const bool is_odd = (fir_len % 2) ? (true) : (false);
    const float fir_order = (float)(fir_len - 1);

    // Window coefficients
    float *fir_window = (float *)malloc(fir_len * sizeof(float));
    dsps_wind_blackman_f32(fir_window, fir_len);

    for (int i = 0; i < fir_len; i++)
    {
        if ((i == fir_order / 2) && (is_odd))
        {
            fir_coeffs[i] = 2 * ft;
        }
        else
        {
            fir_coeffs[i] = sinf((2 * M_PI * ft * (i - fir_order / 2))) / (M_PI * (i - fir_order / 2));
        }

        fir_coeffs[i] *= fir_window[i];
    }

    free(fir_window);
}

float IRAM_ATTR alpha_beta_mag(float inphase, float quadrature)
// (c) András Retzler
// taken from libcsdr: https://github.com/simonyiszk/csdr
{
    // Min RMS Err      0.947543636291 0.392485425092
    // Min Peak Err     0.960433870103 0.397824734759
    // Min RMS w/ Avg=0 0.948059448969 0.392699081699
    const float alpha = 0.960433870103; // 1.0; //0.947543636291;
    const float beta = 0.397824734759;
    /* magnitude ~= alpha * max(|I|, |Q|) + beta * min(|I|, |Q|) */
    float abs_inphase = fabs(inphase);
    float abs_quadrature = fabs(quadrature);
    if (abs_inphase > abs_quadrature)
    {
        return alpha * abs_inphase + beta * abs_quadrature;
    }
    else
    {
        return alpha * abs_quadrature + beta * abs_inphase;
    }
}

/*
 * FFT table readiness. calcula_fft() is called from an LVGL timer (the
 * spectrum/waterfall redraw), which starts as soon as init_ui() releases the
 * display - possibly BEFORE sdrTask has run its own dsps_fft2r_init_fc32().
 * With the FT8/DMR/AIS panels init_ui() takes long enough for the timer to
 * win that race, and the FFT then ran with a NULL twiddle table (load access
 * fault in dsps_fft2r_fc32_arp4). app_main() now calls sdr_fft_init() before
 * init_ui(), and calcula_fft() does nothing until the table exists.
 */
static volatile bool s_fft_ready = false;

esp_err_t sdr_fft_init(void)
{
    /* esp-dsp keeps one global table and returns early if it already exists,
     * so the later call in sdrTask (and NR_SS_init's) are harmless */
    esp_err_t err = dsps_fft2r_init_fc32(NULL, SAMPLE_BUFFER_SIZE);
    if (err == ESP_OK)
    {
        s_fft_ready = true;
    }
    return err;
}

void IRAM_ATTR sdrTask(void *args)
{

    esp_err_t ret = ESP_OK;

    sam_variables_init();
    sdr_fft_init(); /* normally already done by app_main(); harmless if so */
    NR_SS_init();

    // Filtro biquad LPF 48000 x 0.15 para modos AM

    float coeffs_am[5];
    float w_lpf_i[5] = {0, 0};
    float w_lpf_q[5] = {0, 0};

    dsps_biquad_gen_lpf_f32(coeffs_am, 0.2, 1); // Q=3

    float w_hpf[5] = {0, 0};
    float w_lpf[5] = {0, 0};
    float w_notch[5] = {0, 0};

    float coeffs_hpf[5];
    float coeffs_lpf[5];

    // Filtros FIR de I para SSB
    dsps_fir_init_f32(&fir_i, CF_Re, history_i, 128);

    // Filtros FIR de Q para SSB
    dsps_fir_init_f32(&fir_q, CF_Im, history_q, 128);

    /* FIR MR para decimar/interpolar */

    float fird_coeffs[FIR_COEFFS_LEN];

    // dsps_firmr_init_f32(&firmr_i, (float *)fird_coeffs, firmr_i_State, FIR_COEFFS_LEN, 1, 4, 4);
    // dsps_firmr_init_f32(&firmr_q, (float *)fird_coeffs, firmr_q_State, FIR_COEFFS_LEN, 1, 4, 4);
    // dsps_firmr_init_f32(&firmr_p, (float *)fird_coeffs, firmr_p_State, FIR_COEFFS_LEN, 4, 1, 0);

    // FIRD

    /* Generador FIR */

    dsps_fird_init_f32(&fird_i, fird_coeffs, fird_delay_i, FIR_COEFFS_LEN, DR);
    dsps_fird_init_f32(&fird_q, fird_coeffs, fird_delay_q, FIR_COEFFS_LEN, DR);

    generate_FIR_coefficients(fird_coeffs, FIR_COEFFS_LEN, 0.75 / DR);

    /* See the field comment in sdr_priv.h: PSRAM, not static internal-RAM arrays. */
    i_sample_wide = heap_caps_malloc(WFM_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    q_sample_wide = heap_caps_malloc(WFM_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    wfm_discrim = heap_caps_malloc(WFM_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!i_sample_wide || !q_sample_wide || !wfm_discrim)
    {
        ESP_LOGE("sdr", "WFM buffer allocation failed (PSRAM); WFM mode will not be usable");
    }

    /* WFM (broadcast FM) decimator: 192 -> 48 kSps, cutoff at the mono audio bandwidth
     * (~15 kHz). NOT the same 0.75/DR convention fird_i/fird_q use above - that value is
     * normalized to THEIR 48 kHz input, and reused verbatim here (192 kHz input) would
     * put the cutoff at 36 kHz, decimating with almost no anti-alias protection at all. */
    float fird_wfm_coeffs[FIR_COEFFS_LEN];
    dsps_fird_init_f32(&fird_wfm, fird_wfm_coeffs, fird_wfm_delay, FIR_COEFFS_LEN, DR);
    generate_FIR_coefficients(fird_wfm_coeffs, FIR_COEFFS_LEN, 15000.0f / (float)RTL_DSP_WIDE_RATE);

    /* 50 us de-emphasis (EU broadcast standard), one-pole IIR at RTL_DSP_WIDE_RATE */
    const float wfm_dt = 1.0f / (float)RTL_DSP_WIDE_RATE;
    const float wfm_tau = 50e-6f;
    const float wfm_deemph_alpha = wfm_dt / (wfm_tau + wfm_dt);

    int i = 0;

    while (1)
    {
        /* WFM needs the wide 192 kSps I/Q tap (see sdr.h's WFM_BUFFER_SIZE comment);
         * every other mode keeps the normal 48 kSps one. Safe to call every iteration:
         * rtl_source_set_wide() only acts (and briefly re-primes) when the mode actually
         * changes, so this costs nothing the rest of the time. */
        const bool wfm = (demod_modo == DEMOD_WFM);
        rtl_source_set_wide(wfm);

        /* The codec's I2S TX write below paces this loop; rtl_source absorbs the
         * dongle-vs-codec clock difference either way. */
        if (wfm)
        {
            ret = rtl_source_read_float(i_sample_wide, q_sample_wide, WFM_BUFFER_SIZE, 100);
        }
        else
        {
            ret = rtl_source_read_float(i_sample, q_sample, SAMPLE_BUFFER_SIZE, 100);
        }
        if (ret != ESP_OK)
        {
            /* No dongle or stalled stream: keep the codec fed with silence */
            memset(sampleData_out, 0, sizeof(sampleData_out));
            audio_out_write((const int16_t *)&sampleData_out[0].sample, SAMPLE_BUFFER_SIZE);
            continue;
        }

        unsigned int start_sdrtask = dsp_get_cpu_cycle_count();

        /* Vectores para FFT: in WFM, fed from the wide buffer so the spectrum/waterfall
         * show the whole ~192 kHz channel (a 1024-point FFT of 192 kHz-rate samples
         * spans it exactly); every other mode keeps showing the normal 48 kHz span. */
        if (wfm)
        {
            memcpy(i_fft, i_sample_wide, sizeof(i_fft));
            memcpy(q_fft, q_sample_wide, sizeof(q_fft));
        }
        else
        {
            memcpy(i_fft, i_sample, sizeof(i_fft));
            memcpy(q_fft, q_sample, sizeof(q_fft));
        }

        if (wfm)
        {
            /* Broadcast FM: discriminate at the full RTL_DSP_WIDE_RATE I/Q rate (needed
             * for the ~180 kHz Carson bandwidth of a 75 kHz-deviation signal), then a
             * 50 us de-emphasis and a real decimating FIR (fird_wfm, ~15 kHz cutoff -
             * correctly normalized to this wide input rate, unlike the 0.75/DR narrow-
             * band convention below) bring it down to the 48 kSps SAMPLE_BUFFER_SIZE
             * block every other mode already produces each loop iteration.
             *
             * The discriminator's own formula, scaling and phase-memory handling exactly
             * mirror the NFM branch below - only the sample rate and the deviation-to-
             * amplitude normalization (WFM's much larger deviation) differ. NFM's own
             * state (fm_variables) is untouched, so switching between NFM and WFM never
             * cross-contaminates the other mode's discriminator phase memory. */
            float angle, x, y;

            for (i = 0; i < WFM_BUFFER_SIZE; i++)
            {
                y = (q_sample_wide[i] * wfm_variables.i_sample_prev) - (i_sample_wide[i] * wfm_variables.q_sample_prev);
                x = (i_sample_wide[i] * wfm_variables.i_sample_prev) + (q_sample_wide[i] * wfm_variables.q_sample_prev);

                angle = ApproxAtan2(y, x);

                if (isnanf(angle))
                {
                    angle = 0.0f;
                }

                /* rad/sample -> Hz (f_dev = angle * fs / (2*pi)), normalized so that
                 * +-WFM_MAX_DEVIATION_HZ maps to roughly +-1.0, the same full-scale
                 * convention every other demod_out value already uses. */
                float dev_hz = angle * (SDR_INV_PI_F * 0.5f * (float)RTL_DSP_WIDE_RATE);

                wfm_variables.deemph_state += (dev_hz * (1.0f / WFM_MAX_DEVIATION_HZ) - wfm_variables.deemph_state) * wfm_deemph_alpha;
                wfm_discrim[i] = wfm_variables.deemph_state;

                wfm_variables.q_sample_prev = q_sample_wide[i];
                wfm_variables.i_sample_prev = i_sample_wide[i];
            }

            dsps_fird_f32_ansi(&fird_wfm, wfm_discrim, demod_out, SAMPLE_BUFFER_SIZE);

            /* AIS mode (tuned to 162.000 MHz on this wide path): hand the raw
             * 192 kSps I/Q to the AIS task (copy only, see ais/ais_app.c) and
             * mute the broadcast-FM audio, which would only be hiss here. */
            if (ais_app_is_active())
            {
                ais_app_feed_iq(i_sample_wide, q_sample_wide, WFM_BUFFER_SIZE);
                memset(demod_out, 0, SAMPLE_BUFFER_SIZE * sizeof(float));
            }

            for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) // convierte a int16
            {
                sampleData_out[i].ch[0] = (int16_t)(demod_out[i] * (float)INT16_MAX);
                sampleData_out[i].ch[1] = sampleData_out[i].ch[0];
            }
        }
        else if (demod_modo != DEMOD_FM)
        {
            // Ya estamos en CODEC_SAMPLERATE
            // Hago una conversion de frecuencia a SR/4
            // p.e. 192khz serán 48khz, por lo tanto 5.450 pasa a ser 5.402. Sintonizamos por abajo
            // pero presentamos la frecuencia con esa suma de SR/4.
            for (i = 0; i < SAMPLE_BUFFER_SIZE; i += 4)
            { // i_sample_d contains I = real values
                // i_sample_d contains Q = imaginary values
                // xnew(0) =  xreal(0) + jximag(0)
                // leave as it is!
                // xnew(1) =  - ximag(1) + jxreal(1)
                float hh1 = -q_sample[i + 1];
                float hh2 = i_sample[i + 1];
                i_sample[i + 1] = hh1;
                q_sample[i + 1] = hh2;
                // xnew(2) = -xreal(2) - jximag(2)
                hh1 = -i_sample[i + 2];
                hh2 = -q_sample[i + 2];
                i_sample[i + 2] = hh1;
                q_sample[i + 2] = hh2;
                // xnew(3) = + ximag(3) - jxreal(3)
                hh1 = q_sample[i + 3];
                hh2 = -i_sample[i + 3];
                i_sample[i + 3] = hh1;
                q_sample[i + 3] = hh2;
            }

            // dsps_firmr_f32(&firmr_i, i_sample, i_sample_out, SAMPLE_BUFFER_SIZE);
            // dsps_firmr_f32(&firmr_q, q_sample, q_sample_out, SAMPLE_BUFFER_SIZE);

            dsps_fird_f32_ansi(&fird_i, i_sample, i_sample_out_d, SAMPLE_BUFFER_SIZE / DR);
            dsps_fird_f32_ansi(&fird_q, q_sample, q_sample_out_d, SAMPLE_BUFFER_SIZE / DR);

            if (demod_modo == DEMOD_USB || demod_modo == DEMOD_LSB) // En AM/SAM/FM no aplicamos desfase a Q
            {
                dsps_fir_f32(&fir_i, i_sample_out_d, i_sample_out_d, SAMPLE_BUFFER_SIZE / DR);
                dsps_fir_f32(&fir_q, q_sample_out_d, q_sample_out_d, SAMPLE_BUFFER_SIZE / DR);

                if (demod_modo == DEMOD_LSB)
                {
                    dsps_add_f32(i_sample_out_d, q_sample_out_d, demod_out_d, SAMPLE_BUFFER_SIZE / DR, 1, 1, 1); // Demodula USB
                }
                else if (demod_modo == DEMOD_USB)
                {
                    dsps_sub_f32(i_sample_out_d, q_sample_out_d, demod_out_d, SAMPLE_BUFFER_SIZE / DR, 1, 1, 1); // Demodula LSB

                    /* FT8 tap: raw 12 kHz USB audio, BEFORE the passband
                     * biquads, NR and RxAGC below - the same point the
                     * DeepSDR 101 taps (s_ssb_dec). FT8 has its own AGC and
                     * needs the full 0-1600 Hz window regardless of the
                     * filter chosen for listening. Only copies into a
                     * stream buffer (no-op unless FT8 mode is on); all the
                     * FT8 DSP runs in its own task - see ft8/ft8_app.c. */
                    ft8_app_feed_audio(demod_out_d, SAMPLE_BUFFER_SIZE / DR);
                }
            }
            else if (demod_modo >= DEMOD_SAM && demod_modo <= DEMOD_SAMU)
                SAM(i_sample_out_d, q_sample_out_d, demod_out_d, SAMPLE_BUFFER_SIZE / DR, demod_modo);
            else if (demod_modo == DEMOD_AM)
            {
                for (i = 0; i < SAMPLE_BUFFER_SIZE / DR; i++)
                {
                    audiotmp = alpha_beta_mag(i_sample_out_d[i], q_sample_out_d[i]);
                    w = audiotmp + wold * 0.9999f; // yes, I want a superb bass response ;-)
                    demod_out_d[i] = w - wold;
                    wold = w;
                }
            }
        }
        else
        {
            float angle, x, y;

            /* DMR tap: the raw 48 kS/s IQ, before this discriminator. The DMR
             * task applies its own 12.5 kHz channel filter and discriminator
             * (this NFM path has no channel filter at all). Only copies into a
             * stream buffer; no-op unless DMR mode is on - see dmr/dmr_app.c. */
            dmr_app_feed_iq(i_sample, q_sample, SAMPLE_BUFFER_SIZE);

            for (i = 0; i < SAMPLE_BUFFER_SIZE; i++)
            {
                y = (q_sample[i] * fm_variables.i_sample_prev) - (i_sample[i] * fm_variables.q_sample_prev);
                x = (i_sample[i] * fm_variables.i_sample_prev) + (q_sample[i] * fm_variables.q_sample_prev);

                angle = ApproxAtan2(y, x);

                if (isnanf(angle))
                {
                    angle = 0.0f;
                }

                demod_out[i] = angle * (SDR_INV_PI_F * 0.1f); /* was (angle / M_PI) * 0.1f: a soft-float double division per sample */

                fm_variables.q_sample_prev = q_sample[i]; // save "previous" value of each channel to allow detection of the change of angle in next go-around
                fm_variables.i_sample_prev = i_sample[i];
            }

            /* DMR mode: replace the 4FSK hiss with the decoded voice (or
             * silence, when there is none or mbelib is not compiled in). */
            if (dmr_app_is_active())
            {
                dmr_app_read_audio(demod_out, SAMPLE_BUFFER_SIZE);
            }
        }

        /* Pongo entrada en salida */

        if (!wfm)
        {

            if (demod_modo != DEMOD_FM)
            {

                if (f_actualiza)
                {
                    // int f_alta, f_baja;

                    f_actualiza = false;

                    switch (filtro_indice)
                    {
                    case F_CW:
                        currentVFO.f_alta = 500;
                        currentVFO.f_baja = 100;
                        break;

                    case F_1K8:
                        currentVFO.f_alta = 1800;
                        currentVFO.f_baja = 100;
                        break;

                    case F_2K3:
                        currentVFO.f_alta = 2300;
                        currentVFO.f_baja = 100;
                        break;

                    case F_3K6:
                        currentVFO.f_alta = 2700;
                        currentVFO.f_baja = 100;
                        break;

                    case F_VAR:
                        currentVFO.f_alta = 3600;
                        currentVFO.f_baja = 100;
                        break;

                    default:
                        currentVFO.f_alta = 2700;
                        currentVFO.f_baja = 100;
                        break;
                    }

                    // Filtro variable

                    dsps_biquad_gen_hpf_f32(coeffs_hpf, (float)currentVFO.f_baja / (SAMPLE_RATE / DR), 1);
                    dsps_biquad_gen_lpf_f32(coeffs_lpf, (float)currentVFO.f_alta / (SAMPLE_RATE / DR), 1);
                }

                dsps_biquad_f32(demod_out_d, demod_out_d, SAMPLE_BUFFER_SIZE / DR, coeffs_lpf, w_lpf);
                dsps_biquad_f32(demod_out_d, demod_out_d, SAMPLE_BUFFER_SIZE / DR, coeffs_hpf, w_hpf);

                if (f_nr && !f_nrss)
                    NR(demod_out_d, SAMPLE_BUFFER_SIZE / DR, f_autonotch_nr);

                if (f_nrss && !f_nr)
                    NR_SS(demod_out_d, SAMPLE_BUFFER_SIZE / DR);

                RxAGC(demod_out_d, SAMPLE_BUFFER_SIZE / DR);

                // dsps_firmr_f32(&firmr_p, demod_out_d, demod_out, SAMPLE_BUFFER_SIZE / DR);

                // Up sampLing (Interpolation)
                for (i = SAMPLE_BUFFER_SIZE / DR - 1; i >= 0; i--)
                {
                    demod_out[i * DR] = DR * demod_out_d[i];
                    for (int j = 1; j < DR; j++)
                    {
                        demod_out[i * DR + j] = 0;
                    }
                }

                // Anti-aliasing Filter
                dsps_biquad_f32(demod_out, demod_out, SAMPLE_BUFFER_SIZE, RX_biquad0, z_IIR0);
                dsps_biquad_f32(demod_out, demod_out, SAMPLE_BUFFER_SIZE, RX_biquad1, z_IIR1);
                dsps_biquad_f32(demod_out, demod_out, SAMPLE_BUFFER_SIZE, RX_biquad2, z_IIR2);
                dsps_biquad_f32(demod_out, demod_out, SAMPLE_BUFFER_SIZE, RX_biquad3, z_IIR3);
            }

            for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) // convierte a int16
            {
                sampleData_out[i].ch[0] = (int16_t)(demod_out[i] * (float)INT16_MAX);
                sampleData_out[i].ch[1] = sampleData_out[i].ch[0]; // segundo canal para el SFM
            }
        }
        /* wfm: sampleData_out was already built above, in the WFM branch itself - this
         * block used to be an unconditional `if (1) {...}` with an unreachable `else`
         * (dead code, from before WFM existed); switching it to `if (!wfm)` gave that
         * `else` a way to run for real, clobbering WFM's audio with a raw copy of the
         * input samples. Removed: there is nothing left for WFM to do here. */

        time_sdrtask = dsp_get_cpu_cycle_count() - start_sdrtask;

        if (debug)
        {
            // Serial.printf("SDR Task cycles: %u\n", dsp_get_cpu_cycle_count() - start_sdrtask);
        }

        // Envia el DAC SAMPLE_BUFFER_SIZE * 4 ( 2 canales, 16 bit cada uno)
        ret = audio_out_write((const int16_t *)&sampleData_out[0].sample, SAMPLE_BUFFER_SIZE);
    }

    // vTaskDelete(NULL);
}

void shift_right_circular(int16_t *v, size_t size, int offset)
{
    if (size == 0 || offset == 0)
        return;
    offset %= size;
    int16_t tmp[offset];
    memcpy(tmp, &v[size - offset], offset * sizeof(int16_t));
    memmove(&v[offset], v, (size - offset) * sizeof(int16_t));
    memcpy(v, tmp, offset * sizeof(int16_t));
}

void IRAM_ATTR calcula_fft(void)
{
    if (!s_fft_ready)
    {
        return; /* twiddle table not built yet - see sdr_fft_init() */
    }
    int N = SAMPLE_BUFFER_SIZE;

    // save old pixels for lowpass filter
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        pixelold[i] = pixelnew[i];
    }

    // Hann window: constant, so compute it only once (1024 cosf per frame otherwise)
    static bool wind_ready = false;
    if (!wind_ready)
    {
        dsps_wind_hann_f32(wind, N);
        wind_ready = true;
    }

    // Convert two input vectors to one complex vector i,q
    for (int i = 0; i < N; i++)
    {
        fft_vector[i * 2 + 0] = i_fft[i] * wind[i];
        fft_vector[i * 2 + 1] = q_fft[i] * wind[i];
    }

    // FFT
    dsps_fft2r_fc32_arp4(fft_vector, N);
    //  Bit reverse
    dsps_bit_rev_fc32(fft_vector, N);

    // calculate mag = I*I + Q*Q,
    // and simultaneously put them into the right order
    for (int i = 0; i < N / 2; i++)
    {
        fft_mag[i + N / 2] = (fft_vector[i * 2] * fft_vector[i * 2] + fft_vector[i * 2 + 1] * fft_vector[i * 2 + 1]);
        fft_mag[i + 0] = (fft_vector[(i + N / 2) * 2] * fft_vector[(i + N / 2) * 2] + fft_vector[(i + N / 2) * 2 + 1] * fft_vector[(i + N / 2) * 2 + 1]);
    }

    /* Raw per-bin dB (unscaled, uncalibrated - see calcula_fft()'s callers/sdr_priv.h's
     * comment) and this frame's own min/max, tracked for the AGC below. */
    float frame_db_min = 1e9f;
    float frame_db_max = -1e9f;
    /* PSRAM, not a static internal-RAM array (was, until Jorge hit "Not enough memory
     * for LVGL buffer" again after several turns of UI additions - this 4 KiB, plus
     * smooth_tmp's 2 KiB below, was part of what pushed internal RAM back over the
     * edge, the same failure mode as the WFM buffers' original ESP_ERR_NO_MEM). Lazy
     * one-time allocation, matching fft_color_map()'s palette_lut_init() pattern. */
    static float *raw_db = NULL;
    if (raw_db == NULL)
    {
        raw_db = heap_caps_malloc(SAMPLE_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
        if (raw_db == NULL)
        {
            ESP_LOGE("sdr", "raw_db PSRAM allocation failed; spectrum/waterfall AGC will not run this boot");
            return;
        }
    }

    for (int i = 0; i < N; i++)
    {
        fft_mag[i] = SPEC_TEMPORAL_ALPHA * fft_mag[i] + (1.0f - SPEC_TEMPORAL_ALPHA) * fft_mag_old[i];
        fft_mag_old[i] = fft_mag[i];
        raw_db[N - 1 - i] = 20 * log10f_fast(fft_mag[i] * (float)(32768.0f));
    }
    for (int i = 0; i < N; i++)
    {
        if (raw_db[i] < frame_db_min) frame_db_min = raw_db[i];
        if (raw_db[i] > frame_db_max) frame_db_max = raw_db[i];
    }

#ifdef SPEC_AGC_LOG
    /* See the // #define SPEC_AGC_LOG near the top of this file. */
    {
        static int log_count = 0;
        if (++log_count >= 50) /* ~1 s at the UI timer's ~50 Hz */
        {
            log_count = 0;
            ESP_LOGI("sagc", "frame_db [%.1f .. %.1f]  window [%.1f .. %.1f]",
                     frame_db_min, frame_db_max, spec_db_min, spec_db_max);
        }
    }
#endif

    if (spec_agc_enabled)
    {
        /* See sdr_priv.h's comment on spec_db_min/spec_db_max for the full design and
         * the ported-from-a-sibling-project provenance of these constants. */
        float target_min = frame_db_min - SPEC_AGC_FLOOR_MARGIN_DB;
        float target_max = target_min + SPEC_AGC_MIN_SPAN_DB;
        if (frame_db_max + SPEC_AGC_CEIL_MARGIN_DB > target_max)
        {
            target_max = frame_db_max + SPEC_AGC_CEIL_MARGIN_DB;
        }
        if (target_min < SPEC_AGC_DB_FLOOR) target_min = SPEC_AGC_DB_FLOOR;
        if (target_max > SPEC_AGC_DB_CEIL) target_max = SPEC_AGC_DB_CEIL;
        if (target_max < target_min + SPEC_AGC_MIN_GAP) target_max = target_min + SPEC_AGC_MIN_GAP;

        spec_db_min += (target_min - spec_db_min) * SPEC_AGC_SMOOTH_ALPHA;
        spec_db_max += (target_max - spec_db_max) * SPEC_AGC_SMOOTH_ALPHA;
    }

    /* Map [spec_db_min, spec_db_max] -> [0, WAVEFORM_HEIGHT-1], clamped: this is both
     * the bar height (spectrum()/waterfall_update() use pixelnew directly as a pixel
     * height) and, via the existing (uint8_t)abs(pixelnew[x]) cast, the palette index -
     * same dual role pixelnew always had, just properly windowed and clamped now
     * (previously nothing stopped it exceeding 255 and wrapping in that cast). */
    const float scale = (float)(WAVEFORM_HEIGHT - 1) / (spec_db_max - spec_db_min);
    for (int i = 0; i < N; i++)
    {
        float v = (raw_db[i] - spec_db_min) * scale;
        if (v < 0.0f) v = 0.0f;
        if (v > (float)(WAVEFORM_HEIGHT - 1)) v = (float)(WAVEFORM_HEIGHT - 1);
        pixelnew[i] = (int16_t)v;
    }

    /* "SPT": spatial (bin-to-bin) smoothing - see sdr_priv.h's spec_smooth_passes
     * comment for the full design/provenance. Purely visual, redone from scratch
     * every frame on top of pixelnew (post-AGC, post-quantization); it never
     * touches fft_mag_old, the temporal EMA state above, which is a separate,
     * complementary smoothing (frame-to-frame, not bin-to-bin). */
    if (spec_smooth_passes > 0 && N >= 3)
    {
        /* PSRAM, not static internal RAM - see raw_db's comment above for why. */
        static int16_t *smooth_tmp = NULL;
        if (smooth_tmp == NULL)
        {
            smooth_tmp = heap_caps_malloc(SAMPLE_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
        }
        if (smooth_tmp == NULL)
        {
            ESP_LOGE("sdr", "smooth_tmp PSRAM allocation failed; skipping SPT this frame");
        }
        else
        for (uint8_t pass = 0; pass < spec_smooth_passes; pass++)
        {
            smooth_tmp[0] = pixelnew[0];
            for (int i = 1; i < N - 1; i++)
            {
                smooth_tmp[i] = (int16_t)(((int32_t)pixelnew[i - 1] + 2 * (int32_t)pixelnew[i] + (int32_t)pixelnew[i + 1]) / 4);
            }
            smooth_tmp[N - 1] = pixelnew[N - 1];
            for (int i = 0; i < N; i++)
            {
                pixelnew[i] = smooth_tmp[i];
            }
        }
    }

    // Rota 128 a la derecha para corregir el problema con el CANVAS dichoso de LGVL

    // shift_right_circular(pixelnew, N, 128);
}
