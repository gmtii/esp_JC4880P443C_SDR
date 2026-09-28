/* rtl_dsp.c - see rtl_dsp.h */
#include "rtl_dsp.h"

#include <math.h>
#include <string.h>

#define CIC_N 4
/* CIC DC gain R^N = 625; input is (2*byte - 255) in [-255, 255] */
#define CIC_NORM (1.0f / (625.0f * 255.0f))

static double bessel_i0(double x)
{
    double sum = 1.0, term = 1.0;
    for (int k = 1; k < 60; k++) {
        const double t = x / (2.0 * k);
        term *= t * t;
        sum += term;
        if (term < 1e-14 * sum) {
            break;
        }
    }
    return sum;
}

/* Kaiser-windowed sinc low-pass, unity DC gain. fc in cycles/sample. */
static void design_fir(float *h, int taps, double fc, double beta)
{
    double tmp[RTL_DSP_FIR_TAPS];
    const double m = (taps - 1) / 2.0;
    const double i0b = bessel_i0(beta);
    double sum = 0.0;
    for (int i = 0; i < taps; i++) {
        const double x = i - m;
        const double s = (fabs(x) < 1e-9) ? 2.0 * fc : sin(2.0 * M_PI * fc * x) / (M_PI * x);
        const double r = x / m;
        const double w = bessel_i0(beta * sqrt(fmax(0.0, 1.0 - r * r))) / i0b;
        tmp[i] = s * w;
        sum += tmp[i];
    }
    for (int i = 0; i < taps; i++) {
        h[i] = (float)(tmp[i] / sum);
    }
}

void rtl_dsp_init(rtl_dsp_t *d, bool conjugate)
{
    memset(d, 0, sizeof(*d));
    d->step = 1.0f;
    d->conjugate = conjugate;
    /* Cut-off at 24 kHz / 192 kHz: passband edge ~19 kHz, stopband from ~29 kHz (~75 dB) */
    design_fir(d->coeff, RTL_DSP_FIR_TAPS, 24000.0 / (double)RTL_DSP_MID_RATE, 7.0);
}

void rtl_dsp_set_step(rtl_dsp_t *d, float step)
{
    if (step < RTL_DSP_STEP_MIN) step = RTL_DSP_STEP_MIN;
    if (step > RTL_DSP_STEP_MAX) step = RTL_DSP_STEP_MAX;
    d->step = step;
}

void rtl_dsp_set_offset(rtl_dsp_t *d, int32_t offset_hz)
{
    if (offset_hz == d->nco_offset_hz) {
        return;
    }
    if (d->nco_offset_hz == 0) {
        d->nco_c = 1.0f;   /* NCO was bypassed: start from phase 0 */
        d->nco_s = 0.0f;
        d->nco_renorm = 0;
    }
    /* multiply by e^{-j 2 pi offset t}: moves the component at +offset to 0 Hz */
    const double w = -2.0 * 3.14159265358979323846 * (double)offset_hz / (double)RTL_DSP_MID_RATE;
    d->nco_dc = (float)cos(w);
    d->nco_ds = (float)sin(w);
    d->nco_offset_hz = offset_hz;
}

void rtl_dsp_set_wide(rtl_dsp_t *d, bool wide)
{
    d->wide = wide;
}

static inline int16_t to_i16(float v)
{
    v *= 32767.0f;
    if (v > 32767.0f) return 32767;
    if (v < -32767.0f) return -32767;
    return (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

/* 4-point cubic Lagrange at fractional position mu in [0,1) between h1 and h2 */
static inline float lagrange3(const float *h, float mu)
{
    const float c0 = -mu * (mu - 1.0f) * (mu - 2.0f) * (1.0f / 6.0f);
    const float c1 = (mu + 1.0f) * (mu - 1.0f) * (mu - 2.0f) * 0.5f;
    const float c2 = -(mu + 1.0f) * mu * (mu - 2.0f) * 0.5f;
    const float c3 = (mu + 1.0f) * mu * (mu - 1.0f) * (1.0f / 6.0f);
    return c0 * h[0] + c1 * h[1] + c2 * h[2] + c3 * h[3];
}

size_t rtl_dsp_process(rtl_dsp_t *d, const uint8_t *cu8, size_t n_bytes,
                       int16_t *out, size_t max_out_frames)
{
    size_t n_out = 0;
    const size_t n_iq = n_bytes / 2;
    const float step = d->step;

    for (size_t k = 0; k < n_iq; k++) {
        /* --- CIC integrators (input rate) --- */
        uint32_t xi = (uint32_t)(2 * (int32_t)cu8[2 * k] - 255);
        uint32_t xq = (uint32_t)(2 * (int32_t)cu8[2 * k + 1] - 255);
        d->integ_i[0] += xi;
        d->integ_i[1] += d->integ_i[0];
        d->integ_i[2] += d->integ_i[1];
        d->integ_i[3] += d->integ_i[2];
        d->integ_q[0] += xq;
        d->integ_q[1] += d->integ_q[0];
        d->integ_q[2] += d->integ_q[1];
        d->integ_q[3] += d->integ_q[2];

        if (++d->cic_cnt < RTL_DSP_CIC_R) {
            continue;
        }
        d->cic_cnt = 0;

        /* --- CIC combs (192 kSps) --- */
        uint32_t ci = d->integ_i[3], cq = d->integ_q[3], t;
        for (int s = 0; s < CIC_N; s++) {
            t = ci; ci -= d->comb_i[s]; d->comb_i[s] = t;
            t = cq; cq -= d->comb_q[s]; d->comb_q[s] = t;
        }
        float fi = (float)(int32_t)ci * CIC_NORM;
        float fq = (float)(int32_t)cq * CIC_NORM;

        /* --- digital fine tuning (NCO), see rtl_dsp_set_offset() --- */
        if (d->nco_offset_hz != 0) {
            const float c = d->nco_c, s = d->nco_s;
            const float ri = fi * c - fq * s;
            const float rq = fi * s + fq * c;
            fi = ri;
            fq = rq;
            d->nco_c = c * d->nco_dc - s * d->nco_ds;
            d->nco_s = c * d->nco_ds + s * d->nco_dc;
            if (++d->nco_renorm >= 1024u) {
                /* keep |phasor| = 1 against float rounding drift */
                const float g = 1.5f - 0.5f * (d->nco_c * d->nco_c + d->nco_s * d->nco_s);
                d->nco_c *= g;
                d->nco_s *= g;
                d->nco_renorm = 0;
            }
        }

        /* --- cubic interpolator, variable delay --- */
        d->hist_i[0] = d->hist_i[1]; d->hist_i[1] = d->hist_i[2];
        d->hist_i[2] = d->hist_i[3]; d->hist_i[3] = fi;
        d->hist_q[0] = d->hist_q[1]; d->hist_q[1] = d->hist_q[2];
        d->hist_q[2] = d->hist_q[3]; d->hist_q[3] = fq;
        if (d->hist_fill < 4) {
            d->hist_fill++;
            continue;
        }

        while (d->mu < 1.0f) {
            const float yi = lagrange3(d->hist_i, d->mu);
            const float yq = lagrange3(d->hist_q, d->mu);
            d->mu += step;

            if (d->wide) {
                /* Wide (WFM) tap: emit the resampler's own 192 kSps output directly, no
                 * further decimation. Anti-aliasing here relies solely on the CIC's own
                 * roll-off across the full +-96 kHz Nyquist (no 96-tap FIR cleanup as the
                 * narrow path gets) - some droop toward the band edges is expected; this
                 * is adequate for an FM discriminator, tuned so the wanted signal sits
                 * near the centre, but would be worth revisiting for weak-signal or
                 * high-fidelity (stereo/RDS) use. */
                float acc_i = yi, acc_q = yq;
                if (d->conjugate) {
                    acc_q = -acc_q;
                }
                if (n_out < max_out_frames) {
                    out[2 * n_out] = to_i16(acc_i);
                    out[2 * n_out + 1] = to_i16(acc_q);
                    n_out++;
                } else {
                    d->out_dropped++;
                }
                continue;
            }

            /* --- FIR decimator (192 -> 48 kSps) --- */
            const uint32_t pos = d->dl_pos;
            d->dl_i[pos] = yi; d->dl_i[pos + RTL_DSP_FIR_TAPS] = yi;
            d->dl_q[pos] = yq; d->dl_q[pos + RTL_DSP_FIR_TAPS] = yq;
            d->dl_pos = (pos + 1 == RTL_DSP_FIR_TAPS) ? 0 : pos + 1;

            if (++d->fir_cnt < RTL_DSP_FIR_R) {
                continue;
            }
            d->fir_cnt = 0;

            const float *xi_p = &d->dl_i[d->dl_pos];   /* oldest sample first */
            const float *xq_p = &d->dl_q[d->dl_pos];
            float acc_i = 0.0f, acc_q = 0.0f;
            for (int j = 0; j < RTL_DSP_FIR_TAPS; j++) {   /* symmetric taps: order irrelevant */
                acc_i += d->coeff[j] * xi_p[j];
                acc_q += d->coeff[j] * xq_p[j];
            }
            if (d->conjugate) {
                acc_q = -acc_q;
            }
            if (n_out < max_out_frames) {
                out[2 * n_out] = to_i16(acc_i);
                out[2 * n_out + 1] = to_i16(acc_q);
                n_out++;
            } else {
                d->out_dropped++;
            }
        }
        d->mu -= 1.0f;
    }
    return n_out;
}

/* ------------------------------------------------------------------------- */

void rtl_rate_ctl_reset(rtl_rate_ctl_t *c)
{
    c->fill_avg = (float)RTL_RATE_TARGET_FRAMES;
    c->step = 1.0f;
}

float rtl_rate_ctl_update(rtl_rate_ctl_t *c, float level_frames, bool primed)
{
    if (!primed) {
        c->fill_avg = (float)RTL_RATE_TARGET_FRAMES;
        c->step = 1.0f;
        return c->step;
    }
    c->fill_avg += (level_frames - c->fill_avg) * RTL_RATE_AVG_ALPHA;
    float adj = RTL_RATE_KP * (c->fill_avg - (float)RTL_RATE_TARGET_FRAMES);
    if (adj > RTL_RATE_STEP_LIMIT) adj = RTL_RATE_STEP_LIMIT;
    if (adj < -RTL_RATE_STEP_LIMIT) adj = -RTL_RATE_STEP_LIMIT;
    c->step = 1.0f + adj;   /* level too high -> step > 1 -> fewer output frames */
    return c->step;
}
