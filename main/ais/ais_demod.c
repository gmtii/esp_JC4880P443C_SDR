#include "ais_demod.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif

#define DECIM 4
#define RATE (AIS_IN_RATE / DECIM) /* 48 kSps */
#define SPS 5                      /* samples per symbol at 9600 baud */
#define TAPS 64
#ifndef CUTOFF_HZ
#define CUTOFF_HZ 9000.0f
#endif
#define CH_OFFSET_HZ 25000.0f
#ifndef DC_ALPHA
#define DC_ALPHA 0.004f            /* carrier-offset tracker, per 48 kSps sample (host sweep: 0.02 lost half the
                                    * frames at 15 dB - it follows the data content; 0.0035-0.005 is the plateau) */
#endif
#define MAX_BITS (AIS_MAX_FRAME_BYTES * 8 + 8)
#define BURST_ON_RATIO 3.0f        /* power / noise floor to declare a burst (~5 dB) */
#define BURST_OFF_RATIO 1.8f
#define FAST_DC_SAMPLES (24 * SPS) /* fast acquisition over ~ the training sequence */
#ifndef FAST_DC_ALPHA
#define FAST_DC_ALPHA 0.02f /* host sweep: best of 0.02/0.04/0.08 at 10-15 dB SNR */
#endif
#define MIN_FRAME_BITS 72          /* smallest AIS message (56 bits) + FCS */

typedef struct
{
    int prev_sym;
    uint8_t reg;
    bool in_frame;
    bool flag_tail;   /* six 1s seen: the next bit must be the flag's closing 0 */
    int ones;
    int nbits;
    uint8_t bits[MAX_BITS];
} hdlc_t;

typedef struct
{
    /* NCO */
    float c, s, dc_rot, ds_rot;
    int renorm;
    /* decimating FIR (doubled ring) */
    float hi[2 * TAPS], hq[2 * TAPS];
    int hidx, dphase;
    /* discriminator + filters */
    float pi_, pq_;
    float box[SPS];
    float zi[SPS], zq[SPS]; /* last SPS filtered samples (differential detector) */
    int bidx;
    float bsum;
    float dc;
    float pwr, noise;  /* signal power (fast) and noise-floor (slow, follows minimum) */
    int fast_left;     /* samples of fast DC acquisition left after a burst onset */
    bool active;       /* power above the noise floor: a burst is on the air */
    int sample_ctr;   /* 48 kSps samples, modulo SPS: selects the sampling phase */
    hdlc_t ph[SPS];   /* one NRZI/HDLC decoder per sampling phase (see header) */
    /* duplicate suppression: the same frame is usually valid on 2-3 phases */
    uint16_t last_crc;
    uint32_t last_crc_ms;
} chan_t;

static float s_taps[TAPS];
static chan_t *s_ch; /* [2], ~13 KB: PSRAM on the P4 (ais_demod_alloc) */
static ais_frame_cb_t s_cb;
static void *s_ctx;
static uint32_t s_samples; /* input samples since reset (stream clock) */
static ais_demod_stats_t s_stats;

uint16_t ais_crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFFu;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
        {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0x8408u) : (uint16_t)(crc >> 1);
        }
    }
    return (uint16_t)~crc;
}

static void chan_init(chan_t *ch, float offset_hz)
{
    const double w = -2.0 * 3.14159265358979323846 * offset_hz / AIS_IN_RATE;
    memset(ch, 0, sizeof(*ch));
    ch->c = 1.0f;
    ch->dc_rot = (float)cos(w);
    ch->ds_rot = (float)sin(w);
    ch->pi_ = 1.0f;
}

bool ais_demod_alloc(void)
{
    if (s_ch == NULL)
    {
#ifdef ESP_PLATFORM
        s_ch = heap_caps_calloc(2, sizeof(chan_t), MALLOC_CAP_SPIRAM);
#else
        s_ch = calloc(2, sizeof(chan_t));
#endif
    }
    return s_ch != NULL;
}

void ais_demod_reset(void)
{
    if (!ais_demod_alloc())
    {
        return;
    }
    /* Channel A (161.975) is 25 kHz below the 162.000 MHz LO; with conjugated
     * I/Q it shows up at +25 kHz. */
    const float a_off = AIS_IQ_CONJUGATED ? +CH_OFFSET_HZ : -CH_OFFSET_HZ;
    chan_init(&s_ch[0], a_off);
    chan_init(&s_ch[1], -a_off);
    s_samples = 0;
}

void ais_demod_init(ais_frame_cb_t cb, void *ctx)
{
    const float pi = 3.14159265358979f;
    float sum = 0.0f;
    for (int i = 0; i < TAPS; i++)
    {
        float m = (float)i - (TAPS - 1) / 2.0f;
        float x = 2.0f * CUTOFF_HZ / AIS_IN_RATE;
        float sinc = (fabsf(m) < 1e-6f) ? x : sinf(pi * x * m) / (pi * m);
        float w = 0.42f - 0.5f * cosf(2.0f * pi * i / (TAPS - 1)) + 0.08f * cosf(4.0f * pi * i / (TAPS - 1));
        s_taps[i] = sinc * w;
        sum += s_taps[i];
    }
    for (int i = 0; i < TAPS; i++)
    {
        s_taps[i] /= sum;
    }
    s_cb = cb;
    s_ctx = ctx;
    memset(&s_stats, 0, sizeof(s_stats));
    ais_demod_reset();
}

void ais_demod_get_stats(ais_demod_stats_t *out)
{
    *out = s_stats;
}

/* ---- HDLC --------------------------------------------------------------- */

static void frame_end(chan_t *ch, hdlc_t *h, int idx)
{
    /* the flag's leading 0 and five 1s were appended before the 6th 1 */
    int n = h->nbits - 6;
    uint8_t bytes[AIS_MAX_FRAME_BYTES + 2];
    int nb;

    if (n < MIN_FRAME_BITS || (n % 8) != 0 || n / 8 > AIS_MAX_FRAME_BYTES + 2)
    {
        return;
    }
    nb = n / 8;
    memset(bytes, 0, (size_t)nb);
    for (int k = 0; k < n; k++)
    {
        bytes[k / 8] |= (uint8_t)(h->bits[k] << (k % 8)); /* HDLC: LSB first */
    }
    if (ais_crc16(bytes, nb - 2) != (uint16_t)(bytes[nb - 2] | (bytes[nb - 1] << 8)))
    {
        s_stats.crc_errors[idx]++;
        return;
    }
    {
        /* same frame already delivered from a neighbouring sampling phase? */
        const uint16_t crc = (uint16_t)(bytes[nb - 2] | (bytes[nb - 1] << 8));
        const uint32_t now = s_samples / (AIS_IN_RATE / 1000);
        if (crc == ch->last_crc && (now - ch->last_crc_ms) < 30)
        {
            return;
        }
        ch->last_crc = crc;
        ch->last_crc_ms = now;
    }
    s_stats.frames_ok[idx]++;
    if (s_cb != NULL)
    {
        ais_frame_t f;
        f.channel = idx;
        /* each byte went out LSB first, so the packed bytes ARE the message
         * bytes, read MSB first */
        memcpy(f.bytes, bytes, (size_t)(nb - 2));
        f.nbits = (nb - 2) * 8;
        f.freq_offset_hz = ch->dc;
        f.t_ms = s_samples / (AIS_IN_RATE / 1000);
        s_cb(&f, s_ctx);
    }
}

static void hdlc_bit(chan_t *ch, hdlc_t *h, int idx, int bit)
{
    h->reg = (uint8_t)((h->reg >> 1) | (bit << 7));

    if (h->flag_tail)
    {
        /* Six 1s just closed a frame. The flag's final 0 must NOT become the
         * first data bit of the next frame (a first version did exactly that
         * whenever noise had opened a false frame before the real flag, so
         * every real frame came out shifted by one bit and failed its CRC). */
        h->flag_tail = false;
        h->nbits = 0;
        h->ones = 0;
        h->in_frame = (bit == 0); /* 0: a complete flag, it opens the next frame; 1: abort */
        return;
    }
    if (!h->in_frame)
    {
        if (h->reg == 0x7E)
        {
            h->in_frame = true;
            h->nbits = 0;
            h->ones = 0;
        }
        return;
    }
    if (bit)
    {
        if (++h->ones == 6)
        {
            frame_end(ch, h, idx); /* flag (or abort): end of frame */
            h->flag_tail = true;
            return;
        }
    }
    else
    {
        if (h->ones == 5)
        {
            h->ones = 0; /* stuffed zero: drop it */
            return;
        }
        h->ones = 0;
    }
    if (h->nbits < MAX_BITS)
    {
        h->bits[h->nbits++] = (uint8_t)bit;
    }
    else
    {
        h->in_frame = false; /* runaway: back to flag hunting */
    }
}

/* ---- Symbol decisions / NRZI ----------------------------------------------
 * No timing loop: every one of the SPS possible sampling phases runs its own
 * slicer + NRZI + HDLC decoder, and whichever phase sits near the eye centre
 * passes the CRC. A first version tracked the symbol clock with a zero-
 * crossing loop instead; it often had not converged by the end of the 24-bit
 * training sequence, and lost ~25 % of the frames even at 30 dB SNR. Over
 * one AIS frame (<= 27 ms) clock drift is negligible (50 ppm = 0.013 symbol),
 * so a fixed phase per decoder is enough. */
static void chan_sample(chan_t *ch, int idx, float fi, float fq)
{
    hdlc_t *h;
    int sym;
    float f;
#ifdef AIS_PER_SAMPLE_DISC
    /* per-sample FM discriminator + one-symbol boxcar (first version) */
    float re = fi * ch->pi_ + fq * ch->pq_;
    float im = fq * ch->pi_ - fi * ch->pq_;
    f = atan2f(im, re) * (RATE / (2.0f * 3.14159265358979f));
    ch->pi_ = fi;
    ch->pq_ = fq;
    ch->bsum += f - ch->box[ch->bidx];
    ch->box[ch->bidx] = f;
    ch->bidx = (ch->bidx + 1) % SPS;
    f = ch->bsum * (1.0f / SPS);
#else
    /* One-symbol differential detection: the phase turned over the last
     * symbol, arg(z[n] * conj(z[n-SPS])) - +-pi/2 for GMSK with h = 0.5. Same
     * quantity as summing SPS per-sample discriminator outputs, but taken in
     * one step, so noise-driven +-2pi jumps of individual samples near the
     * FM threshold can no longer corrupt it. Scaled to Hz like before. */
    {
        const int old = ch->bidx;
        const float oi = ch->zi[old], oq = ch->zq[old];
        float re = fi * oi + fq * oq;
        float im = fq * oi - fi * oq;
        ch->zi[old] = fi;
        ch->zq[old] = fq;
        ch->bidx = (ch->bidx + 1) % SPS;
        f = atan2f(im, re) * (RATE / (2.0f * 3.14159265358979f * SPS));
    }
#endif

    /*
     * Carrier-offset (DC) tracking, gated by a burst detector. Slow tracking
     * (DC_ALPHA) gives the best sensitivity but needs ~50 symbols to settle,
     * longer than the 24-symbol training sequence, so a ship 1-2 kHz off
     * frequency lost its first bits. Now: while idle the estimate is frozen
     * (noise would only drag it around); when the power jumps above the noise
     * floor, the first ~24 symbols are tracked fast, then slow.
     */
    {
        const float p = fi * fi + fq * fq;
        ch->pwr += 0.05f * (p - ch->pwr);
        if (ch->noise <= 0.0f || ch->pwr < ch->noise)
        {
            ch->noise = ch->pwr;                         /* follows the minimum quickly... */
        }
        else
        {
            ch->noise += 0.0002f * (ch->pwr - ch->noise); /* ...and rises only slowly */
        }
        if (!ch->active && ch->pwr > BURST_ON_RATIO * ch->noise)
        {
            ch->active = true;
            ch->fast_left = FAST_DC_SAMPLES;
        }
        else if (ch->active && ch->pwr < BURST_OFF_RATIO * ch->noise)
        {
            ch->active = false;
        }
        if (ch->active)
        {
            if (ch->fast_left > 0)
            {
                ch->fast_left--;
                ch->dc += FAST_DC_ALPHA * (f - ch->dc);
            }
            else
            {
                ch->dc += DC_ALPHA * (f - ch->dc);
            }
        }
    }

    h = &ch->ph[ch->sample_ctr];
    ch->sample_ctr = (ch->sample_ctr + 1) % SPS;
    sym = (f - ch->dc) >= 0.0f;
    /* NRZI: no transition = 1, transition = 0 */
    hdlc_bit(ch, h, idx, sym == h->prev_sym);
    h->prev_sym = sym;
}

void ais_demod_feed(const float *in_i, const float *in_q, int n)
{
    for (int k = 0; k < n; k++)
    {
        for (int c = 0; c < 2; c++)
        {
            chan_t *ch = &s_ch[c];
            float ri = in_i[k] * ch->c - in_q[k] * ch->s;
            float rq = in_i[k] * ch->s + in_q[k] * ch->c;
            float nc = ch->c * ch->dc_rot - ch->s * ch->ds_rot;
            ch->s = ch->c * ch->ds_rot + ch->s * ch->dc_rot;
            ch->c = nc;
            if (++ch->renorm >= 1024)
            {
                float g = 1.5f - 0.5f * (ch->c * ch->c + ch->s * ch->s);
                ch->c *= g;
                ch->s *= g;
                ch->renorm = 0;
            }

            ch->hi[ch->hidx] = ri;
            ch->hi[ch->hidx + TAPS] = ri;
            ch->hq[ch->hidx] = rq;
            ch->hq[ch->hidx + TAPS] = rq;
            ch->hidx = (ch->hidx + 1) % TAPS;

            if (++ch->dphase >= DECIM)
            {
                const float *wi = &ch->hi[ch->hidx], *wq = &ch->hq[ch->hidx];
                float fi = 0.0f, fq = 0.0f;
                ch->dphase = 0;
                for (int t = 0; t < TAPS; t++)
                {
                    fi += s_taps[t] * wi[t];
                    fq += s_taps[t] * wq[t];
                }
                chan_sample(ch, c, fi, fq);
            }
        }
        s_samples++;
    }
}

uint32_t ais_demod_now_ms(void)
{
    return s_samples / (AIS_IN_RATE / 1000);
}
