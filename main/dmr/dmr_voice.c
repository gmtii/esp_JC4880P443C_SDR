#include "dmr_voice.h"

#include <string.h>
#if defined(DMR_VOICE_TRACE) || defined(DMR_VOICE_BURST_DUMP)
#include <stdio.h>
#endif

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#if defined(CONFIG_DMR_VOICE_MBELIB) && CONFIG_DMR_VOICE_MBELIB
#define DMR_VOICE_ENABLED 1
#endif
#endif

static volatile int s_pref = DMR_VOICE_PREF_AUTO;
static volatile int s_playing = -1;

void dmr_voice_set_pref(int pref)
{
    s_pref = (pref == 0 || pref == 1) ? pref : DMR_VOICE_PREF_AUTO;
    if (s_pref != DMR_VOICE_PREF_AUTO && s_playing != s_pref)
    {
        s_playing = -1; /* re-choose on the next voice burst */
    }
}

int dmr_voice_get_pref(void)
{
    return s_pref;
}

int dmr_voice_get_playing(void)
{
    return s_playing;
}

#ifndef DMR_VOICE_ENABLED
/* ---- mbelib not enabled: metadata-only build ------------------------------ */
void dmr_voice_get_timing(uint32_t *frames, uint32_t *avg_us) { *frames = 0; *avg_us = 0; }
uint32_t dmr_voice_get_concealed(void) { return 0; }
bool dmr_voice_available(void) { return false; }
void dmr_voice_set_sink(dmr_voice_sink_t sink) { (void)sink; }
void dmr_voice_reset(void) { s_playing = -1; }
void dmr_voice_burst(int slot, const uint8_t burst[132], int voice_idx, uint32_t t_ms)
{
    (void)slot; (void)burst; (void)voice_idx; (void)t_ms;
}
void dmr_voice_call_end(int slot) { (void)slot; }
void dmr_voice_set_encrypted(int slot, bool enc) { (void)slot; (void)enc; }

#else
/* ---- mbelib voice ------------------------------------------------------------ */
#include "mbelib.h"
#include <math.h>

/*
 * DMR AMBE+2 interleave schedule, from DSD (Copyright (C) 2010 DSD Author,
 * ISC licence - see THIRD_PARTY_NOTICES.md): dibit i of a 72-bit frame goes
 * to ambe_fr[rW[i]][rX[i]] (its high bit) and ambe_fr[rY[i]][rZ[i]] (low bit).
 */
static const uint8_t rW[36] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
                               0, 1, 0, 1, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2};
static const uint8_t rX[36] = {23, 10, 22, 9, 21, 8, 20, 7, 19, 6, 18, 5, 17, 4, 16, 3, 15, 2,
                               14, 1, 13, 0, 12, 10, 11, 9, 10, 8, 9, 7, 8, 6, 7, 5, 6, 4};
static const uint8_t rY[36] = {0, 2, 0, 2, 0, 2, 0, 2, 0, 3, 0, 3, 1, 3, 1, 3, 1, 3,
                               1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3};
static const uint8_t rZ[36] = {5, 3, 4, 2, 3, 1, 2, 0, 1, 13, 0, 12, 22, 11, 21, 10, 20, 9,
                               19, 8, 18, 7, 17, 6, 16, 5, 15, 4, 14, 3, 13, 2, 12, 1, 11, 0};

#define VOICE_TIMEOUT_MS 500 /* no voice burst for this long ends the call */
#if defined(CONFIG_DMR_VOICE_UVQUALITY)
#define UV_QUALITY CONFIG_DMR_VOICE_UVQUALITY
#else
#define UV_QUALITY 3         /* mbelib unvoiced synthesis quality (DSD default) */
#endif
#ifdef ESP_PLATFORM
#include "esp_timer.h"
#define NOW_US() esp_timer_get_time()
#else
#include <time.h>
static int64_t NOW_US(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (int64_t)t.tv_sec * 1000000 + t.tv_nsec / 1000; }
#endif
static volatile uint32_t s_frames;
static volatile uint64_t s_frame_us_total;
/* mbelib float synthesis output -> ~+-1 full scale */
#define OUT_SCALE (1.0f / 4096.0f) /* host-measured: speech peaks ~0.95, rms ~0.09 */

typedef struct
{
    mbe_parms cur, prev, prev_enh;
    bool enc;
    uint32_t last_voice_ms;
    float level; /* running rms of recent error-free frames (concealment reference) */
} vslot_t;

/*
 * Burst concealment. On weak or fading signals mbelib sometimes synthesises a
 * frame whose parameters were corrupted beyond what its FEC reports: it still
 * counts a few corrected errors (errs2 1-3), but the output is a loud 20 ms
 * squawk, sometimes at full scale. Host test (real AMBE frames re-transmitted
 * at 6 dB SNR): every such burst had errs2 > 0 and several times the energy of
 * the speech around it. So: a frame WITH corrected errors whose rms jumps
 * above CONCEAL_RATIO x the recent clean level is scaled down to that level.
 * Error-free frames (and errored frames of normal energy, usually decoded
 * correctly) are left untouched; only error-free frames update the level.
 */
#define CONCEAL_RATIO 2.0f
#define CONCEAL_FLOOR 0.02f   /* never treat near-silence as a reference */
#define LEVEL_ALPHA 0.1f      /* per 20 ms frame */
static volatile uint32_t s_concealed;

/* Soft limiter: linear to 0.7, then a smooth knee toward 1.0 (no hard clip). */
static inline float soft_limit(float v)
{
    const float a = v < 0.0f ? -v : v;
    if (a <= 0.7f)
    {
        return v;
    }
    {
        const float over = (a - 0.7f) / 0.3f;
        const float y = 0.7f + 0.3f * (over / (1.0f + over)); /* -> 1.0 asymptotically */
        return v < 0.0f ? -y : y;
    }
}

static vslot_t s_v[2];
static dmr_voice_sink_t s_sink;

bool dmr_voice_available(void) { return true; }

uint32_t dmr_voice_get_concealed(void)
{
    return s_concealed;
}

void dmr_voice_get_timing(uint32_t *frames, uint32_t *avg_us)
{
    uint32_t n = s_frames;
    *frames = n;
    *avg_us = n ? (uint32_t)(s_frame_us_total / n) : 0;
}

void dmr_voice_set_sink(dmr_voice_sink_t sink)
{
    s_sink = sink;
}

static void slot_init(int slot)
{
    mbe_initMbeParms(&s_v[slot].cur, &s_v[slot].prev, &s_v[slot].prev_enh);
}

void dmr_voice_reset(void)
{
    memset(s_v, 0, sizeof(s_v));
    slot_init(0);
    slot_init(1);
    s_playing = -1;
}

void dmr_voice_set_encrypted(int slot, bool enc)
{
    s_v[slot & 1].enc = enc;
}

void dmr_voice_call_end(int slot)
{
    if (s_playing == slot)
    {
        s_playing = -1;
    }
    s_v[slot & 1].enc = false;
}

static void decode_frame(int slot, const uint8_t *dib)
{
    char ambe_fr[4][24], ambe_d[49], err_str[64];
    float pcm[160];
    int errs = 0, errs2 = 0;

    memset(ambe_fr, 0, sizeof(ambe_fr));
    for (int i = 0; i < 36; i++)
    {
        ambe_fr[rW[i]][rX[i]] = (char)((dib[i] >> 1) & 1u);
        ambe_fr[rY[i]][rZ[i]] = (char)(dib[i] & 1u);
    }
    err_str[0] = '\0';
    {
        int64_t t0 = NOW_US();
        mbe_processAmbe3600x2450Framef(pcm, &errs, &errs2, err_str, ambe_fr, ambe_d,
                                       &s_v[slot].cur, &s_v[slot].prev, &s_v[slot].prev_enh, UV_QUALITY);
        s_frame_us_total += (uint64_t)(NOW_US() - t0);
        s_frames++;
    }
#ifdef DMR_VOICE_AMBE_DUMP
    { /* host test only: raw 49-bit AMBE parameters, for bit-exact comparison */
        extern void dmr_voice_ambe_dump(const char ambe_d[49], int errs2);
        dmr_voice_ambe_dump(ambe_d, errs2);
    }
#endif
    {
        float e = 0.0f, rms;
        vslot_t *v = &s_v[slot];
        for (int k = 0; k < 160; k++)
        {
            pcm[k] *= OUT_SCALE;
            e += pcm[k] * pcm[k];
        }
        rms = sqrtf(e / 160.0f);
        if (errs2 > 0 && v->level > 0.0f && rms > CONCEAL_RATIO * v->level + CONCEAL_FLOOR)
        {
            const float g = (v->level + CONCEAL_FLOOR) / rms;
            for (int k = 0; k < 160; k++)
            {
                pcm[k] *= g;
            }
            s_concealed++;
        }
        else if (errs2 == 0)
        {
            v->level += LEVEL_ALPHA * (rms - v->level);
        }
        for (int k = 0; k < 160; k++)
        {
            pcm[k] = soft_limit(pcm[k]);
        }
    }
    if (s_sink != NULL)
    {
        s_sink(pcm, 160);
    }
}

void dmr_voice_burst(int slot, const uint8_t burst[132], int voice_idx, uint32_t t_ms)
{
    vslot_t *v = &s_v[slot & 1];
    uint8_t f2[36];
    (void)voice_idx;
#ifdef DMR_VOICE_TRACE
    printf("VTRACE %u slot %d idx %d playing %d\n", (unsigned)t_ms, slot, voice_idx, s_playing);
#endif
#ifdef DMR_VOICE_BURST_DUMP
    { /* host test only: raw voice bursts (132 dibits each) for re-transmission tests */
        static FILE *bf;
        if (!bf) bf = fopen("voice_bursts.bin", "wb");
        fwrite(burst, 1, 132, bf);
        fflush(bf);
    }
#endif

    /* drop a stalled playing slot */
    if (s_playing >= 0 && (t_ms - s_v[s_playing].last_voice_ms) > VOICE_TIMEOUT_MS)
    {
        s_playing = -1;
    }
    if (v->last_voice_ms == 0 || (t_ms - v->last_voice_ms) > VOICE_TIMEOUT_MS)
    {
        slot_init(slot); /* new call on this slot: fresh vocoder state */
    }
    v->last_voice_ms = t_ms;

    if (s_playing < 0)
    {
        if (s_pref == DMR_VOICE_PREF_AUTO || s_pref == slot)
        {
            s_playing = slot;
        }
    }
    if (s_playing != slot || v->enc)
    {
        return; /* only the slot being listened to costs CPU */
    }

    memcpy(f2, &burst[36], 18);
    memcpy(&f2[18], &burst[78], 18);
    decode_frame(slot, &burst[0]);
    decode_frame(slot, f2);
    decode_frame(slot, &burst[96]);
}
#endif
