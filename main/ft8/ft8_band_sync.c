#include "ft8_band_sync.h"

#include <stdlib.h>
#include <string.h>

/* ---- Tuning ------------------------------------------------------------- */
#define HEARD_SCORE 20           /* host corpus: pure noise tops at 10-12, real signals 30+ */
#define LOCK_ERR_S 0.060f        /* |median DT| below this counts as a good slot */
#define LOCK_GOOD_SLOTS 2        /* good decoded slots in a row to declare LOCKED */
#define UNLOCK_ERR_S 0.400f      /* |median DT| above this while LOCKED is suspicious... */
#define UNLOCK_BAD_SLOTS 2       /* ...this many decoded slots in a row drops back to SYNCING */
#define DEADBAND_S 0.020f        /* no trim below this (per-decode noise is ~32 ms stdev) */
#define LOCKED_GAIN 0.5f         /* damped trims once locked */
#define LOCKED_MAX_MS 150        /* per-slot trim clamp once locked */
#define SYNCING_MAX_MS 3000      /* per-slot clamp while syncing (covers the decoder's search range) */
#define SWEEP_STEP_MS 4000       /* blind sweep step: 4 steps cover the 15 s circle */
#define SWEEP_SLOTS_BLIND 2      /* captured slots per step when the clock was never set */
#define SWEEP_SLOTS_SET 4        /* ...when it was set, only while signals are HEARD but undecodable */

/* Clock set by eye but badly off: probe around the value that was set, near
 * offsets first. The decoder tolerates roughly -2.1..+2.7 s of clock error,
 * so 4 s steps leave no gaps; +-8 s reaches the whole 15 s circle. */
static const int32_t k_set_sweep_ms[] = {0, 4000, -4000, 8000, -8000};
#define SET_SWEEP_N ((int)(sizeof(k_set_sweep_ms) / sizeof(k_set_sweep_ms[0])))

static ft8_bsync_state_t s_state;
static bool s_clock_set;
static bool s_skip_next;   /* next slot was captured on the old grid - ignore its timing */
static int s_good, s_bad;
static int s_slots_since_step;
static int s_set_sweep_idx; /* position in k_set_sweep_ms (net offset currently applied) */
static bool s_have_err;
static float s_last_err;

static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static float median(const float *v, int n)
{
    float tmp[64];
    if (n > 64)
    {
        n = 64;
    }
    memcpy(tmp, v, (size_t)n * sizeof(float));
    qsort(tmp, (size_t)n, sizeof(float), cmp_float);
    return (n & 1) ? tmp[n / 2] : 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
}

static int32_t clamp_ms(float ms, int32_t lim)
{
    int32_t v = (int32_t)(ms + (ms >= 0.0f ? 0.5f : -0.5f));
    if (v > lim)
    {
        v = lim;
    }
    if (v < -lim)
    {
        v = -lim;
    }
    return v;
}

void ft8_bsync_reset(bool clock_roughly_set)
{
    s_state = FT8_BSYNC_SEARCHING;
    s_clock_set = clock_roughly_set;
    s_skip_next = false;
    s_good = s_bad = 0;
    s_slots_since_step = 0;
    s_set_sweep_idx = 0;
    s_have_err = false;
}

void ft8_bsync_capture_restarted(void)
{
    s_skip_next = false;
}

void ft8_bsync_external_set(void)
{
    s_clock_set = true;
    s_skip_next = false;
    if (s_state == FT8_BSYNC_LOCKED)
    {
        s_state = FT8_BSYNC_SYNCING;
    }
    s_good = s_bad = 0;
}

/* Returned correction: a signal measured LATE (DT > 0) means the local
 * boundary came too early, i.e. the local clock runs ahead -> move it back. */
static int32_t emit(int32_t delta_ms)
{
    if (delta_ms != 0 && abs(delta_ms) <= FT8_BSYNC_RESTART_MS)
    {
        s_skip_next = true; /* the capture already running keeps the old error */
    }
    return delta_ms;
}

int32_t ft8_bsync_slot(const ft8_bsync_input_t *in)
{
    const bool skip = s_skip_next;
    float err, gain;

    s_skip_next = false;

    if (in->n_dt <= 0)
    {
        /* No decode: a locked/syncing grid just holds (crystal drift is
         * negligible over minutes); only the acquisition states act. */
        if (s_state == FT8_BSYNC_SEARCHING || s_state == FT8_BSYNC_HEARD)
        {
            s_state = (in->top_score >= HEARD_SCORE) ? FT8_BSYNC_HEARD : FT8_BSYNC_SEARCHING;
            s_slots_since_step++;
            if (!s_clock_set && s_slots_since_step >= SWEEP_SLOTS_BLIND)
            {
                s_slots_since_step = 0;
                return emit(SWEEP_STEP_MS);
            }
            if (s_clock_set && s_state == FT8_BSYNC_HEARD && s_slots_since_step >= SWEEP_SLOTS_SET)
            {
                int next = (s_set_sweep_idx + 1) % SET_SWEEP_N;
                int32_t delta = k_set_sweep_ms[next] - k_set_sweep_ms[s_set_sweep_idx];
                s_set_sweep_idx = next;
                s_slots_since_step = 0;
                return emit(delta);
            }
            if (s_clock_set && s_state == FT8_BSYNC_SEARCHING)
            {
                s_slots_since_step = 0; /* quiet band: never sweep a clock that was set */
            }
        }
        return 0;
    }

    err = median(in->dt_s, in->n_dt);
    s_last_err = err;
    s_have_err = true;
    s_slots_since_step = 0;
    s_set_sweep_idx = 0; /* the grid now sits on real traffic: that is the new reference */

    if (skip)
    {
        /* Measured on the pre-correction grid. Still proves the band decodes. */
        if (s_state < FT8_BSYNC_SYNCING)
        {
            s_state = FT8_BSYNC_SYNCING;
        }
        return 0;
    }

    /* Fewer decodes -> noisier median -> gentler steering. */
    gain = (in->n_dt >= 3) ? 1.0f : 0.5f;

    switch (s_state)
    {
    case FT8_BSYNC_SEARCHING:
    case FT8_BSYNC_HEARD:
        /* First CRC-valid decode: jump straight onto it. */
        s_state = FT8_BSYNC_SYNCING;
        s_good = 0;
        if (err > DEADBAND_S || err < -DEADBAND_S)
        {
            return emit(clamp_ms(-err * 1000.0f, SYNCING_MAX_MS));
        }
        return 0;

    case FT8_BSYNC_SYNCING:
        if (err < LOCK_ERR_S && err > -LOCK_ERR_S)
        {
            if (++s_good >= LOCK_GOOD_SLOTS)
            {
                s_state = FT8_BSYNC_LOCKED;
                s_bad = 0;
            }
        }
        else
        {
            s_good = 0;
        }
        if (err > DEADBAND_S || err < -DEADBAND_S)
        {
            return emit(clamp_ms(-err * 1000.0f * gain, SYNCING_MAX_MS));
        }
        return 0;

    case FT8_BSYNC_LOCKED:
    default:
        if (err > UNLOCK_ERR_S || err < -UNLOCK_ERR_S)
        {
            if (++s_bad >= UNLOCK_BAD_SLOTS)
            {
                /* Consistently off: something moved (external clock change,
                 * USB stream hiccup). Re-acquire with full steering. */
                s_state = FT8_BSYNC_SYNCING;
                s_good = 0;
                return emit(clamp_ms(-err * 1000.0f, SYNCING_MAX_MS));
            }
            return 0; /* one odd slot (e.g. only badly-timed stations): ignore */
        }
        s_bad = 0;
        if (err > DEADBAND_S || err < -DEADBAND_S)
        {
            return emit(clamp_ms(-err * 1000.0f * gain * LOCKED_GAIN, LOCKED_MAX_MS));
        }
        return 0;
    }
}

ft8_bsync_state_t ft8_bsync_get_state(void)
{
    return s_state;
}

const char *ft8_bsync_state_name(ft8_bsync_state_t s)
{
    switch (s)
    {
    case FT8_BSYNC_HEARD:
        return "HEARD, NO DECODE";
    case FT8_BSYNC_SYNCING:
        return "SYNCING";
    case FT8_BSYNC_LOCKED:
        return "LOCKED";
    default:
        return "SEARCHING";
    }
}

bool ft8_bsync_get_last_error(float *err_s)
{
    *err_s = s_last_err;
    return s_have_err;
}
