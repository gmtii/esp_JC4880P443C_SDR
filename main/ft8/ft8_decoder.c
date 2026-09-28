#include "ft8_decoder.h"
#include "ft8_waterfall_adapter.h"
#include "ft8/decode.h"
#include "ft8/message.h"
#include "ft8/constants.h"
#include "ft8_time.h" /* UTC timestamp for each decoded line (system clock on the ESP32-P4) */
#include "ft8_port.h"
#include <math.h>
#include <time.h>

/* Your own QTH's Maidenhead grid locator (09/2026, per the project
 * owner's request to show distance-to-station on decoded CQ lines) -
 * This is only the compiled-in default: it can be changed at runtime
 * with the PC time-sync tool's MSG_SET_GRID frame (time_sync.c) and is
 * then persisted in NVS (see ft8_app.c). 4 characters
 * (field+square, e.g. "IL18") is enough for a perfectly good distance
 * estimate; 6 characters (adding the subsquare, e.g. "IL18vl") only
 * refines which ~5x5km cell within that square counts as "home" -
 * negligible next to typical HF distances, but free precision if you
 * already know your 6-character square. grid_to_latlon() below
 * accepts either length. */
#define FT8_OWN_GRID "IL18vl"

/* Maidenhead grid locator -> latitude/longitude of the CELL CENTER
 * (not a corner - a corner would bias distance by up to half a
 * square's width, ~110km at the equator for a 4-character grid).
 * Accepts 4 or 6 characters; anything else is rejected (returns
 * false) rather than guessing. Case-insensitive on the letter pairs,
 * matching how grids are conventionally written either way.
 *
 * EXPLICIT "RR73" GUARD (09/2026 #6, per the project owner: this was
 * still showing a distance on plain "...RR73" sign-off lines, which
 * are NOT a grid square - RR73 is a QSO-ending acknowledgment). This
 * is defense in depth: ft8_lib's own message.c already tags RR73 (and
 * RRR/73) as FTX_FIELD_TOKEN, never FTX_FIELD_GRID, in every decode
 * path checked (ftx_message_decode_std()/_nonstd()) - so the caller's
 * own offsets.types[] scan should already exclude it before this
 * function is ever reached with it. But "RR73" is ALSO, purely by
 * coincidence, a syntactically valid-LOOKING 4-character grid by the
 * plain letter-letter-digit-digit pattern checked below (R and R are
 * both in A-R, 7 and 3 are both digits) - so if that upstream
 * filtering is ever bypassed, changes, or a future caller is added
 * that doesn't do the same offsets.types[] check first, this
 * rejects it right at the source rather than relying solely on every
 * caller getting the filtering right. Checked case-insensitively, same
 * as the rest of this function; only excludes the literal 4-character
 * token, not e.g. a genuine "RR" field square (R,R is a real, valid
 * square in the far Pacific/Antarctic region - only the FULL "RR73"
 * combination is a reserved protocol token). */
static bool grid_to_latlon(const char *grid, int len, float *lat, float *lon)
{
    char c0, c1;

    if (len != 4 && len != 6) { return false; }

    c0 = (char)((grid[0] >= 'a' && grid[0] <= 'z') ? (grid[0] - 'a' + 'A') : grid[0]);
    c1 = (char)((grid[1] >= 'a' && grid[1] <= 'z') ? (grid[1] - 'a' + 'A') : grid[1]);
    if (c0 < 'A' || c0 > 'R' || c1 < 'A' || c1 > 'R') { return false; }
    if (grid[2] < '0' || grid[2] > '9' || grid[3] < '0' || grid[3] > '9') { return false; }
    if (len == 4 && c0 == 'R' && c1 == 'R' && grid[2] == '7' && grid[3] == '3') { return false; } /* see this function's own "RR73" guard comment above */

    *lon = (float)(c0 - 'A') * 20.0f - 180.0f + (float)(grid[2] - '0') * 2.0f;
    *lat = (float)(c1 - 'A') * 10.0f - 90.0f + (float)(grid[3] - '0') * 1.0f;

    if (len == 6) {
        char c4 = (char)((grid[4] >= 'A' && grid[4] <= 'Z') ? (grid[4] - 'A' + 'a') : grid[4]);
        char c5 = (char)((grid[5] >= 'A' && grid[5] <= 'Z') ? (grid[5] - 'A' + 'a') : grid[5]);
        if (c4 < 'a' || c4 > 'x' || c5 < 'a' || c5 > 'x') { return false; }
        *lon += (float)(c4 - 'a') * (2.0f / 24.0f) + (1.0f / 24.0f);
        *lat += (float)(c5 - 'a') * (1.0f / 24.0f) + (1.0f / 48.0f);
    } else {
        *lon += 1.0f; /* center of the 2deg-wide 4-character square */
        *lat += 0.5f; /* center of the 1deg-tall 4-character square */
    }
    return true;
}

/* Great-circle (haversine) distance in km between two lat/lon points
 * - standard formula, single call per decoded CQ line (at most a
 * handful of times per 15s slot), nowhere near hot enough to need
 * anything faster than plain libm sinf/cosf/atan2f/sqrtf. */
static float haversine_km(float lat1, float lon1, float lat2, float lon2)
{
    const float deg2rad = 3.14159265358979323846f / 180.0f;
    const float earth_radius_km = 6371.0f;
    float dlat = (lat2 - lat1) * deg2rad;
    float dlon = (lon2 - lon1) * deg2rad;
    float a = sinf(dlat * 0.5f) * sinf(dlat * 0.5f)
            + cosf(lat1 * deg2rad) * cosf(lat2 * deg2rad) * sinf(dlon * 0.5f) * sinf(dlon * 0.5f);
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    return earth_radius_km * c;
}

/* Same "always miss/no-op" hash interface as test/host/ft8_host_test.c
 * - see ft8_decoder_process_slot()'s header comment on why. */
static bool hash_lookup_stub(ftx_callsign_hash_type_t hash_type, uint32_t hash, char *callsign)
{
    (void)hash_type;
    (void)hash;
    callsign[0] = '\0';
    return false;
}
static void hash_save_stub(const char *callsign, uint32_t n22)
{
    (void)callsign;
    (void)n22;
}

static ft8_decoded_msg_t s_queue[FT8_DECODER_MSG_QUEUE_SIZE];
static uint8_t s_queue_head; /* next slot to write */
static uint8_t s_queue_tail; /* next slot to read */
static uint8_t s_queue_count;
static uint32_t s_total_decoded_count; /* running total since boot - see ft8_decoder_get_total_count() */
/* Candidate count from the LAST ftx_find_candidates() call (09/2026) -
 * per the project owner's own investigation into growing proc_ms/
 * apparent slot drift: more candidates (busier band) means more LDPC
 * decode attempts, which is real, variable CPU cost - this exposes
 * that number directly so it can be correlated on-screen against
 * proc_ms/the arm-cycle drift, instead of guessing whether a busy
 * band is the cause. */
static int s_last_num_candidates;
static int s_last_num_decoded;
static float s_last_dt[FT8_DECODER_MAX_TIMING];
static int s_last_num_dt;
static int s_last_top_score;
FT8_LOCK_DECLARE(s_queue_lock); /* FT8 task pushes, LVGL task pops - see ft8_port.h */
/* Own QTH lat/lon, plus the grid string itself (kept so ft8_app.c can
 * persist it in NVS via ft8_decoder_get_own_grid()). s_own_latlon_valid
 * stays false (distance field omitted rather than a bogus "0km") if the
 * grid currently set was ever malformed - see ft8_decoder_set_own_grid(). */
static float s_own_lat, s_own_lon;
static bool s_own_latlon_valid;
static char s_own_grid[7]; /* 6 chars + NUL - see ft8_decoder_set_own_grid()'s own comment on why this is never longer */

/* Sets the QTH used for the distance-to-grid field on decoded CQ lines
 * (see grid_to_latlon()/haversine_km() above). Called once at boot with the
 * compiled-in FT8_OWN_GRID default (ft8_decoder_init()), then by ft8_app.c
 * with the value stored in NVS, and by time_sync.c when a MSG_SET_GRID frame
 * arrives from the PC tool.
 *
 * len is truncated to 6 (a Maidenhead locator is never longer) before
 * storing, but the FULL original value is still handed to grid_to_latlon()
 * for validation - truncating first would silently accept a 6-character
 * PREFIX of a longer garbage value. On a malformed grid this leaves
 * s_own_latlon_valid false and s_own_grid EMPTY: distance is simply omitted
 * from every line rather than computed from a bogus position. */
void ft8_decoder_set_own_grid(const char *grid, int len)
{
    int copy_len = (len < 6) ? len : 6;
    int i;

    s_own_latlon_valid = grid_to_latlon(grid, len, &s_own_lat, &s_own_lon);
    if (s_own_latlon_valid) {
        for (i = 0; i < copy_len; i++) { s_own_grid[i] = grid[i]; }
        s_own_grid[copy_len] = '\0';
    } else {
        s_own_grid[0] = '\0';
    }
}

/* See ft8_decoder_set_own_grid()'s own comment. Never NULL - "" when
 * no valid grid is currently set. */
const char *ft8_decoder_get_own_grid(void)
{
    return s_own_grid;
}

void ft8_decoder_init(void)
{
    s_queue_head = 0U;
    s_queue_tail = 0U;
    s_queue_count = 0U;
    s_total_decoded_count = 0U;
    ft8_decoder_set_own_grid(FT8_OWN_GRID, (int)(sizeof(FT8_OWN_GRID) - 1U));
}

static void queue_push(const char *line)
{
    uint8_t i;

    FT8_LOCK(s_queue_lock);

    s_total_decoded_count++; /* count every real decode, even if the display queue below is full and this one gets dropped - see ft8_decoder_get_total_count() */

    if (s_queue_count >= FT8_DECODER_MSG_QUEUE_SIZE)
    {
        FT8_UNLOCK(s_queue_lock);
        return; /* queue full - drop rather than overwrite undrained messages; the UI is expected to drain promptly */
    }

    for (i = 0U; i < FT8_DECODER_LINE_LEN - 1U && line[i] != '\0'; i++)
    {
        s_queue[s_queue_head].line[i] = line[i];
    }
    s_queue[s_queue_head].line[i] = '\0';

    s_queue_head = (uint8_t)((s_queue_head + 1U) % FT8_DECODER_MSG_QUEUE_SIZE);
    s_queue_count++;

    FT8_UNLOCK(s_queue_lock);
}

bool ft8_decoder_get_message(ft8_decoded_msg_t *out)
{
    FT8_LOCK(s_queue_lock);

    if (s_queue_count == 0U)
    {
        FT8_UNLOCK(s_queue_lock);
        return false;
    }

    *out = s_queue[s_queue_tail];
    s_queue_tail = (uint8_t)((s_queue_tail + 1U) % FT8_DECODER_MSG_QUEUE_SIZE);
    s_queue_count--;

    FT8_UNLOCK(s_queue_lock);
    return true;
}

uint32_t ft8_decoder_get_total_count(void)
{
    return s_total_decoded_count;
}

int ft8_decoder_get_last_num_candidates(void)
{
    return s_last_num_candidates;
}

int ft8_decoder_get_last_num_decoded(void)
{
    return s_last_num_decoded;
}

int ft8_decoder_get_last_timing(const float **dt_s)
{
    *dt_s = s_last_dt;
    return s_last_num_dt;
}

int ft8_decoder_get_last_top_score(void)
{
    return s_last_top_score;
}

/*
 * Costas sync score at an arbitrary time position u, in time sub-steps
 * (u = time_offset * time_osr + time_sub). Same metric as ft8_lib's own
 * ft8_sync_score() (expected tone vs its frequency and time neighbours,
 * averaged), but evaluated at any u so the peak can be interpolated between
 * sub-steps. mag[] is laid out [block][time_sub][freq_sub][bin], so one time
 * sub-step is freq_osr*num_bins elements and consecutive sub-steps are
 * contiguous - u maps linearly onto the array.
 */
static float sync_score_u(const ftx_waterfall_t *wf, int u, int freq_offset, int freq_sub)
{
    const int osr = wf->time_osr;
    const int rows = wf->num_blocks * osr;
    const int row_stride = wf->freq_osr * wf->num_bins;
    int score = 0;
    int n = 0;

    for (int m = 0; m < FT8_NUM_SYNC; ++m)
    {
        for (int k = 0; k < FT8_LENGTH_SYNC; ++k)
        {
            int r = u + (FT8_SYNC_OFFSET * m + k) * osr;
            const WF_ELEM_T *p8;
            int sm;

            if (r < 0)
            {
                continue;
            }
            if (r >= rows)
            {
                break;
            }
            p8 = wf->mag + (size_t)r * row_stride + (size_t)freq_sub * wf->num_bins + freq_offset;
            sm = kFT8_Costas_pattern[k];
            if (sm > 0)
            {
                score += WF_ELEM_MAG_INT(p8[sm]) - WF_ELEM_MAG_INT(p8[sm - 1]);
                ++n;
            }
            if (sm < 7)
            {
                score += WF_ELEM_MAG_INT(p8[sm]) - WF_ELEM_MAG_INT(p8[sm + 1]);
                ++n;
            }
            if (k > 0 && r - osr >= 0)
            {
                score += WF_ELEM_MAG_INT(p8[sm]) - WF_ELEM_MAG_INT(p8[sm - osr * row_stride]);
                ++n;
            }
            if (k + 1 < FT8_LENGTH_SYNC && r + osr < rows)
            {
                score += WF_ELEM_MAG_INT(p8[sm]) - WF_ELEM_MAG_INT(p8[sm + osr * row_stride]);
                ++n;
            }
        }
    }
    return (n > 0) ? (float)score / (float)n : 0.0f;
}

/* Fine start time (seconds after capture start, pipeline bias NOT removed)
 * of a decoded candidate: best sub-step within +-2 of the candidate, then a
 * parabola through that point and its two neighbours. */
static float fine_start_s(const ftx_waterfall_t *wf, const ftx_candidate_t *cand)
{
    const int u0 = cand->time_offset * wf->time_osr + cand->time_sub;
    int best_u = u0;
    float best = -1e9f;
    float sm, s0, sp, den, delta = 0.0f;

    for (int u = u0 - 2; u <= u0 + 2; u++)
    {
        float s = sync_score_u(wf, u, cand->freq_offset, cand->freq_sub);
        if (s > best)
        {
            best = s;
            best_u = u;
        }
    }
    sm = sync_score_u(wf, best_u - 1, cand->freq_offset, cand->freq_sub);
    s0 = best;
    sp = sync_score_u(wf, best_u + 1, cand->freq_offset, cand->freq_sub);
    den = sm - 2.0f * s0 + sp;
    if (den < -1e-6f)
    {
        delta = 0.5f * (sm - sp) / den;
        if (delta > 0.5f)
        {
            delta = 0.5f;
        }
        if (delta < -0.5f)
        {
            delta = -0.5f;
        }
    }
    return ((float)best_u + delta) / (float)wf->time_osr * FT8_SYMBOL_PERIOD;
}

static int append_str(char *dst, int pos, int max, const char *src)
{
    while (*src != '\0' && pos < max - 1)
    {
        dst[pos] = *src;
        pos++;
        src++;
    }
    dst[pos] = '\0';
    return pos;
}

/* Zero-padded 2-digit decimal (00-99) - for the HH:MM timestamp
 * prefix (09/2026, per the project owner's request to help tune
 * real-world performance over time: seeing WHEN a decode happened,
 * not just how many, matters for correlating against band
 * conditions/time of day). append_int() above doesn't zero-pad
 * (would print "9" not "09"), hence this separate small helper. */
static int append_u2(char *dst, int pos, int max, uint8_t value)
{
    if (pos < max - 1) { dst[pos++] = (char)('0' + (value / 10U) % 10U); }
    if (pos < max - 1) { dst[pos++] = (char)('0' + value % 10U); }
    dst[pos] = '\0';
    return pos;
}

/* Zero-padded 4-digit decimal (0000-9999) - same "keep the column
 * aligned" reasoning as append_u2() above, for the frequency-offset
 * field (09/2026, per the project owner: this window's 0-1600Hz audio
 * passband means freq_hz_i naturally varies between 3 and 4 digits
 * decode to decode, which without padding shifts every field after it
 * out of alignment - e.g. "169Hz" one line, "1169Hz" the next). Only
 * ever fed a non-negative value in practice (a frequency location
 * within the passband can't be negative) - no sign handling, unlike
 * append_int(). Values above 9999 (shouldn't happen at a 1600Hz
 * passband width) are clamped rather than silently truncating a
 * digit off the front, which would misalign in the exact same way
 * this exists to prevent. */
static int append_u4(char *dst, int pos, int max, int value)
{
    unsigned int v = (value < 0) ? 0U : (unsigned int)value;
    if (v > 9999U) { v = 9999U; }
    if (pos < max - 1) { dst[pos++] = (char)('0' + (v / 1000U) % 10U); }
    if (pos < max - 1) { dst[pos++] = (char)('0' + (v / 100U) % 10U); }
    if (pos < max - 1) { dst[pos++] = (char)('0' + (v / 10U) % 10U); }
    if (pos < max - 1) { dst[pos++] = (char)('0' + v % 10U); }
    dst[pos] = '\0';
    return pos;
}

/* Signed, zero-padded, ALWAYS-explicit-sign 2-digit decimal
 * ("+00".."+99" / "-00".."-99") - same "keep the column aligned"
 * reasoning as append_u2()/append_u4() above, for the SNR field
 * (09/2026 #3, per the project owner: "-5dB" one line, "-12dB" the
 * next shifts everything after it out of alignment exactly like the
 * frequency field did). Unlike append_u4(), this value genuinely can
 * be (and usually is) negative - FT8's whole point is decoding well
 * below the noise floor, so cand->score-derived SNR estimates
 * typically run negative (roughly -24 to +10dB in practice) - so the
 * sign is always printed explicitly (even for 0 or positive values)
 * rather than only prepending "-" for negatives the way append_int()
 * does: printing the sign only sometimes would itself reintroduce a
 * 1-character alignment shift between negative and non-negative
 * lines, which is exactly what this exists to avoid. Magnitude
 * clamped to 99 (SNR estimates this project produces shouldn't
 * realistically reach that) for the same "clamp, don't silently
 * misalign" reasoning as append_u4(). */
static int append_s2(char *dst, int pos, int max, int value)
{
    int neg = (value < 0);
    unsigned int v = neg ? (unsigned int)(-value) : (unsigned int)value;
    if (v > 99U) { v = 99U; }
    if (pos < max - 1) { dst[pos++] = neg ? '-' : '+'; }
    if (pos < max - 1) { dst[pos++] = (char)('0' + (v / 10U) % 10U); }
    if (pos < max - 1) { dst[pos++] = (char)('0' + v % 10U); }
    dst[pos] = '\0';
    return pos;
}

/* DT as "+0.3" / "-1.2" (always signed, one decimal, clamped to +-9.9) -
 * fixed width, same column-alignment reasoning as append_s2(). */
static int append_dt(char *dst, int pos, int max, float dt)
{
    int t = (int)(dt * 10.0f + (dt >= 0.0f ? 0.5f : -0.5f));
    int neg = (t < 0);
    unsigned int v = neg ? (unsigned int)(-t) : (unsigned int)t;
    if (v > 99U) { v = 99U; }
    if (pos < max - 1) { dst[pos++] = neg ? '-' : '+'; }
    if (pos < max - 1) { dst[pos++] = (char)('0' + v / 10U); }
    if (pos < max - 1) { dst[pos++] = '.'; }
    if (pos < max - 1) { dst[pos++] = (char)('0' + v % 10U); }
    dst[pos] = '\0';
    return pos;
}

void ft8_decoder_process_slot(void)
{
    ftx_waterfall_t wf_snapshot;
    const ftx_waterfall_t *wf = &wf_snapshot;
    ftx_candidate_t candidate_list[FT8_DECODER_MAX_CANDIDATES];
    int num_candidates;
    ftx_callsign_hash_interface_t hash_if = { hash_lookup_stub, hash_save_stub };
    uint32_t seen_hash[FT8_DECODER_MAX_CANDIDATES];
    int num_seen = 0;
    int i;
    struct tm slot_time; /* captured once per slot (not per-candidate) - all decodes from the same slot share the same real-world reception time, down to the minute shown */

    ft8_time_get_slot_start_utc(&slot_time);

    /*
     * Build the local waterfall struct against the SNAPSHOT copy
     * (09/2026) - see ft8_waterfall_snapshot_mag()'s own comment for
     * the full "why": by the time this function runs, the REAL
     * capture for the NEXT slot has already been reset+rearmed (see
     * main.c's call site - it now snapshots+rearms BEFORE calling
     * this), so ft8_waterfall_get()'s live structure is no longer the
     * completed slot's data at all. max_blocks/num_bins/time_osr/
     * freq_osr/block_stride/protocol are fixed constants for this
     * project (never anything that changes slot to slot), so there's
     * nothing to snapshot for those - only mag[] itself needed a copy.
     */
    wf_snapshot.max_blocks = FT8_ADAPTER_MAX_BLOCKS;
    wf_snapshot.num_blocks = FT8_ADAPTER_MAX_BLOCKS; /* only ever called once a slot is FULL - see this function's own call site */
    wf_snapshot.num_bins = FT8_ADAPTER_NUM_BINS;
    wf_snapshot.time_osr = FT8_ADAPTER_TIME_OSR;
    wf_snapshot.freq_osr = FT8_ADAPTER_FREQ_OSR;
    wf_snapshot.mag = (WF_ELEM_T *)ft8_waterfall_get_mag_snapshot(); /* cast away const - ftx_find_candidates()/ftx_decode_candidate() only ever read through this pointer, never write */
    wf_snapshot.block_stride = FT8_ADAPTER_TIME_OSR * FT8_ADAPTER_FREQ_OSR * FT8_ADAPTER_NUM_BINS;
    wf_snapshot.protocol = FTX_PROTOCOL_FT8;

    num_candidates = ftx_find_candidates(wf, FT8_DECODER_MAX_CANDIDATES, candidate_list, FT8_DECODER_MIN_SCORE);
    s_last_num_candidates = num_candidates;
    s_last_num_decoded = 0;
    s_last_num_dt = 0;
    s_last_top_score = 0;
    for (i = 0; i < num_candidates; i++)
    {
        if (candidate_list[i].score > s_last_top_score)
        {
            s_last_top_score = candidate_list[i].score;
        }
    }

    for (i = 0; i < num_candidates; i++)
    {
        const ftx_candidate_t *cand = &candidate_list[i];
        ftx_message_t message;
        ftx_decode_status_t status;
        char text[FTX_MAX_MESSAGE_LENGTH];
        ftx_message_offsets_t offsets;
        char line[FT8_DECODER_LINE_LEN];
        int pos;
        int freq_hz_i;
        int snr_i;
        int dup;
        int j;
        float dt_s;

        if (!ftx_decode_candidate(wf, cand, FT8_DECODER_LDPC_ITERATIONS, &message, &status))
        {
            continue; /* LDPC didn't converge or CRC mismatch - see status.ldpc_errors/crc_* for diagnostics if this needs investigating further */
        }

        if (ftx_message_decode(&message, &hash_if, text, &offsets) != FTX_MESSAGE_RC_OK)
        {
            continue;
        }

        /* Simple within-this-slot duplicate suppression by message
         * hash - NOT ft8_lib's own hash table (see this file's header
         * comment on why that's skipped for now). A real transmission
         * decoded from more than one nearby candidate (common - sync
         * search often finds several close time/freq offsets for the
         * same signal, exactly as seen clustered in this session's own
         * WAV test) would otherwise show up as repeated lines. */
        dup = 0;
        for (j = 0; j < num_seen; j++)
        {
            if (seen_hash[j] == message.hash) { dup = 1; break; }
        }
        if (dup) { continue; }
        if (num_seen < FT8_DECODER_MAX_CANDIDATES) { seen_hash[num_seen++] = (uint32_t)message.hash; }

        /* Format: "08:14 0169Hz -05dB CQ IU8DMZ JN70 1234km" - HH:MM
         * timestamp (UTC, from the system clock - see ft8_time.h) first, per the
         * project owner's request to correlate decodes against time
         * of day/band conditions while tuning; freq rounded to the
         * nearest Hz and zero-padded to 4 digits (append_u4() above,
         * 09/2026 #2 - keeps this column aligned across the 3-vs-4-
         * digit range this passband actually produces); SNR from
         * cand->score * 0.5 (ft8_lib's own demo uses this exact
         * placeholder formula, marked there as a TODO for "compute
         * better approximation" - carried over as-is, not improved on
         * here), explicit-sign zero-padded to 2 digits (append_s2()
         * above, 09/2026 #3 - same alignment reasoning, and SNR is
         * usually negative in real use: FT8 is designed to decode
         * well below the noise floor). Distance-to-grid suffix
         * (09/2026 #4) appended after the message text, only when it
         * actually contains a grid square (see the FTX_FIELD_GRID scan
         * below) - see grid_to_latlon()/haversine_km()/FT8_OWN_GRID
         * above. */
        freq_hz_i = (int)((FT8_ADAPTER_MIN_RAW_BIN + cand->freq_offset + (float)cand->freq_sub / wf->freq_osr) / FT8_SYMBOL_PERIOD + 0.5f);
        snr_i = (int)(cand->score * 0.5f);
        dt_s = fine_start_s(wf, cand) - FT8_DT_BIAS_S - 0.5f; /* WSJT-X convention */
        if (s_last_num_dt < FT8_DECODER_MAX_TIMING)
        {
            s_last_dt[s_last_num_dt++] = dt_s;
        }

        pos = 0;
        pos = append_u2(line, pos, FT8_DECODER_LINE_LEN, (uint8_t)slot_time.tm_hour);
        pos = append_str(line, pos, FT8_DECODER_LINE_LEN, ":");
        pos = append_u2(line, pos, FT8_DECODER_LINE_LEN, (uint8_t)slot_time.tm_min);
        pos = append_str(line, pos, FT8_DECODER_LINE_LEN, " ");
        pos = append_u4(line, pos, FT8_DECODER_LINE_LEN, freq_hz_i);
        pos = append_str(line, pos, FT8_DECODER_LINE_LEN, "Hz ");
        pos = append_s2(line, pos, FT8_DECODER_LINE_LEN, snr_i);
        pos = append_str(line, pos, FT8_DECODER_LINE_LEN, "dB ");
        pos = append_dt(line, pos, FT8_DECODER_LINE_LEN, dt_s);
        pos = append_str(line, pos, FT8_DECODER_LINE_LEN, " ~ ");
        pos = append_str(line, pos, FT8_DECODER_LINE_LEN, text);

        /* Distance to the DX station's grid square (09/2026, per the
         * project owner) - only standard CQ-with-grid messages
         * ("CQ CALL GRID") carry a grid square at all; anything else
         * (a signal report, RRR, 73, ...) has no FTX_FIELD_GRID entry
         * in offsets, so this quietly adds nothing for those lines
         * rather than guessing. offsets.offsets[i] is the BYTE OFFSET
         * of that field within `text` (see ftx_message_offsets_t's own
         * comment in message.h) - grids are always exactly 4
         * characters in a standard message, so no separate length
         * field is needed to find where it ends. */
        {
            int fi;
            for (fi = 0; fi < FTX_MAX_MESSAGE_FIELDS; fi++)
            {
                if (offsets.offsets[fi] < 0) { break; }
                if (offsets.types[fi] == FTX_FIELD_GRID)
                {
                    float dx_lat, dx_lon;
                    if (s_own_latlon_valid && grid_to_latlon(&text[offsets.offsets[fi]], 4, &dx_lat, &dx_lon))
                    {
                        int km = (int)(haversine_km(s_own_lat, s_own_lon, dx_lat, dx_lon) + 0.5f);
                        pos = append_str(line, pos, FT8_DECODER_LINE_LEN, " ");
                        pos = append_u4(line, pos, FT8_DECODER_LINE_LEN, km);
                        pos = append_str(line, pos, FT8_DECODER_LINE_LEN, "km");
                    }
                    break;
                }
            }
        }
        (void)pos;

        queue_push(line);
        s_last_num_decoded++;
    }
}

bool ft8_decoder_get_own_latlon(float *lat, float *lon)
{
    if (!s_own_latlon_valid)
    {
        return false;
    }
    *lat = s_own_lat;
    *lon = s_own_lon;
    return true;
}

float ft8_distance_km(float lat1, float lon1, float lat2, float lon2)
{
    return haversine_km(lat1, lon1, lat2, lon2);
}
