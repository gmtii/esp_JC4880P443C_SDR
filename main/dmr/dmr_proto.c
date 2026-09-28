#include "dmr_proto.h"
#include "dmr_fec.h"
#include "dmr_port.h"
#include "dmr_voice.h"

#include <stdio.h>
#include <string.h>

/* Slot type data types (ETSI TS 102 361-1, 9.3.6) */
enum
{
    DT_PI_HEADER = 0,
    DT_VOICE_LC_HEADER = 1,
    DT_TERMINATOR_LC = 2,
    DT_CSBK = 3,
    DT_MBC_HEADER = 4,
    DT_MBC_CONT = 5,
    DT_DATA_HEADER = 6,
    DT_RATE_12 = 7,
    DT_RATE_34 = 8,
    DT_IDLE = 9,
    DT_RATE_1 = 10,
};

/* Full LC opcodes (FLCO) we act on */
enum
{
    FLCO_GROUP_VOICE = 0x00,
    FLCO_UNIT_VOICE = 0x03,
    FLCO_TA_HEADER = 0x04,
    FLCO_TA_BLOCK1 = 0x05,
    FLCO_TA_BLOCK3 = 0x07,
    FLCO_GPS_INFO = 0x08,
};

/* CACH -> TACT: the 7 TACT bits are CACH bits 0,4,8,12,14,18,22 (ETSI TS
 * 102 361-1 Figure 9.6 interleaving), then Hamming(7,4): AT, TC, LCSS(2). */
static const uint8_t k_tact_pos[7] = {0, 4, 8, 12, 14, 18, 22};

#define TA_BITS (49 + 3 * 56)

typedef struct
{
    dmr_slot_info_t info;   /* what the UI sees */
    int voice_idx;          /* -1 not in voice; 0 = burst A ... 5 = F */
    int emb_bad_run;        /* consecutive B..F bursts whose EMB failed */
    bool call_open;         /* an LC for the current call has been logged */
    uint8_t emb[128];       /* embedded LC fragments B..E */
    int emb_nfrag;
    /* talker alias assembly */
    int ta_format, ta_len, ta_char_bits;
    uint8_t ta_bits[TA_BITS];
    uint8_t ta_have;        /* bit0 header, bit1..3 blocks 1..3 */
} slot_state_t;

static slot_state_t s_slot[2];
static int s_last_slot;
static bool s_direct_family; /* last sync was a TDMA direct mode one */
static bool s_ms_mode;       /* MS sourced stream (simplex / hotspot uplink): no timeslots */
/* No voice reaches the vocoder until a burst has passed a polarity-sensitive
 * FEC check: while the demodulator is still on the wrong polarity, bursts are
 * misclassified (an inverted idle burst reads as a voice sync) and would be
 * played as garbage. */
static bool s_proven;
/* BS slot tracking: timeslots strictly alternate every 30 ms, so the slot is
 * predicted by alternation and the CACH TACT only re-phases the prediction
 * when two consecutive TACTs agree against it. A single TACT with two bit
 * errors "corrects" to the wrong word; trusting it alone sent voice bursts to
 * the other slot and let that slot's idle burst cut the superframe short. */
static int s_tact_disagree;
static bool s_slot_phase_known; /* false right after (re)lock: trust the first good TACT */
static dmr_log_entry_t s_log[DMR_LOG_LEN];
static int s_log_count;
static uint32_t s_log_seq;
static dmr_proto_stats_t s_stats;
static dmr_proto_print_fn s_print;

/* UI snapshots */
DMR_LOCK_DECLARE(s_lock);
static dmr_slot_info_t s_pub_slot[2];
static dmr_log_entry_t s_pub_log[DMR_LOG_LEN];
static int s_pub_log_count;
static dmr_proto_stats_t s_pub_stats;

static void printf_line(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#include <stdarg.h>
static void printf_line(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    if (s_print == NULL)
    {
        return;
    }
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    s_print(buf);
}

void dmr_proto_set_printer(dmr_proto_print_fn fn)
{
    s_print = fn;
}

void dmr_proto_reset(void)
{
    memset(s_slot, 0, sizeof(s_slot));
    for (int i = 0; i < 2; i++)
    {
        s_slot[i].voice_idx = -1;
        s_slot[i].info.cc = -1;
    }
    s_last_slot = 0;
    s_direct_family = false;
    s_ms_mode = false;
    s_proven = false;
    s_tact_disagree = 0;
    s_slot_phase_known = false;
    memset(s_log, 0, sizeof(s_log));
    s_log_count = 0;
    memset(&s_stats, 0, sizeof(s_stats));
    dmr_voice_reset();

    DMR_LOCK(s_lock);
    s_pub_slot[0] = s_slot[0].info;
    s_pub_slot[1] = s_slot[1].info;
    s_pub_log_count = 0;
    s_pub_stats = s_stats;
    DMR_UNLOCK(s_lock);
}

/* ------------------------------------------------------------------------- */

static void burst_bits(const dmr_frame_t *f, uint8_t bits[264])
{
    for (int i = 0; i < 132; i++)
    {
        bits[2 * i] = (uint8_t)((f->burst[i] >> 1) & 1u);
        bits[2 * i + 1] = (uint8_t)(f->burst[i] & 1u);
    }
}

static bool is_voice_sync(dmr_sync_t s)
{
    return s == DMR_SYNC_BS_VOICE || s == DMR_SYNC_MS_VOICE || s == DMR_SYNC_TS1_VOICE || s == DMR_SYNC_TS2_VOICE;
}

static bool is_data_sync(dmr_sync_t s)
{
    return s == DMR_SYNC_BS_DATA || s == DMR_SYNC_MS_DATA || s == DMR_SYNC_TS1_DATA || s == DMR_SYNC_TS2_DATA;
}

/* 1 / 2 for timeslotted streams (repeater, direct mode), 0 for MS simplex. */
static int slot_label(int slot_idx)
{
    return s_ms_mode ? 0 : slot_idx + 1;
}

/* ---- Call log -------------------------------------------------------------- */

static void log_new_call(int slot_idx, uint32_t t_ms)
{
    const dmr_slot_info_t *si = &s_slot[slot_idx].info;
    dmr_log_entry_t *e;

    if (s_log_count == DMR_LOG_LEN)
    {
        memmove(&s_log[0], &s_log[1], (DMR_LOG_LEN - 1) * sizeof(dmr_log_entry_t));
        s_log_count--;
    }
    e = &s_log[s_log_count++];
    e->t_ms = t_ms;
    e->slot = (uint8_t)slot_label(slot_idx);
    e->group = si->group;
    e->dst = si->dst;
    e->src = si->src;
    snprintf(e->alias, sizeof(e->alias), "%s", si->alias);
    e->gps_valid = si->gps_valid;
    e->gps_lat = si->gps_lat;
    e->gps_lon = si->gps_lon;
    memcpy(e->gps_grid, si->gps_grid, sizeof(e->gps_grid));
    e->seq = ++s_log_seq;
}

static void log_update_gps(int slot_idx)
{
    const dmr_slot_info_t *si = &s_slot[slot_idx].info;
    for (int i = s_log_count - 1; i >= 0; i--)
    {
        if (s_log[i].slot == slot_label(slot_idx) && s_log[i].src == si->src)
        {
            s_log[i].gps_valid = true;
            s_log[i].gps_lat = si->gps_lat;
            s_log[i].gps_lon = si->gps_lon;
            memcpy(s_log[i].gps_grid, si->gps_grid, sizeof(s_log[i].gps_grid));
            s_log[i].seq = ++s_log_seq;
            return;
        }
    }
}

void dmr_latlon_to_grid(float lat, float lon, char out[7])
{
    float x = lon + 180.0f, y = lat + 90.0f;
    if (x < 0.0f) x = 0.0f;
    if (x >= 360.0f) x = 359.9999f;
    if (y < 0.0f) y = 0.0f;
    if (y >= 180.0f) y = 179.9999f;
    out[0] = (char)('A' + (int)(x / 20.0f));
    out[1] = (char)('A' + (int)(y / 10.0f));
    out[2] = (char)('0' + (int)(x / 2.0f) % 10);
    out[3] = (char)('0' + (int)y % 10);
    out[4] = (char)('a' + (int)((x - 2.0f * (int)(x / 2.0f)) * 12.0f));
    out[5] = (char)('a' + (int)((y - (int)y) * 24.0f));
    out[6] = '\0';
}

/* Two's complement field of n bits (MSB first) as a signed value. */
static int32_t sfield(const uint8_t *bits, int n)
{
    uint32_t v = dmr_bits_to_u32(bits, n);
    if (v & (1u << (n - 1)))
    {
        return (int32_t)v - (int32_t)(1u << n);
    }
    return (int32_t)v;
}

/*
 * GPS Info LC (ETSI TS 102 361-2, FLCO 0x08), 72 bits:
 *   0-15 PF/R/FLCO/FID, 16-19 reserved, 20-22 position error (2*10^n m, 7 =
 *   unknown), 23-47 longitude (25-bit two's complement, 360/2^25 deg/LSB),
 *   48-71 latitude (24-bit two's complement, 180/2^24 deg/LSB) - about 1-2 m.
 * Checked against dsd-fme's dmr_embedded_gps(); unlike it, the sign is undone
 * as exact two's complement (its "0x800001 - value" is one LSB off).
 */
static void apply_gps_lc(int slot_idx, const uint8_t lc[72], uint32_t t_ms)
{
    slot_state_t *st = &s_slot[slot_idx];
    int err = (int)dmr_bits_to_u32(&lc[20], 3);
    float lon = (float)sfield(&lc[23], 25) * (360.0f / 33554432.0f);
    float lat = (float)sfield(&lc[48], 24) * (180.0f / 16777216.0f);
    (void)t_ms;

    if (lat <= -90.0f || lat >= 90.0f || lon <= -180.0f || lon >= 180.0f)
    {
        return;
    }
    st->info.gps_valid = true;
    st->info.gps_lat = lat;
    st->info.gps_lon = lon;
    st->info.gps_err_m = (err == 7) ? -1 : (err == 0 ? 2 : (err == 1 ? 20 : (err == 2 ? 200 : (err == 3 ? 2000 : (err == 4 ? 20000 : 200000)))));
    dmr_latlon_to_grid(lat, lon, st->info.gps_grid);
    s_stats.gps++;
    log_update_gps(slot_idx);
    {
        char errs[24];
        if (st->info.gps_err_m < 0)
        {
            snprintf(errs, sizeof(errs), "error unknown");
        }
        else
        {
            snprintf(errs, sizeof(errs), "+-%d m", st->info.gps_err_m);
        }
        printf_line("        TS%d GPS %.5f%c %.5f%c  %s  %s", slot_label(slot_idx),
                    lat < 0 ? -lat : lat, lat < 0 ? 'S' : 'N', lon < 0 ? -lon : lon, lon < 0 ? 'W' : 'E',
                    st->info.gps_grid, errs);
    }
}

static void log_update_alias(int slot_idx)
{
    const dmr_slot_info_t *si = &s_slot[slot_idx].info;
    for (int i = s_log_count - 1; i >= 0; i--)
    {
        if (s_log[i].slot == slot_label(slot_idx) && s_log[i].src == si->src)
        {
            snprintf(s_log[i].alias, sizeof(s_log[i].alias), "%s", si->alias);
            s_log[i].seq = ++s_log_seq;
            return;
        }
    }
}

/* ---- Link control ----------------------------------------------------------- */

static uint32_t lc_u24(const uint8_t *b)
{
    return ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
}

/* A group/private voice LC (from a voice header, terminator or embedded).
 * A terminator only confirms IDs: it never opens (or logs) a new call. */
static void apply_voice_lc(int slot_idx, const uint8_t bytes[9], uint32_t t_ms, const char *origin)
{
    const bool terminator = (origin[0] == 'T');
    slot_state_t *st = &s_slot[slot_idx];
    uint8_t flco = bytes[0] & 0x3Fu;
    bool group = (flco == FLCO_GROUP_VOICE);
    uint32_t dst = lc_u24(&bytes[3]);
    uint32_t src = lc_u24(&bytes[6]);
    bool changed = !st->info.ids_valid || st->info.src != src || st->info.dst != dst || st->info.group != group;

    if (changed && st->info.src != src)
    {
        /* new talker: forget the previous one's alias and position */
        st->info.alias[0] = '\0';
        st->ta_have = 0;
        st->info.gps_valid = false;
    }
    st->info.encrypted = (bytes[2] & 0x40u) != 0; /* service options: privacy */
    dmr_voice_set_encrypted(slot_idx, st->info.encrypted);
    st->info.ids_valid = true;
    st->info.group = group;
    st->info.dst = dst;
    st->info.src = src;

    if (!terminator && (changed || !st->call_open))
    {
        st->call_open = true;
        st->info.call_start_ms = t_ms;
        log_new_call(slot_idx, t_ms);
        printf_line("%7.3f TS%d CC%d %-3s %s %s%u  SRC %u", t_ms / 1000.0, slot_label(slot_idx), st->info.cc, origin,
                    group ? "GROUP" : "PRIVATE", group ? "TG " : "ID ", (unsigned)dst, (unsigned)src);
    }
}

static void ta_render(slot_state_t *st, int slot_idx)
{
    /* contiguous bits available: header, then blocks in order */
    int avail = (st->ta_char_bits == 7) ? 49 : 48;
    char out[DMR_ALIAS_MAX];
    int n = 0, maxc, pos = 0;

    if (!(st->ta_have & 1u))
    {
        return;
    }
    for (int b = 1; b <= 3 && (st->ta_have & (1u << b)); b++)
    {
        avail += 56;
    }
    maxc = avail / st->ta_char_bits;
    if (st->ta_len > 0 && st->ta_len < maxc)
    {
        maxc = st->ta_len;
    }
    for (int c = 0; c < maxc && n < DMR_ALIAS_MAX - 1; c++, pos += st->ta_char_bits)
    {
        uint32_t v = dmr_bits_to_u32(&st->ta_bits[pos], st->ta_char_bits);
        if (st->ta_char_bits == 16 && v > 0x7E)
        {
            v = '?';
        }
        if (v >= 0x20 && v < 0x7F)
        {
            out[n++] = (char)v;
        }
        else if (v >= 0xA0 && st->ta_format == 1)
        {
            out[n++] = '?'; /* ISO 8859 accented letters: keep the position visible */
        }
    }
    while (n > 0 && out[n - 1] == ' ')
    {
        n--;
    }
    out[n] = '\0';
    if (strcmp(out, st->info.alias) != 0)
    {
        snprintf(st->info.alias, sizeof(st->info.alias), "%s", out);
        log_update_alias(slot_idx);
        printf_line("        TS%d talker alias \"%s\"", slot_label(slot_idx), out);
    }
}

static void apply_emb_lc(int slot_idx, const uint8_t lc[72], uint32_t t_ms)
{
    slot_state_t *st = &s_slot[slot_idx];
    uint8_t b[9];
    uint8_t flco, fid;

    for (int i = 0; i < 9; i++)
    {
        b[i] = (uint8_t)dmr_bits_to_u32(&lc[i * 8], 8);
    }
    flco = b[0] & 0x3Fu;
    fid = b[1];

    if (flco == FLCO_GROUP_VOICE || flco == FLCO_UNIT_VOICE)
    {
        apply_voice_lc(slot_idx, b, t_ms, "EMB");
    }
    else if ((fid == 0 || fid == 0x68) && flco == FLCO_GPS_INFO) /* standard, or Hytera's FID */
    {
        apply_gps_lc(slot_idx, lc, t_ms);
    }
    else if (fid == 0 && flco == FLCO_TA_HEADER)
    {
        int fmt = (int)dmr_bits_to_u32(&lc[16], 2);
        st->ta_format = fmt;
        st->ta_len = (int)dmr_bits_to_u32(&lc[18], 5);
        st->ta_char_bits = (fmt == 0) ? 7 : (fmt == 3 ? 16 : 8);
        if (st->ta_char_bits == 7)
        {
            memcpy(st->ta_bits, &lc[23], 49);
        }
        else
        {
            memcpy(st->ta_bits, &lc[24], 48);
        }
        st->ta_have = 1u; /* a new header invalidates older blocks */
        ta_render(st, slot_idx);
    }
    else if (fid == 0 && flco >= FLCO_TA_BLOCK1 && flco <= FLCO_TA_BLOCK3 && (st->ta_have & 1u))
    {
        int blk = flco - FLCO_TA_HEADER; /* 1..3 */
        int base = (st->ta_char_bits == 7) ? 49 : 48;
        memcpy(&st->ta_bits[base + (blk - 1) * 56], &lc[16], 56);
        st->ta_have |= (uint8_t)(1u << blk);
        ta_render(st, slot_idx);
    }
}

/* ---- Burst handlers ------------------------------------------------------------ */

static bool handle_data_burst(int slot_idx, const uint8_t bits[264], uint32_t t_ms)
{
    slot_state_t *st = &s_slot[slot_idx];
    uint8_t stbits[20], info[196], data[96], bytes[12];
    uint32_t cw;
    int cc, dt;

    memcpy(stbits, &bits[98], 10);
    memcpy(&stbits[10], &bits[156], 10);
    cw = dmr_bits_to_u32(stbits, 20);
    if (!dmr_golay_20_8_decode(&cw))
    {
        return false;
    }
    cc = (int)((cw >> 16) & 0xFu);
    dt = (int)((cw >> 12) & 0xFu);
    st->info.cc = cc;
    st->voice_idx = -1;

    memcpy(info, bits, 98);
    memcpy(&info[98], &bits[166], 98);

    switch (dt)
    {
    case DT_VOICE_LC_HEADER:
    case DT_TERMINATOR_LC:
        /* BPTC leftovers are not fatal: the RS(12,9) behind it corrects one
         * more byte and then has to verify all three syndromes. */
        (void)dmr_bptc196_decode(info, data);
        {
            for (int i = 0; i < 12; i++)
            {
                bytes[i] = (uint8_t)dmr_bits_to_u32(&data[i * 8], 8);
            }
            if (dmr_rs_12_9_decode(bytes, dt == DT_VOICE_LC_HEADER ? DMR_RS_MASK_VOICE_HEADER : DMR_RS_MASK_TERMINATOR))
            {
                uint8_t flco = bytes[0] & 0x3Fu;
                if (dt == DT_VOICE_LC_HEADER)
                {
                    s_stats.lc_headers++; /* repeated headers of one call log it once */
                }
                else
                {
                    s_stats.terminators++;
                }
                if (flco == FLCO_GROUP_VOICE || flco == FLCO_UNIT_VOICE)
                {
                    apply_voice_lc(slot_idx, bytes, t_ms, dt == DT_VOICE_LC_HEADER ? "VLC" : "TLC");
                }
            }
        }
        if (dt == DT_TERMINATOR_LC)
        {
            if (st->call_open)
            {
                printf_line("%7.3f TS%d end of call (%.1f s)", t_ms / 1000.0, slot_label(slot_idx),
                            (t_ms - st->info.call_start_ms) / 1000.0);
            }
            st->call_open = false;
            st->info.activity = DMR_ACT_IDLE;
            dmr_voice_call_end(slot_idx);
        }
        else
        {
            st->info.activity = DMR_ACT_VOICE;
        }
        break;

    case DT_CSBK:
        if (dmr_bptc196_decode(info, data) == 0)
        {
            uint16_t crc = (uint16_t)dmr_bits_to_u32(&data[80], 16);
            if ((dmr_crc_ccitt(data, 80) ^ DMR_CRC_MASK_CSBK) == crc)
            {
                s_stats.csbk++;
            }
        }
        st->info.activity = DMR_ACT_CSBK;
        break;

    case DT_IDLE:
        s_stats.idle++;
        st->info.activity = DMR_ACT_IDLE;
        break;

    default:
        st->info.activity = DMR_ACT_DATA;
        break;
    }
    return true;
}

static bool handle_voice_nosync(int slot_idx, const uint8_t burst[132], const uint8_t bits[264], uint32_t t_ms)
{
    slot_state_t *st = &s_slot[slot_idx];
    uint8_t emb[16];
    uint32_t cw;
    int lcss;

    /* B..F follow a detected burst A. (Tried: treating a sync-less burst
     * after F as the next A. It only helped at ~5 dB SNR and made the decoder
     * play non-voice bursts at the end of overs - reverted. The real cause of
     * the lost superframes was the slot assignment, see s_tact_disagree.) */
    st->voice_idx++;
    if (st->voice_idx > 5)
    {
        st->voice_idx = -1; /* no new burst A: the call paused or ended */
        return false;
    }
    /* the AMBE frames carry their own FEC: decode them even if the EMB fails */
    if (s_proven)
    {
        dmr_voice_burst(slot_idx, burst, st->voice_idx, t_ms);
    }

    memcpy(emb, &bits[108], 8);
    memcpy(&emb[8], &bits[148], 8);
    cw = dmr_bits_to_u32(emb, 16);
    if (!dmr_qr_16_7_decode(&cw))
    {
        st->emb_nfrag = 0;
        if (++st->emb_bad_run >= 3)
        {
            st->voice_idx = -1; /* three undecodable EMBs in a row: the call has gone */
        }
        return false;
    }
    st->emb_bad_run = 0;
    st->info.cc = (int)((cw >> 12) & 0xFu);
    lcss = (int)((cw >> 9) & 0x3u);
    st->info.activity = DMR_ACT_VOICE;

    /* LCSS: 1 = first fragment, 3 = continuation, 2 = last, 0 = single */
    if (lcss == 1)
    {
        memcpy(st->emb, &bits[116], 32);
        st->emb_nfrag = 1;
    }
    else if (lcss == 3 && st->emb_nfrag >= 1 && st->emb_nfrag <= 2)
    {
        memcpy(&st->emb[32 * st->emb_nfrag], &bits[116], 32);
        st->emb_nfrag++;
    }
    else if (lcss == 2 && st->emb_nfrag == 3)
    {
        uint8_t lc[72];
        memcpy(&st->emb[96], &bits[116], 32);
        st->emb_nfrag = 0;
        if (dmr_emb_lc_decode(st->emb, lc))
        {
            s_stats.emb_lc_ok++;
            apply_emb_lc(slot_idx, lc, t_ms);
        }
        else
        {
            s_stats.emb_lc_bad++;
        }
    }
    else if (lcss != 0)
    {
        st->emb_nfrag = 0; /* out of sequence */
    }
    return true;
}

/* ---- Entry point ---------------------------------------------------------------- */

int dmr_proto_frame(const dmr_frame_t *f)
{
    uint8_t bits[264];
    int slot_idx;
    int grade = DMR_FRAME_BAD;
    slot_state_t *st;

    s_stats.frames++;
    burst_bits(f, bits);

    if (f->sync == DMR_SYNC_TS1_VOICE || f->sync == DMR_SYNC_TS1_DATA ||
        f->sync == DMR_SYNC_TS2_VOICE || f->sync == DMR_SYNC_TS2_DATA)
    {
        s_direct_family = true;
    }
    else if (f->sync != DMR_SYNC_NONE)
    {
        s_direct_family = false;
    }
    if (f->bs || s_direct_family)
    {
        s_ms_mode = false;
    }
    else if (f->sync == DMR_SYNC_MS_VOICE || f->sync == DMR_SYNC_MS_DATA)
    {
        s_ms_mode = true;
    }

    if (f->bs)
    {
        uint8_t cbits[24], tact[7];
        uint32_t cw;
        for (int i = 0; i < 12; i++)
        {
            cbits[2 * i] = (uint8_t)((f->cach[i] >> 1) & 1u);
            cbits[2 * i + 1] = (uint8_t)(f->cach[i] & 1u);
        }
        for (int i = 0; i < 7; i++)
        {
            tact[i] = cbits[k_tact_pos[i]];
        }
        cw = dmr_bits_to_u32(tact, 7);
        slot_idx = 1 - s_last_slot; /* prediction: strict alternation */
        if (dmr_hamming_7_4_decode(&cw))
        {
            int tc = (int)((cw >> 5) & 1u); /* TC bit (not proof of lock, see dmr_proto.h) */
            if (!s_slot_phase_known)
            {
                /* fresh lock: no alternation history yet - the TACT sets the phase */
                slot_idx = tc;
                s_slot_phase_known = true;
                s_tact_disagree = 0;
            }
            else if (tc == slot_idx)
            {
                s_tact_disagree = 0;
            }
            else if (++s_tact_disagree >= 2)
            {
                slot_idx = tc; /* consistently out of phase: re-phase */
                s_tact_disagree = 0;
            }
        }
    }
    else if (f->sync == DMR_SYNC_TS1_VOICE || f->sync == DMR_SYNC_TS1_DATA)
    {
        slot_idx = 0;
    }
    else if (f->sync == DMR_SYNC_TS2_VOICE || f->sync == DMR_SYNC_TS2_DATA)
    {
        slot_idx = 1;
    }
    else if (s_direct_family && f->sync == DMR_SYNC_NONE)
    {
        slot_idx = 1 - s_last_slot; /* direct mode: slots alternate every 30 ms */
    }
    else
    {
        slot_idx = 0; /* MS sourced / simplex: a single channel every 60 ms */
    }
#ifdef DMR_SLOT_TRACE
    printf("SLOT t=%u sync=%d slot=%d known=%d\n", (unsigned)f->t_ms, (int)f->sync, slot_idx, (int)s_slot_phase_known);
#endif
    s_last_slot = slot_idx;
    st = &s_slot[slot_idx];

    if (is_data_sync(f->sync))
    {
        grade = handle_data_burst(slot_idx, bits, f->t_ms) ? DMR_FRAME_OK : DMR_FRAME_BAD;
    }
    else if (is_voice_sync(f->sync))
    {
        st->voice_idx = 0;
        st->emb_nfrag = 0;
        st->emb_bad_run = 0;
        grade = DMR_FRAME_NEUTRAL; /* proven (or not) by the EMBs of B..F */
        if (s_proven)
        {
            dmr_voice_burst(slot_idx, f->burst, 0, f->t_ms);
        }
    }
    else if (st->voice_idx >= 0)
    {
        grade = handle_voice_nosync(slot_idx, f->burst, bits, f->t_ms) ? DMR_FRAME_OK : DMR_FRAME_BAD;
    }

    if (grade == DMR_FRAME_OK)
    {
        s_proven = true;
        st->info.last_ms = f->t_ms;
        s_stats.frames_valid++;
    }

    DMR_LOCK(s_lock);
    s_pub_slot[0] = s_slot[0].info;
    s_pub_slot[1] = s_slot[1].info;
    memcpy(s_pub_log, s_log, sizeof(s_log));
    s_pub_log_count = s_log_count;
    s_pub_stats = s_stats;
    DMR_UNLOCK(s_lock);

    return grade;
}

void dmr_proto_get_slot(int slot_idx, dmr_slot_info_t *out)
{
    DMR_LOCK(s_lock);
    *out = s_pub_slot[slot_idx & 1];
    DMR_UNLOCK(s_lock);
}

int dmr_proto_get_log(dmr_log_entry_t out[DMR_LOG_LEN])
{
    int n;
    DMR_LOCK(s_lock);
    n = s_pub_log_count;
    memcpy(out, s_pub_log, (size_t)n * sizeof(dmr_log_entry_t));
    DMR_UNLOCK(s_lock);
    return n;
}

void dmr_proto_get_stats(dmr_proto_stats_t *out)
{
    DMR_LOCK(s_lock);
    *out = s_pub_stats;
    DMR_UNLOCK(s_lock);
}

bool dmr_proto_is_ms_mode(void)
{
    return s_ms_mode;
}

void dmr_proto_unprove(void)
{
    s_proven = false;
    dmr_voice_reset();
}

void dmr_proto_new_lock(void)
{
    s_slot_phase_known = false; /* timing restarts: slot phase must be re-learned */
}
