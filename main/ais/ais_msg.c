#include "ais_msg.h"

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
#define LOCK() portENTER_CRITICAL(&s_lock)
#define UNLOCK() portEXIT_CRITICAL(&s_lock)
#else
#define LOCK() ((void)0)
#define UNLOCK() ((void)0)
#endif

static ais_vessel_t *s_v; /* [AIS_MAX_VESSELS], PSRAM on the P4 (ais_msg_alloc) */
static int s_nv;
static ais_msg_stats_t s_stats;
static ais_print_fn s_print;
static ais_utc_fn s_utc_cb;

void ais_msg_set_printer(ais_print_fn fn) { s_print = fn; }
void ais_msg_set_utc_callback(ais_utc_fn fn) { s_utc_cb = fn; }

static void pr(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void pr(const char *fmt, ...)
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

bool ais_msg_alloc(void)
{
    if (s_v == NULL)
    {
#ifdef ESP_PLATFORM
        s_v = heap_caps_calloc(AIS_MAX_VESSELS, sizeof(ais_vessel_t), MALLOC_CAP_SPIRAM);
#else
        s_v = calloc(AIS_MAX_VESSELS, sizeof(ais_vessel_t));
#endif
    }
    return s_v != NULL;
}

void ais_msg_reset(void)
{
    if (!ais_msg_alloc())
    {
        return;
    }
    LOCK();
    memset(s_v, 0, AIS_MAX_VESSELS * sizeof(ais_vessel_t));
    s_nv = 0;
    memset(&s_stats, 0, sizeof(s_stats));
    UNLOCK();
}

/* ---- bit field access (message bits, MSB first) ---------------------------- */

static uint32_t u(const ais_frame_t *f, int start, int len)
{
    uint32_t v = 0;
    for (int i = start; i < start + len; i++)
    {
        int bit = (i < f->nbits) ? (f->bytes[i / 8] >> (7 - i % 8)) & 1 : 0;
        v = (v << 1) | (uint32_t)bit;
    }
    return v;
}

static int32_t s(const ais_frame_t *f, int start, int len)
{
    uint32_t v = u(f, start, len);
    if (v & (1u << (len - 1)))
    {
        return (int32_t)(v | ~((1u << len) - 1u));
    }
    return (int32_t)v;
}

/* 6-bit ASCII text; '@' padding and trailing spaces removed */
static void text(const ais_frame_t *f, int start, int nch, char *out, int outlen)
{
    int n = 0;
    for (int i = 0; i < nch && n < outlen - 1; i++)
    {
        if (start + 6 * (i + 1) > f->nbits)
        {
            break;
        }
        uint32_t c = u(f, start + 6 * i, 6);
        out[n++] = (char)(c < 32 ? c + 64 : c);
    }
    out[n] = '\0';
    for (int i = 0; i < n; i++)
    {
        if (out[i] == '@')
        {
            out[i] = '\0';
            n = i;
            break;
        }
    }
    while (n > 0 && out[n - 1] == ' ')
    {
        out[--n] = '\0';
    }
}

/* ---- vessel table -------------------------------------------------------------- */

static ais_vessel_t *vessel(uint32_t mmsi, uint32_t t_ms)
{
    int oldest = 0;
    for (int i = 0; i < s_nv; i++)
    {
        if (s_v[i].mmsi == mmsi)
        {
            return &s_v[i];
        }
        if (s_v[i].last_ms < s_v[oldest].last_ms)
        {
            oldest = i;
        }
    }
    {
        ais_vessel_t *v = (s_nv < AIS_MAX_VESSELS) ? &s_v[s_nv++] : &s_v[oldest]; /* full: evict the stalest */
        memset(v, 0, sizeof(*v));
        v->mmsi = mmsi;
        v->sog_kn = -1.0f;
        v->cog_deg = -1.0f;
        v->heading_deg = -1;
        v->nav_status = 15;
        v->last_ms = t_ms;
        return v;
    }
}

/* positions: 1/10000 minute; 181 deg / 91 deg mean "not available" */
static bool set_pos(ais_vessel_t *v, int32_t lon_raw, int32_t lat_raw)
{
    float lon = (float)lon_raw / 600000.0f, lat = (float)lat_raw / 600000.0f;
    if (lon <= -180.0f || lon > 180.0f || lat < -90.0f || lat > 90.0f)
    {
        return false;
    }
    v->lat = lat;
    v->lon = lon;
    v->pos_valid = true;
    return true;
}

static void set_motion(ais_vessel_t *v, uint32_t sog, uint32_t cog, uint32_t hdg)
{
    v->sog_kn = (sog == 1023) ? -1.0f : (float)sog / 10.0f;
    v->cog_deg = (cog >= 3600) ? -1.0f : (float)cog / 10.0f;
    v->heading_deg = (hdg == 511) ? -1 : (int)hdg;
}

void ais_msg_handle(const ais_frame_t *f)
{
    if (s_v == NULL)
    {
        return;
    }
    const uint32_t type = u(f, 0, 6);
    const uint32_t mmsi = u(f, 8, 30);
    ais_vessel_t *v;

    if (f->nbits < 40)
    {
        return;
    }
    LOCK();
    if (type < 28)
    {
        s_stats.by_type[type]++;
    }
    v = vessel(mmsi, f->t_ms);
    v->last_ms = f->t_ms;
    v->msgs++;
    v->last_channel = (uint8_t)f->channel;

    switch (type)
    {
    case 1:
    case 2:
    case 3:
        if (f->nbits < 168)
        {
            break;
        }
        v->kind = AIS_KIND_CLASS_A;
        v->nav_status = (uint8_t)u(f, 38, 4);
        set_motion(v, u(f, 50, 10), u(f, 116, 12), u(f, 128, 9));
        set_pos(v, s(f, 61, 28), s(f, 89, 27));
        s_stats.decoded++;
        UNLOCK();
        pr("%c %09u A pos %.5f %.5f  %.1f kn  %.1f deg", 'A' + f->channel, (unsigned)mmsi, v->lat, v->lon, v->sog_kn, v->cog_deg);
        return;

    case 4:
        if (f->nbits < 168)
        {
            break;
        }
        v->kind = AIS_KIND_BASE;
        set_pos(v, s(f, 79, 28), s(f, 107, 27));
        {
            int yy = (int)u(f, 38, 14), mo = (int)u(f, 52, 4), dd = (int)u(f, 56, 5);
            int hh = (int)u(f, 61, 5), mi = (int)u(f, 66, 6), ss = (int)u(f, 72, 6);
            s_stats.decoded++;
            if (yy >= 2020 && mo >= 1 && mo <= 12 && dd >= 1 && dd <= 31 && hh < 24 && mi < 60 && ss < 60)
            {
                s_stats.utc_valid = true;
                s_stats.year = yy; s_stats.month = mo; s_stats.day = dd;
                s_stats.hour = hh; s_stats.minute = mi; s_stats.second = ss;
                s_stats.utc_ms = f->t_ms;
                UNLOCK();
                pr("%c %09u base station UTC %04d-%02d-%02d %02d:%02d:%02d", 'A' + f->channel, (unsigned)mmsi, yy, mo, dd, hh, mi, ss);
                if (s_utc_cb != NULL)
                {
                    s_utc_cb(yy, mo, dd, hh, mi, ss);
                }
                return;
            }
        }
        break;

    case 5:
        if (f->nbits < 424)
        {
            break;
        }
        v->kind = AIS_KIND_CLASS_A;
        text(f, 70, 7, v->callsign, sizeof(v->callsign));
        text(f, 112, 20, v->name, sizeof(v->name));
        v->ship_type = (uint8_t)u(f, 232, 8);
        text(f, 302, 20, v->destination, sizeof(v->destination));
        s_stats.decoded++;
        UNLOCK();
        pr("%c %09u A static \"%s\" %s type %u -> %s", 'A' + f->channel, (unsigned)mmsi, v->name, v->callsign, v->ship_type, v->destination);
        return;

    case 18:
    case 19:
        if (f->nbits < 168)
        {
            break;
        }
        v->kind = AIS_KIND_CLASS_B;
        set_motion(v, u(f, 46, 10), u(f, 112, 12), u(f, 124, 9));
        set_pos(v, s(f, 57, 28), s(f, 85, 27));
        if (type == 19 && f->nbits >= 271)
        {
            text(f, 143, 20, v->name, sizeof(v->name));
            v->ship_type = (uint8_t)u(f, 263, 8);
        }
        s_stats.decoded++;
        UNLOCK();
        pr("%c %09u B pos %.5f %.5f  %.1f kn  %.1f deg", 'A' + f->channel, (unsigned)mmsi, v->lat, v->lon, v->sog_kn, v->cog_deg);
        return;

    case 21:
        if (f->nbits < 272)
        {
            break;
        }
        v->kind = AIS_KIND_ATON;
        text(f, 43, 20, v->name, sizeof(v->name));
        set_pos(v, s(f, 164, 28), s(f, 192, 27));
        s_stats.decoded++;
        UNLOCK();
        pr("%c %09u AtoN \"%s\" %.5f %.5f", 'A' + f->channel, (unsigned)mmsi, v->name, v->lat, v->lon);
        return;

    case 24:
        if (f->nbits < 160)
        {
            break;
        }
        if (v->kind != AIS_KIND_CLASS_A)
        {
            v->kind = AIS_KIND_CLASS_B;
        }
        if (u(f, 38, 2) == 0)
        {
            text(f, 40, 20, v->name, sizeof(v->name)); /* part A: name */
        }
        else if (f->nbits >= 132)
        {
            v->ship_type = (uint8_t)u(f, 40, 8);        /* part B: type + callsign */
            text(f, 90, 7, v->callsign, sizeof(v->callsign));
        }
        s_stats.decoded++;
        UNLOCK();
        pr("%c %09u B static \"%s\" %s", 'A' + f->channel, (unsigned)mmsi, v->name, v->callsign);
        return;

    default:
        break;
    }
    s_stats.unknown++;
    UNLOCK();
}

int ais_msg_get_vessels(ais_vessel_t *out, int max)
{
    int n;
    if (s_v == NULL)
    {
        return 0;
    }
    LOCK();
    n = s_nv < max ? s_nv : max;
    memcpy(out, s_v, (size_t)n * sizeof(ais_vessel_t));
    UNLOCK();
    /* most recent first (small n: insertion sort, outside the lock) */
    for (int i = 1; i < n; i++)
    {
        ais_vessel_t t = out[i];
        int j = i - 1;
        while (j >= 0 && out[j].last_ms < t.last_ms)
        {
            out[j + 1] = out[j];
            j--;
        }
        out[j + 1] = t;
    }
    return n;
}

void ais_msg_get_stats(ais_msg_stats_t *out)
{
    LOCK();
    *out = s_stats;
    UNLOCK();
}
