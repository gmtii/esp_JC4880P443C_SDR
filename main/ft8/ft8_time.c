#include "ft8_time.h"

#include <sys/time.h>
#include <stddef.h>

static volatile ft8_time_src_t s_src = FT8_TIME_SRC_NONE;
static volatile uint32_t s_sync_seq;
static volatile int32_t s_last_jump_ms;

/* Civil date -> days since 1970-01-01 (Howard Hinnant's days_from_civil) -
 * avoids depending on timegm(), which newlib does not provide. */
static int64_t days_from_civil(int64_t y, unsigned m, unsigned d)
{
    int64_t era;
    unsigned yoe, doy, doe;

    y -= (m <= 2U) ? 1 : 0;
    era = (y >= 0 ? y : y - 399) / 400;
    yoe = (unsigned)(y - era * 400);
    doy = (153U * (m > 2U ? m - 3U : m + 9U) + 2U) / 5U + d - 1U;
    doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

int64_t ft8_time_now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + (int64_t)(tv.tv_usec / 1000);
}

static void ms_to_tm(int64_t ms, struct tm *out)
{
    time_t t = (time_t)(ms / 1000);
    gmtime_r(&t, out);
}

void ft8_time_get_utc(struct tm *out)
{
    ms_to_tm(ft8_time_now_ms(), out);
}

void ft8_time_get_slot_start_utc(struct tm *out)
{
    /* Called shortly after a 15 s boundary: stepping back half a slot before
     * flooring lands inside the slot that just ended. */
    int64_t ms = ft8_time_now_ms() - 7500;
    ms -= ms % 15000;
    ms_to_tm(ms, out);
}

bool ft8_time_is_synced(void)
{
    return s_src != FT8_TIME_SRC_NONE;
}

ft8_time_src_t ft8_time_get_source(void)
{
    return s_src;
}

const char *ft8_time_source_name(ft8_time_src_t src)
{
    switch (src)
    {
    case FT8_TIME_SRC_MANUAL:
        return "manual";
    case FT8_TIME_SRC_SERIAL:
        return "serial";
    case FT8_TIME_SRC_NTP:
        return "NTP";
    case FT8_TIME_SRC_AIS:
        return "AIS";
    default:
        return "no sync";
    }
}

void ft8_time_set_epoch_ms(int64_t epoch_ms, ft8_time_src_t src)
{
    struct timeval tv;
    int64_t old_ms = ft8_time_now_ms();

    tv.tv_sec = (time_t)(epoch_ms / 1000);
    tv.tv_usec = (suseconds_t)((epoch_ms % 1000) * 1000);
    settimeofday(&tv, NULL);

    s_last_jump_ms = (int32_t)(epoch_ms - old_ms);
    s_src = src;
    s_sync_seq++;
}

void ft8_time_set_utc_fields(int year, int month, int day, int hour, int minute, int second, ft8_time_src_t src)
{
    int64_t target_s = days_from_civil(year, (unsigned)month, (unsigned)day) * 86400 + (int64_t)hour * 3600 + (int64_t)minute * 60 + second;
    int64_t now_ms = ft8_time_now_ms();

    if (ft8_time_is_synced() && (now_ms / 1000) == target_s)
    {
        /* Already inside that exact second: a whole-second message carries no
         * better information than what we have, so keep the sub-second phase.
         * Still counts as a successful sync (source refresh, no jump). */
        s_src = src;
        return;
    }

    ft8_time_set_epoch_ms(target_s * 1000, src);
}

void ft8_time_shift_ms(int32_t delta_ms, ft8_time_src_t src)
{
    ft8_time_set_epoch_ms(ft8_time_now_ms() + delta_ms, src);
}

void ft8_time_manual_sync_slot(void)
{
    int64_t ms = ft8_time_now_ms();
    int64_t rem = ms % 15000;
    ms -= rem;
    if (rem >= 7500)
    {
        ms += 15000; /* pressed late in the slot: the boundary being marked is the next one */
    }
    ft8_time_set_epoch_ms(ms, FT8_TIME_SRC_MANUAL);
}

void ft8_time_set_time_of_day(int hour, int minute, int second)
{
    int64_t ms = ft8_time_now_ms();
    int64_t day_start = ms - (ms % 86400000);
    int64_t tod = ((int64_t)hour * 3600 + (int64_t)minute * 60 + second) * 1000;
    ft8_time_set_epoch_ms(day_start + tod, FT8_TIME_SRC_MANUAL);
}

void ft8_time_trim_ms(int32_t delta_ms)
{
    struct timeval tv;
    int64_t ms = ft8_time_now_ms() + delta_ms;
    tv.tv_sec = (time_t)(ms / 1000);
    tv.tv_usec = (suseconds_t)((ms % 1000) * 1000);
    settimeofday(&tv, NULL);
}

uint32_t ft8_time_get_sync_seq(void)
{
    return s_sync_seq;
}

int32_t ft8_time_get_last_jump_ms(void)
{
    return s_last_jump_ms;
}
