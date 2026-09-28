#ifndef FT8_TIME_H
#define FT8_TIME_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

/*
 * UTC time source for FT8 slot scheduling and decode timestamps.
 *
 * Uses the ESP32-P4 system clock (gettimeofday()/settimeofday(), driven by the
 * main crystal through esp_timer - sub-millisecond resolution and ~10 ppm
 * accuracy, i.e. well under 1 s/day of drift between syncs). This is the same
 * clock ESP-IDF's SNTP client sets, so a future NTP (ESP32-C6) or external-RTC
 * source only has to call ft8_time_set_epoch_ms() / settimeofday() - nothing
 * else in the FT8 code needs to change.
 *
 * Current sync sources:
 *  - FT8_TIME_SRC_SERIAL: the DeepSDR's PC tool (time_sync_sdr101.py) over the
 *    board's UART0, same wire protocol - see time_sync.c.
 *  - FT8_TIME_SRC_MANUAL: the TIME popup on the FT8 panel - HH:MM:SS typed in
 *    by eye, or the SYNC :00 key pressed at second :00 of a reference clock.
 *
 * Neither needs to be precise: once FT8 is running, the band-sync loop
 * (ft8_band_sync.c) steers this clock onto the transmissions on the air and
 * trims it with ft8_time_trim_ms().
 *
 * Unlike the GD32's RTC (whole seconds only - which forced the "only reseed if
 * the correction is > 2 s" workaround there), this clock keeps the sub-second
 * phase, so the slot grid can be aligned to the true :00/:15/:30/:45 edge.
 */
typedef enum
{
    FT8_TIME_SRC_NONE = 0,
    FT8_TIME_SRC_MANUAL,
    FT8_TIME_SRC_SERIAL,
    FT8_TIME_SRC_NTP, /* reserved for the ESP32-C6 / SNTP integration */
    FT8_TIME_SRC_AIS, /* AIS base station report (message 4), second resolution */
} ft8_time_src_t;

/* UTC milliseconds since 1970-01-01. */
int64_t ft8_time_now_ms(void);

/* Current UTC as broken-down time. */
void ft8_time_get_utc(struct tm *out);

/* UTC start time of the slot that most recently COMPLETED (for timestamping
 * its decodes - WSJT-X convention). Safe to call anywhere in the first ~7 s
 * after the boundary, which is where ft8_app.c runs the decode. */
void ft8_time_get_slot_start_utc(struct tm *out);

bool ft8_time_is_synced(void);
ft8_time_src_t ft8_time_get_source(void);
const char *ft8_time_source_name(ft8_time_src_t src);

/* Hard set from calendar fields with whole-second resolution (MSG_FULL_SET).
 * If the clock is already synced and already inside that same second, it is
 * left untouched so the sub-second phase from an earlier, finer sync is not
 * thrown away. */
void ft8_time_set_utc_fields(int year, int month, int day, int hour, int minute, int second, ft8_time_src_t src);

/* Hard set from an absolute UTC epoch in ms (for SNTP/RTC sources). */
void ft8_time_set_epoch_ms(int64_t epoch_ms, ft8_time_src_t src);

/* Relative correction (MSG_SHIFT). Positive = clock moves forward. */
void ft8_time_shift_ms(int32_t delta_ms, ft8_time_src_t src);

/* "Sync slot" key: snap to the nearest 15 s boundary - press it when a
 * reference clock shows :00, :15, :30 or :45. That is all the band-sync loop
 * needs (it only cares about the slot phase). Minutes and hours are kept. */
void ft8_time_manual_sync_slot(void);

/* Manual time-of-day entry ("by eye", from the FT8 panel's TIME popup): sets
 * HH:MM:SS UTC and keeps the current date. A second or two of error is fine -
 * the band-sync loop (ft8_band_sync.c) takes it from there. */
void ft8_time_set_time_of_day(int hour, int minute, int second);

/* Small correction from the band-sync loop: moves the clock like
 * ft8_time_shift_ms() but keeps the current source and does NOT count as an
 * external sync (ft8_time_get_sync_seq() is unchanged), because the caller
 * (ft8_app.c) handles its own corrections. */
void ft8_time_trim_ms(int32_t delta_ms);

/* Incremented on every correction; ft8_app.c compares it against its last
 * seen value to decide whether the running capture is still aligned. */
uint32_t ft8_time_get_sync_seq(void);

/* Size of the most recent correction (new - old), in ms. */
int32_t ft8_time_get_last_jump_ms(void);

#endif /* FT8_TIME_H */
