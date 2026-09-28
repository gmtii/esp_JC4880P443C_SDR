#ifndef FT8_BAND_SYNC_H
#define FT8_BAND_SYNC_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Band-referenced slot clock for FT8: locks the local 15 s slot grid to the
 * transmissions actually on the air, so no NTP/RTC/serial time source is
 * needed - a rough manual time-of-day setting is enough (or nothing at all).
 *
 * Idea taken from the PortaPack Mayhem FT8 RX (PR #3320, "FT8 RX: add an FT8
 * receiver that locks its slot clock to the band"): every station keys to the
 * same UTC boundary, so the band itself is the time reference. The
 * implementation here is our own and differs in the measurement: instead of
 * dithering the capture phase across slots, each CRC-valid decode is timed
 * directly with sub-step resolution (parabolic interpolation of the Costas
 * sync score, see ft8_decoder_get_last_timing()), which the ESP32-P4 can
 * afford with time_osr=2. Host calibration against WSJT-X: 32 ms stdev per
 * decode, and a slot usually carries several decodes.
 *
 * Only CRC-valid decodes are trusted (never the sync score alone): the
 * Costas array repeats every 36 symbols, so a window misaligned by one full
 * period still scores well on 2 of 3 sync groups, stably, slot after slot -
 * but a decode never succeeds on that alias.
 *
 * States (shown in the UI):
 *   SEARCHING  no decode, band sounds quiet (top sync score < threshold)
 *   HEARD      Costas-like energy present but nothing decodes: clock too far
 *              off, or a receive-chain problem - reported apart on purpose
 *   SYNCING    decodes arriving, grid being steered onto them
 *   LOCKED     median DT within +-60 ms for 2 decoded slots; now only small,
 *              damped trims (the P4 crystal drifts ~1 ms/min, trivial)
 *
 * Acquisition: the decoder already searches -1.6..+3.2 s around the local
 * boundary (ft8_lib's time_offset range), so a clock set "by eye" within
 * about +-2 s decodes on the very first slot and the first correction lands
 * it. If signals are HEARD but nothing decodes for 4 slots (clock set badly),
 * the grid probes 0, +4, -4, +8, -8 s around the value that was set. With no
 * usable clock (never set), the grid is swept in 4 s steps every two captured
 * slots (both even and odd transmit periods), which covers the whole 15 s
 * circle in at most four steps (~3 min on an active band).
 *
 * Pure C, no ESP-IDF dependency: simulated on the host (test/host).
 */

typedef enum
{
    FT8_BSYNC_SEARCHING = 0,
    FT8_BSYNC_HEARD,
    FT8_BSYNC_SYNCING,
    FT8_BSYNC_LOCKED,
} ft8_bsync_state_t;

/* Per-slot measurement, straight from ft8_decoder after a slot is decoded. */
typedef struct
{
    int n_dt;          /* CRC-valid decodes timed */
    const float *dt_s; /* their DT, WSJT-X convention (0 = correct clock) */
    int top_score;     /* best sync score among all candidates */
} ft8_bsync_input_t;

/* Call on FT8 mode entry. clock_roughly_set: the time of day has been set
 * by any means (manual, serial...) so the error is expected to be small and
 * the blind sweep is only used as a last resort. */
void ft8_bsync_reset(bool clock_roughly_set);

/*
 * Call once per decoded slot. Returns the correction to apply to the local
 * clock in ms (positive = move the clock forward), or 0.
 * The caller applies it and, if it is larger than what the running capture
 * can absorb (see FT8_BSYNC_RESTART_MS), restarts the capture at the next
 * boundary and calls ft8_bsync_capture_restarted().
 */
int32_t ft8_bsync_slot(const ft8_bsync_input_t *in);

/* Corrections up to this size are absorbed by the capture already running
 * (it still collects its 93 blocks before the moved boundary); larger ones
 * need a restart at the next boundary. */
#define FT8_BSYNC_RESTART_MS 100

/* The running capture was restarted after a correction: the next decoded
 * slot is measured on the new grid, so it must not be skipped. */
void ft8_bsync_capture_restarted(void);

/* The clock was set by an external source (serial tool, manual entry). */
void ft8_bsync_external_set(void);

ft8_bsync_state_t ft8_bsync_get_state(void);
const char *ft8_bsync_state_name(ft8_bsync_state_t s);

/* Median DT of the last slot that had decodes (seconds), and whether any. */
bool ft8_bsync_get_last_error(float *err_s);

#endif /* FT8_BAND_SYNC_H */
