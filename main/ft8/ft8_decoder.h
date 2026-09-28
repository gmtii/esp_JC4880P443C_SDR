#ifndef FT8_DECODER_H
#define FT8_DECODER_H

#include <stdint.h>
#include <stdbool.h>

/*
 * ESP32-P4 port note: carried over from the DeepSDR 101 (GD32F450) firmware.
 * Changes: the message queue is now filled by the FT8 task and drained by the
 * LVGL task, so push/pop are guarded (see ft8_port.h); line timestamps come
 * from the system clock (ft8_time.h) instead of the GD32's RTC; and a
 * per-slot decode count is exposed for the UI/log.
 *
 * Orchestrates a real decode pass over a captured slot: calls
 * ft8_lib's ftx_find_candidates()/ftx_decode_candidate()/
 * ftx_message_decode() directly (the pieces the host WAV test
 * (test/host/ft8_host_test.c) already exercised against real off-air
 * recordings), and queues the results as ready-to-display text lines.
 *
 * Deliberately NOT slot-timing aware (same scope limit as
 * ft8_waterfall_adapter.h/ft8_decimator.h) - call
 * ft8_decoder_process_slot() whenever ft8_waterfall_is_full() says a
 * capture is ready; arming/rearming the next capture is the caller's
 * job (see main.c's minimal bring-up hook).
 */

#define FT8_DECODER_MAX_CANDIDATES 140 /* matches ft8_lib's own demo default (kMax_candidates) */
#define FT8_DECODER_MIN_SCORE      10  /* matches ft8_lib's own demo default (kMin_score) */
#define FT8_DECODER_LDPC_ITERATIONS 25 /* matches ft8_lib's own demo default (kLDPC_iterations) */
#define FT8_DECODER_MSG_QUEUE_SIZE 32  /* a busy 1600 Hz window rarely exceeds ~20 decodes per slot; the UI drains every 100 ms */
#define FT8_DECODER_LINE_LEN       72  /* "HH:MM " (6) + "0169Hz -05dB +0.3 ~ " (20) + message (up to 35) + " 1234km" (7) + NUL, rounded up (ESP32-P4: DT column added) */

/*
 * Timing bias of THIS pipeline: a message whose first Costas symbol starts at
 * time T (seconds after the capture started) is measured at
 * T + FT8_DT_BIAS_S by the fine-timing estimator in ft8_decoder.c (the STFT
 * frame for block k is centred on k*0.16 s, plus resampler delay). Calibrated
 * on the host against WSJT-X's own DT column for the ft8_lib WAV corpus
 * (test/host/calibrate_dt.py). The DT printed on each line and fed to the
 * band-sync loop uses the WSJT-X convention: DT = T - 0.5 s, so a station
 * with a correct clock shows DT ~ 0.
 */
#define FT8_DT_BIAS_S              0.163f /* calibrate_dt.py: 508 matched decodes, stdev 0.032 s vs WSJT-X */
#define FT8_DECODER_MAX_TIMING     32    /* per-slot DT measurements kept for the band-sync loop */

/* One decoded message, already formatted as a display-ready line
 * ("freq snr ~ message") - see ft8_decoder_process_slot()'s comment
 * for the exact format. */
typedef struct
{
    char line[FT8_DECODER_LINE_LEN];
} ft8_decoded_msg_t;

/* Call once, at startup (alongside ft8_decimator_reset()/
 * ft8_waterfall_reset()) - clears the message queue. */
void ft8_decoder_init(void);

/* Sets/reads the QTH grid used for the distance-to-grid field on decoded
 * CQ lines. ft8_decoder_init() seeds it from the compiled-in FT8_OWN_GRID
 * default; ft8_app.c then applies the NVS-stored value (if any) and saves it
 * back whenever time_sync.c receives a new one from the PC tool. */
void ft8_decoder_set_own_grid(const char *grid, int len);
const char *ft8_decoder_get_own_grid(void);

/* Runs ftx_find_candidates() + ftx_decode_candidate() +
 * ftx_message_decode() over the mag[] SNAPSHOT (see
 * ft8_waterfall_snapshot_mag()), and pushes every successfully decoded,
 * non-duplicate message into the output queue. Call ONCE per completed slot,
 * from the FT8 task (it walks up to FT8_DECODER_MAX_CANDIDATES candidates,
 * each running a bp_decode() LDPC pass using ~4.4 KB of stack - the FT8
 * task in ft8_app.c is created with a 16 KB stack for that reason). Duplicate
 * suppression is a simple exact-match-within-this-slot check, not
 * ft8_lib's own hash table (this deliberately skips the
 * ftx_callsign_hash_interface_t plumbing for now - compound/hashed
 * callsigns may decode less completely than a full WSJT-X-style
 * multi-pass would give you, but ordinary messages - the large
 * majority in practice - decode in full without it). */
void ft8_decoder_process_slot(void);

/* Pulls one queued decoded-message line. Returns true and fills *out
 * if one was pending, false (leaves *out untouched) if the queue is
 * empty. Thread-safe: called from the LVGL task (ft8_ui.c) while the FT8
 * task pushes. */
bool ft8_decoder_get_message(ft8_decoded_msg_t *out);

/* Running total of real decoded messages since ft8_decoder_init()
 * (i.e. since boot) - counts every successful, non-duplicate decode
 * even if the display queue was full and that particular message got
 * dropped before ever reaching ft8_decoder_get_message(). Added
 * (09/2026) per the project owner's request: a persistent counter,
 * shown continuously in the UI, to help tell at a glance how
 * productive a given band/time/tuning is while experimenting, since
 * the scrolling text panel alone doesn't make totals easy to track. */
uint32_t ft8_decoder_get_total_count(void);

/* Candidate count from the last ftx_find_candidates() call - see its
 * own declaration comment in ft8_decoder.c for why. */
int ft8_decoder_get_last_num_candidates(void);

/* Number of (non-duplicate) messages decoded in the last processed slot. */
int ft8_decoder_get_last_num_decoded(void);

/* DT (WSJT-X convention, seconds) of every CRC-valid message decoded in the
 * last slot, measured with sub-step resolution (parabolic interpolation of
 * the Costas sync score around the decoded candidate). Only CRC-valid decodes
 * are timed: the Costas pattern repeats every 36 symbols, so a window off by
 * one period still scores well on sync alone while every data symbol is
 * wrong - a decode never succeeds on that alias. Returns the count. */
int ft8_decoder_get_last_timing(const float **dt_s);

/* Highest sync score among the last slot's candidates (0 if none): lets the
 * UI tell "band quiet" apart from "signals heard but nothing decodes". */
int ft8_decoder_get_last_top_score(void);

/* Own QTH (centre of the configured grid square) in degrees, false if no
 * valid grid is set - shared with the DMR GPS display (dmr_ui.c). */
bool ft8_decoder_get_own_latlon(float *lat, float *lon);

/* Great-circle distance in km (same haversine as the FT8 distance field). */
float ft8_distance_km(float lat1, float lon1, float lat2, float lon2);

#endif /* FT8_DECODER_H */
