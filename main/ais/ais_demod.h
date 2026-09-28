#ifndef AIS_DEMOD_H
#define AIS_DEMOD_H

#include <stdint.h>
#include <stdbool.h>

/*
 * AIS receiver front end: both channels at once from the 192 kSps "wide" I/Q
 * (the WFM path), tuned to 162.000 MHz so that 87B (161.975) and 88B
 * (162.025) sit at -25 and +25 kHz.
 *
 * Per channel: NCO to 0 Hz -> 64-tap low-pass, decimate 4 -> 48 kSps
 * (5 samples/symbol) -> one-symbol differential phase detector -> carrier-
 * offset (DC) tracking gated by a burst detector -> for EACH of the 5
 * sampling phases: slicer -> NRZI -> HDLC (flags, bit de-stuffing) ->
 * CRC-16/X.25; duplicates from neighbouring phases are dropped.
 *
 * Host results (test/host_ais, 180 synthetic messages per point, +500 Hz):
 * 100 % at 20 dB SNR, 99 % at 15 dB, 76 % at 12 dB, 33 % at 10 dB; carrier
 * offset tolerated up to ~2 kHz. AIS-catcher (coherent demodulation) still
 * decodes everything at 10 dB, so there is ~3-4 dB left on the table.
 *
 * GMSK + NRZI is polarity-agnostic (NRZI encodes transitions), so a
 * conjugated front end only swaps which channel is A and which is B - see
 * AIS_IQ_CONJUGATED.
 */

#define AIS_IN_RATE 192000
#define AIS_MAX_FRAME_BYTES 128 /* 1008 bits (5 slots) max + FCS */

/* The ESP32-P4 board's rtl_source delivers conjugated I/Q (see
 * README_DMR.md): RF above the LO appears at negative frequency. */
#define AIS_IQ_CONJUGATED 1

typedef struct
{
    int channel;                         /* 0 = A (161.975 MHz), 1 = B (162.025 MHz) */
    uint8_t bytes[AIS_MAX_FRAME_BYTES];  /* message bits, MSB first (FCS removed)     */
    int nbits;                           /* message length in bits                   */
    float freq_offset_hz;                /* measured carrier offset of this burst    */
    uint32_t t_ms;                       /* stream time                              */
} ais_frame_t;

typedef void (*ais_frame_cb_t)(const ais_frame_t *f, void *ctx);

typedef struct
{
    uint32_t frames_ok[2];
    uint32_t crc_errors[2];
} ais_demod_stats_t;

/* Allocates the per-channel state (PSRAM on the P4). Called by init/reset
 * too; returns false if the allocation failed. */
bool ais_demod_alloc(void);
void ais_demod_init(ais_frame_cb_t cb, void *ctx);
void ais_demod_reset(void);

/* n complex samples at AIS_IN_RATE. */
void ais_demod_feed(const float *i, const float *q, int n);

void ais_demod_get_stats(ais_demod_stats_t *out);

/* Stream clock (ms of I/Q processed since reset) - same base as frame t_ms. */
uint32_t ais_demod_now_ms(void);

/* CRC-16/X.25 over bytes in HDLC (LSB-first) order; exposed for tests. */
uint16_t ais_crc16(const uint8_t *data, int len);

#endif /* AIS_DEMOD_H */
