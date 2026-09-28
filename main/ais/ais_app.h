#ifndef AIS_APP_H
#define AIS_APP_H

#include <stdbool.h>
#include <stddef.h>

/*
 * AIS receive mode. Uses the WFM path's 192 kSps I/Q (tuned to 162.000 MHz,
 * both AIS channels at +-25 kHz): sdrTask calls ais_app_feed_iq() with each
 * wide block (a copy into a PSRAM stream buffer only); the AIS task runs the
 * demodulator (ais_demod.c) and message decoder (ais_msg.c); ais_ui.c shows
 * the vessels. Base-station UTC (message 4) sets the shared clock when no
 * better source (serial tool / NTP) is present.
 */
bool ais_app_init(void);
bool ais_app_available(void);
void ais_app_set_active(bool on);
bool ais_app_is_active(void);
void ais_app_feed_iq(const float *i, const float *q, size_t n);

#endif /* AIS_APP_H */
