#ifndef AIS_MSG_H
#define AIS_MSG_H

#include <stdint.h>
#include <stdbool.h>
#include "ais_demod.h"

/*
 * AIS message decoding (ITU-R M.1371) and the vessel table.
 * Handled: 1/2/3 (class A position), 4 (base station: position + UTC),
 * 5 (class A static/voyage), 18/19 (class B position [+ name]), 21 (aid to
 * navigation), 24 A/B (class B static). Others are counted and ignored.
 */

#define AIS_MAX_VESSELS 64
#define AIS_NAME_LEN 21

typedef enum { AIS_KIND_CLASS_A, AIS_KIND_CLASS_B, AIS_KIND_BASE, AIS_KIND_ATON } ais_kind_t;

typedef struct
{
    uint32_t mmsi;
    ais_kind_t kind;
    char name[AIS_NAME_LEN];
    char callsign[8];
    char destination[AIS_NAME_LEN];
    uint8_t ship_type;        /* 0 = unknown */
    bool pos_valid;
    float lat, lon;           /* degrees */
    float sog_kn;             /* <0 unknown */
    float cog_deg;            /* <0 unknown */
    int heading_deg;          /* -1 unknown */
    uint8_t nav_status;       /* class A: 15 = undefined */
    uint32_t last_ms;         /* stream time of the last message */
    uint32_t msgs;
    uint8_t last_channel;     /* 0 = A, 1 = B */
} ais_vessel_t;

typedef struct
{
    uint32_t by_type[28];
    uint32_t decoded, unknown;
    bool utc_valid;
    int year, month, day, hour, minute, second; /* last base station report */
    uint32_t utc_ms;                            /* stream time it was received */
} ais_msg_stats_t;

/* Allocates the vessel table (PSRAM on the P4); called by reset too. */
bool ais_msg_alloc(void);
void ais_msg_reset(void);
/* Decode one CRC-valid frame from ais_demod and update the vessel table. */
void ais_msg_handle(const ais_frame_t *f);

/* Snapshot for the UI, most recently heard first. Returns the count. */
int ais_msg_get_vessels(ais_vessel_t *out, int max);
void ais_msg_get_stats(ais_msg_stats_t *out);

/* Optional printer (host tests / serial log). */
typedef void (*ais_print_fn)(const char *line);
void ais_msg_set_printer(ais_print_fn fn);

/* Called when a base station reports a plausible UTC time (hook for the
 * shared clock). */
typedef void (*ais_utc_fn)(int year, int month, int day, int hour, int minute, int second);
void ais_msg_set_utc_callback(ais_utc_fn fn);

#endif /* AIS_MSG_H */
