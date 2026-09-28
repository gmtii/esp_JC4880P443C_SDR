#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>
#include <stdbool.h>

/*
 * PC -> radio time/grid sync link, same binary frame format as the DeepSDR
 * 101 firmware's time_sync.c, so the existing PC tool (time_sync_sdr101.py
 * and its Windows .exe) works unchanged - just point it at the ESP32-P4
 * board's UART0 COM port (115200 8N1, the same port the console/ESP_LOG and
 * uart_commands.c use).
 *
 * Frame (multi-byte fields little-endian):
 *   [0] 0xA5  [1] len  [2] msg_type  [3..] payload  [3+len] crc8  [4+len] 0x5A
 *   crc8 = CRC8-CCITT (poly 0x07, init 0x00) over msg_type + payload.
 *   MSG_FULL_SET (0x01) 7 B: year_lo, year_hi, month, day, hour, minute, second
 *   MSG_SHIFT    (0x02) 3 B: add_one_second (0/1), subsec_lo, subsec_hi
 *                            (clock += 1 s if flag, then -= subsec/256 s - the
 *                            GD32 RTC's shift-register semantics, kept as-is)
 *   MSG_SET_GRID (0x03) 4|6 B: Maidenhead locator, ASCII, no NUL
 *
 * Unlike the GD32 (which polled a USB CDC ring from its main loop), bytes are
 * pushed in one at a time by uart_commands.c's reader task, which shares the
 * port with the plain-text command parser: 0xA5 is not ASCII, so it
 * unambiguously starts a binary frame.
 */

/* Callback fired (from the UART task) after a MSG_SET_GRID frame has been
 * applied to the decoder - ft8_app.c uses it to persist the grid in NVS. */
typedef void (*time_sync_grid_cb_t)(const char *grid);
void time_sync_set_grid_callback(time_sync_grid_cb_t cb);

/*
 * Feeds one received byte. Returns true if the byte belongs to a binary
 * frame (consumed here - the caller must NOT pass it to its text parser),
 * false if it is ordinary text. A frame that stalls for more than ~250 ms is
 * abandoned so a truncated frame can never swallow later text commands.
 */
bool time_sync_feed_byte(uint8_t byte, uint32_t now_ms);

/* Frames rejected (bad CRC / length / end marker) since boot. */
uint32_t time_sync_get_error_count(void);

/* Frames successfully applied since boot. */
uint32_t time_sync_get_ok_count(void);

#endif /* TIME_SYNC_H */
