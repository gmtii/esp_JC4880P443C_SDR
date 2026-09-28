#include "time_sync.h"
#include "ft8_time.h"
#include "ft8_decoder.h"

#define FRAME_START 0xA5U
#define FRAME_END 0x5AU
#define MSG_FULL_SET 0x01U
#define MSG_SHIFT 0x02U
#define MSG_SET_GRID 0x03U
#define MSG_FULL_SET_LEN 7U
#define MSG_SHIFT_LEN 3U
#define MSG_SET_GRID_LEN_4 4U
#define MSG_SET_GRID_LEN_6 6U
#define MSG_PAYLOAD_MAX MSG_FULL_SET_LEN
#define FRAME_MAX (5U + MSG_PAYLOAD_MAX)
#define FRAME_TIMEOUT_MS 250U

static uint8_t s_frame[FRAME_MAX];
static uint8_t s_fill;          /* bytes collected so far (0 = idle) */
static uint32_t s_last_byte_ms; /* for the stall timeout */
static uint32_t s_error_count;
static uint32_t s_ok_count;
static time_sync_grid_cb_t s_grid_cb;

void time_sync_set_grid_callback(time_sync_grid_cb_t cb)
{
    s_grid_cb = cb;
}

static uint8_t crc8_ccitt(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;
    for (uint16_t i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = (uint8_t)((crc & 0x80U) ? (((unsigned)crc << 1) ^ 0x07U) : ((unsigned)crc << 1));
        }
    }
    return crc;
}

static void apply_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    switch (type)
    {
    case MSG_FULL_SET:
        if (len == MSG_FULL_SET_LEN)
        {
            int year = (int)(payload[0] | ((unsigned)payload[1] << 8));
            ft8_time_set_utc_fields(year, payload[2], payload[3], payload[4], payload[5], payload[6], FT8_TIME_SRC_SERIAL);
            s_ok_count++;
            return;
        }
        break;
    case MSG_SHIFT:
        if (len == MSG_SHIFT_LEN)
        {
            uint16_t frac = (uint16_t)(payload[1] | ((uint16_t)payload[2] << 8));
            int32_t delta_ms = (payload[0] != 0U) ? 1000 : 0;
            delta_ms -= (int32_t)(((uint32_t)frac * 1000U) / 256U);
            ft8_time_shift_ms(delta_ms, FT8_TIME_SRC_SERIAL);
            s_ok_count++;
            return;
        }
        break;
    case MSG_SET_GRID:
        if (len == MSG_SET_GRID_LEN_4 || len == MSG_SET_GRID_LEN_6)
        {
            ft8_decoder_set_own_grid((const char *)payload, (int)len);
            if (s_grid_cb != NULL && ft8_decoder_get_own_grid()[0] != '\0')
            {
                s_grid_cb(ft8_decoder_get_own_grid());
            }
            s_ok_count++;
            return;
        }
        break;
    default:
        break;
    }
    s_error_count++;
}

bool time_sync_feed_byte(uint8_t byte, uint32_t now_ms)
{
    if (s_fill > 0U && (uint32_t)(now_ms - s_last_byte_ms) > FRAME_TIMEOUT_MS)
    {
        s_fill = 0U; /* stalled frame: abandon it */
        s_error_count++;
    }

    if (s_fill == 0U)
    {
        if (byte != FRAME_START)
        {
            return false; /* plain text - not ours */
        }
        s_frame[s_fill++] = byte;
        s_last_byte_ms = now_ms;
        return true;
    }

    s_last_byte_ms = now_ms;

    if (s_fill == 1U && byte > MSG_PAYLOAD_MAX)
    {
        s_fill = 0U; /* impossible length byte - resync */
        s_error_count++;
        return true;
    }

    s_frame[s_fill++] = byte;

    if (s_fill >= 3U)
    {
        uint8_t len = s_frame[1];
        uint8_t total = (uint8_t)(5U + len);

        if (s_fill == total)
        {
            uint8_t type = s_frame[2];
            uint8_t rx_crc = s_frame[3U + len];
            uint8_t rx_end = s_frame[4U + len];

            s_fill = 0U;

            if (rx_end != FRAME_END || crc8_ccitt(&s_frame[2], (uint16_t)(1U + len)) != rx_crc)
            {
                s_error_count++;
                return true;
            }
            apply_frame(type, &s_frame[3], len);
        }
    }
    return true;
}

uint32_t time_sync_get_error_count(void)
{
    return s_error_count;
}

uint32_t time_sync_get_ok_count(void)
{
    return s_ok_count;
}
