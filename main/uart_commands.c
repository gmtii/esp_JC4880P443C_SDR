#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#include "uart_commands.h"
#include "msi001.h"
#include "esp_timer.h"
#include <time.h>

#include "time_sync.h"
#include "ft8_time.h"
#include "esp_rtl_sdr.h"

#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

static const char *TAG = "UART_CMD";

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);
}

void uart_command_handler(char *cmd) {
    if (strcmp(cmd, "+") == 0) {
        ESP_LOGI(TAG, "Comando recibido: encender LED");
    }
    else if (strcmp(cmd, "-") == 0) {
        ESP_LOGI(TAG, "Comando recibido: apagar LED");
    }
    else if (strcmp(cmd, "usbguard") == 0) {
        /* The esp_rtl_sdr driver latches a "USB fault guard" after three
         * consecutive panics during USB enumeration and then refuses to
         * install. Panics elsewhere that merely happen during enumeration
         * (e.g. the early-boot FFT race fixed in sdr_fft_init()) trip it too.
         * This clears it; reboot afterwards. */
        esp_rtl_sdr_usb_fault_guard_reset();
        ESP_LOGI(TAG, "USB fault guard cleared - reboot now (press the reset button)");
    }
    else if (strcmp(cmd, "utc") == 0) {
        /* Quick check of the clock FT8 schedules from (see ft8/ft8_time.h) */
        struct tm t;
        ft8_time_get_utc(&t);
        ESP_LOGI(TAG, "UTC %04d-%02d-%02d %02d:%02d:%02d (%s), sync frames ok=%lu err=%lu",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec,
                 ft8_time_source_name(ft8_time_get_source()),
                 (unsigned long)time_sync_get_ok_count(), (unsigned long)time_sync_get_error_count());
    }
    else {
        ESP_LOGW(TAG, "Comando desconocido: %s", cmd);
    }
}

void uart_command_loop(void *arg) {
    uint8_t data[BUF_SIZE];
    char command[128];
    int cmd_index = 0;

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, 1, pdMS_TO_TICKS(20));

        if (len > 0) {
            /* Binary time/grid frames from the PC time-sync tool (start byte
             * 0xA5, see ft8/time_sync.h) share this port with the text
             * commands; bytes that belong to a frame never reach the text
             * parser below. */
            if (time_sync_feed_byte(data[0], (uint32_t)(esp_timer_get_time() / 1000))) {
                continue;
            }

            char c = data[0];

            if (c == '\n' || c == '\r') {
                if (cmd_index > 0) {
                    command[cmd_index] = '\0';
                    uart_command_handler(command);
                    cmd_index = 0;
                }
            } else {
                if (cmd_index < sizeof(command) - 1) {
                    command[cmd_index++] = c;
                }
            }
        }
    }
}