#pragma once

/*
 * Tiny platform shim so the FT8 DSP/decoder modules (ported from the DeepSDR 101
 * GD32F450 firmware) build unchanged both on the ESP32-P4 and in the host-side
 * WAV test harness (test/host in the delivery - plain gcc, no ESP-IDF).
 *
 * On the GD32 these modules shared state between the audio DMA ISR and the main
 * loop, guarded with __disable_irq()/__enable_irq(). On the ESP32-P4 the whole
 * DSP chain (AGC -> resampler -> FFT -> waterfall encode -> decode) runs inside
 * ONE FreeRTOS task (see ft8_app.c), so that ISR/main-loop race no longer exists.
 * The only state still crossing task boundaries is the decoded-message queue
 * (FT8 task -> LVGL task), guarded with FT8_LOCK()/FT8_UNLOCK() below.
 */

#include <stdlib.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "esp_heap_caps.h"

#define FT8_LOCK_DECLARE(name) static portMUX_TYPE name = portMUX_INITIALIZER_UNLOCKED
#define FT8_LOCK(name) portENTER_CRITICAL(&(name))
#define FT8_UNLOCK(name) portEXIT_CRITICAL(&(name))

/* Large FT8 buffers live in PSRAM: internal RAM on this board is already tight
 * (see the LVGL/USB-host history in sdr.c/rtl_source.c). */
static inline void *ft8_port_calloc_large(size_t n, size_t sz)
{
    void *p = heap_caps_calloc(n, sz, MALLOC_CAP_SPIRAM);
    if (p == NULL)
    {
        p = heap_caps_calloc(n, sz, MALLOC_CAP_8BIT); /* fallback: any RAM that fits */
    }
    return p;
}
#else
#define FT8_LOCK_DECLARE(name) static int name
#define FT8_LOCK(name) ((void)(name))
#define FT8_UNLOCK(name) ((void)(name))

static inline void *ft8_port_calloc_large(size_t n, size_t sz)
{
    return calloc(n, sz);
}
#endif
