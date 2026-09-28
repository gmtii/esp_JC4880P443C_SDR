#pragma once

/*
 * Platform shim so the DMR demodulator/decoder build both on the ESP32-P4 and
 * in the host test harness (test/host_dmr, plain gcc). Same pattern as
 * ft8/ft8_port.h: all DSP and protocol work runs in one FreeRTOS task
 * (dmr_app.c); only the snapshots the UI reads cross task boundaries, and
 * those are guarded with DMR_LOCK()/DMR_UNLOCK().
 */

#include <stdlib.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "esp_heap_caps.h"

#define DMR_LOCK_DECLARE(name) static portMUX_TYPE name = portMUX_INITIALIZER_UNLOCKED
#define DMR_LOCK(name) portENTER_CRITICAL(&(name))
#define DMR_UNLOCK(name) portEXIT_CRITICAL(&(name))

static inline void *dmr_port_calloc_large(size_t n, size_t sz)
{
    void *p = heap_caps_calloc(n, sz, MALLOC_CAP_SPIRAM);
    return p ? p : heap_caps_calloc(n, sz, MALLOC_CAP_8BIT);
}
#else
#define DMR_LOCK_DECLARE(name) static int name
#define DMR_LOCK(name) ((void)(name))
#define DMR_UNLOCK(name) ((void)(name))

static inline void *dmr_port_calloc_large(size_t n, size_t sz)
{
    return calloc(n, sz);
}
#endif
