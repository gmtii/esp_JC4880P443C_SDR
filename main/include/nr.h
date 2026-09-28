#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

void IRAM_ATTR NR(float *demod_out_d, const int BUFFER_SIZE, int ANR_on);

#ifdef __cplusplus
}
#endif