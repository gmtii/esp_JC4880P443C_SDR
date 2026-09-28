#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "math.h"

#include "sdr_math.h"

void NR_SS(float *demod_out_d, int BUFFER_SIZE);
void NR_SS_init();

#ifdef __cplusplus
}
#endif
