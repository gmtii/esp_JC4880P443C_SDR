#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "math.h"

#include "sdr.h"

#include "sdr_math.h"

// SAM
// new synchronous AM PLL & PHASE detector
// wdsp Warren Pratt, 2016

#define SAM_PLL_HILBERT_STAGES 7
#define OUT_IDX (3 * SAM_PLL_HILBERT_STAGES)


void sam_variables_init(void);
void SAM(float *i_sample_out, float *q_sample_out, float *demod_out, int BUFFER_SIZE, int demod_modo);

#ifdef __cplusplus
}
#endif
