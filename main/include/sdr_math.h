
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "math.h"


/* Single-precision, parenthesised. The P4 FPU has no double support, and the old
 * definitions ("2*M_PI", "M_PI/2") were double constants without parentheses, so any
 * float expression using them ran in soft-float. */
#define SDR_PI_F 3.14159265358979323846f
#define SDR_INV_PI_F 0.31830988618379067154f
#define TPI (2.0f * SDR_PI_F)
#define PIH (SDR_PI_F / 2.0f)
#define FOURPI (2.0f * TPI)
#define SIXPI (3.0f * TPI)

float ApproxAtan(float z);
float ApproxAtan2(float y, float x);
float log10f_fast(float X);
float convertToF32(int16_t sample);
int16_t convertToInt16(float sample);
float sign(float x);


#ifdef __cplusplus
}
#endif