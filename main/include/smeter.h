#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

extern float fft_mag_old[SAMPLE_BUFFER_SIZE];
extern float fft_mag[SAMPLE_BUFFER_SIZE];


float calculadBm(void);
void init_smeter(void);

#ifdef __cplusplus
}
#endif