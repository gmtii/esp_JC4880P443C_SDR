#pragma once
#include <stdbool.h>
#include <stdint.h>

void hal_encoder_init(void);

int32_t hal_encoder_get_diff(void);
bool hal_encoder_is_pressed(void);