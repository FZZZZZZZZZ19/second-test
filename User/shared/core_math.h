#ifndef CORE_MATH_H
#define CORE_MATH_H
#include "app_types.h"
float app_clamp(float x, float lo, float hi);
int32_t app_count_delta(uint16_t now, uint16_t previous);
uint32_t app_crc32(const uint8_t *data, size_t size);
#endif
