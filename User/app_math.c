/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_core.h"
#include <math.h>
float app_clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
int32_t app_count_delta(uint16_t now, uint16_t previous) {
    uint16_t d = (uint16_t)(now - previous);
    return d < 32768u ? (int32_t)d : (int32_t)d - 65536;
}
uint32_t app_crc32(const uint8_t *data, size_t size) {
    uint32_t crc = ~0u;
    for (size_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (unsigned j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}
