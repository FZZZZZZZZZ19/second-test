/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include <math.h>
void motor_init_zero(void) {
    (void)board_motor_init_zero();
}
void motor_brake_zero(void) {
    board_motor_write_en(SIDE_LEFT, 0);
    board_motor_write_en(SIDE_RIGHT, 0);
}
void motor_apply_forward(float left, float right) {
    if (!isfinite(left) || !isfinite(right) || left < 0 || right < 0) {
        motor_brake_zero();
        return;
    }
    float limit = g_app_param.duty_limit;
    left = app_clamp(left, 0, limit);
    right = app_clamp(right, 0, limit);
    board_motor_write_en(SIDE_LEFT, (uint32_t)(left * 10000 + .5f));
    board_motor_write_en(SIDE_RIGHT, (uint32_t)(right * 10000 + .5f));
}
