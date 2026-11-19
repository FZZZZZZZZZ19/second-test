/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_port.h"
#include "board_config.h"
#include "zf_common_headfile.h"
bool board_motor_init_zero(void) {
    /* Drive EN low as GPIO before selecting PWM alternate functions. */
    gpio_init(E9, GPO, 0, GPO_PUSH_PULL);
    gpio_init(E14, GPO, 0, GPO_PUSH_PULL);
    gpio_init(BOARD_LEFT_PH, GPO, BOARD_LEFT_PH_FORWARD, GPO_PUSH_PULL);
    gpio_init(BOARD_RIGHT_PH, GPO, BOARD_RIGHT_PH_FORWARD, GPO_PUSH_PULL);
    pwm_init(BOARD_LEFT_EN, BOARD_PWM_HZ, 0);
    pwm_init(BOARD_RIGHT_EN, BOARD_PWM_HZ, 0);
    return true;
}
void board_motor_write_en(SideId side, uint32_t duty) {
    if (duty > PWM_DUTY_MAX)
        duty = 0;
    pwm_set_duty(side == SIDE_LEFT ? BOARD_LEFT_EN : BOARD_RIGHT_EN, duty);
}
void board_motor_write_ph(SideId side, bool level) {
    gpio_set_level(side == SIDE_LEFT ? BOARD_LEFT_PH : BOARD_RIGHT_PH, level ? 1 : 0);
}
