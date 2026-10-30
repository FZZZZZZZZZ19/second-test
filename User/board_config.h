#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H
/* Netlist trace: docs/hardware/epro-netlist.json and 引脚与驱动核对.md.
 * H3[1..4] -> driver CN5[1..4] with a straight 1:1 harness.
 */
#define BOARD_PINMAP_CONFIRMED 1
#define BOARD_LEFT_EN TIM1_PWM_MAP3_CH1_E9
#define BOARD_RIGHT_EN TIM1_PWM_MAP3_CH4_E14
#define BOARD_LEFT_PH E11
#define BOARD_RIGHT_PH E13
/* Electrical initial values only; verify wheel direction before CAL_DIRECTION. */
#define BOARD_LEFT_PH_FORWARD 1
#define BOARD_RIGHT_PH_FORWARD 0
#define BOARD_LEFT_ENCODER TIM2_ENCOEDER
#define BOARD_RIGHT_ENCODER TIM4_ENCOEDER
#define BOARD_PWM_HZ 20000u
#endif
