/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_port.h"
#include "board_config.h"
#include "zf_common_headfile.h"
#if !BOARD_PINMAP_CONFIRMED
#error Verify board pin assignment before building target firmware
#endif
static volatile uint64_t time_base;
static volatile uint16_t time_last;
BoardIrqToken board_irq_save(void) {
    uint32_t token;
    __asm volatile("csrr %0, 0x800" : "=r"(token)::"memory");
    __disable_irq();
    return token & 0x88u;
}
void board_irq_restore(BoardIrqToken token) {
    uint32_t mask = 0x88u;
    __asm volatile("csrc 0x800, %0" ::"r"(mask) : "memory");
    __asm volatile("csrs 0x800, %0" ::"r"(token) : "memory");
}
void board_memory_barrier(void) {
    __asm volatile("fence rw,rw" ::: "memory");
}
void board_clock_init(void) {
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    uint32_t hz = clocks.PCLK1_Frequency;
    if ((RCC->CFGR0 & RCC_PPRE1) >= RCC_PPRE1_DIV2)
        hz *= 2;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    TIM_TimeBaseInitTypeDef t = {0};
    t.TIM_Period = 65535;
    t.TIM_Prescaler = (uint16_t)(hz / 1000000u - 1u);
    t.TIM_CounterMode = TIM_CounterMode_Up;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM7, &t);
    TIM_SetCounter(TIM7, 0);
    TIM_Cmd(TIM7, ENABLE);
    time_last = 0;
    time_base = 0;
}
void board_clock_tick(void) {
    BoardIrqToken t = board_irq_save();
    uint16_t raw = (uint16_t)TIM7->CNT;
    time_base += (uint16_t)(raw - time_last);
    time_last = raw;
    board_irq_restore(t);
}
static uint64_t clock_us(void) {
    BoardIrqToken t = board_irq_save();
    uint64_t value = time_base + (uint16_t)((uint16_t)TIM7->CNT - time_last);
    board_irq_restore(t);
    return value;
}
uint32_t board_time_ms(void) {
    return (uint32_t)(clock_us() / 1000u);
}
uint32_t board_time_us(void) {
    return (uint32_t)clock_us();
}
void board_encoder_init(void) {
    encoder_quad_init(BOARD_LEFT_ENCODER, TIM2_ENCOEDER_MAP0_CH1_A0, TIM2_ENCOEDER_MAP0_CH2_A1);
    encoder_quad_init(BOARD_RIGHT_ENCODER, TIM4_ENCOEDER_MAP1_CH1_D12, TIM4_ENCOEDER_MAP1_CH2_D13);
}
uint16_t board_encoder_raw16(SideId s) {
    return (uint16_t)encoder_get_count(s == SIDE_LEFT ? BOARD_LEFT_ENCODER : BOARD_RIGHT_ENCODER);
}
bool board_camera_stop(void) {
    DVP->CR0 &= (uint8_t)~RB_DVP_ENABLE;
    DVP->CR1 &= (uint8_t)~RB_DVP_DMA_EN;
    board_memory_barrier();
    return !(DVP->CR0 & RB_DVP_ENABLE);
}
bool board_camera_start(void) {
    /* Same native initialization as the working example. No per-frame reset. */
    DVP->IFR = 0;
    dvp_camera_init((uint32 *)&mt9v03x_image[0][0], (uint32 *)&mt9v03x_image[0][0],
                    MT9V03X_W * MT9V03X_H, MT9V03X_H);
    board_memory_barrier();
    return (DVP->CR0 & RB_DVP_ENABLE) && (DVP->CR1 & RB_DVP_DMA_EN);
}
void board_controls_init(void) {
    gpio_init(B14, GPI, 1, GPI_PULL_UP);
    gpio_init(B12, GPI, 1, GPI_PULL_UP);
    gpio_init(D8, GPI, 1, GPI_PULL_UP);
}
uint8_t board_key_code(void) {
    return (uint8_t)((gpio_get_level(B14) << 2) | (gpio_get_level(B12) << 1) | gpio_get_level(D8));
}
