#ifndef BOARD_PORT_H
#define BOARD_PORT_H
#include "app_types.h"
typedef uint32_t BoardIrqToken;
BoardIrqToken board_irq_save(void);
void board_irq_restore(BoardIrqToken token);
void board_memory_barrier(void);
void board_clock_init(void);
void board_clock_tick(void);
uint32_t board_time_ms(void);
uint32_t board_time_us(void);
bool board_motor_init_zero(void);
void board_motor_write_en(SideId side, uint32_t duty);
void board_motor_write_ph(SideId side, bool level);
void board_encoder_init(void);
uint16_t board_encoder_raw16(SideId side);
bool board_camera_hold_after_frame_isr(void);
bool board_camera_restart_full_frame(void);
void board_controls_init(void);
uint8_t board_key_code(void);
#endif
