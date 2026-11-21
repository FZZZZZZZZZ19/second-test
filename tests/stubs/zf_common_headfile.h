#ifndef UI_TEST_ZF_H
#define UI_TEST_ZF_H
#include <stdint.h>
enum { E9=0x49, E11=0x4b, E13=0x4d, E14=0x4e };
#define TIM1_PWM_MAP3_CH1_E9 (0x03000 | E9)
#define TIM1_PWM_MAP3_CH4_E14 (0x03300 | E14)
#define GPO 1
#define GPO_PUSH_PULL 1
#define PWM_DUTY_MAX 10000u
void gpio_init(int pin,int dir,uint8_t level,int mode);
void gpio_set_level(int pin,uint8_t level);
void pwm_init(int channel,uint32_t hz,uint32_t duty);
void pwm_set_duty(int channel,uint32_t duty);
#define TFT180_CROSSWISE 0
#define TFT180_6X8_FONT 0
#define TFT180_8X16_FONT 1
#define TIM6_PIT 6
#define UART_3 3
#define MT9V03X_W 188
#define MT9V03X_H 120
extern uint8_t mt9v03x_image[120][188];
extern volatile uint8_t mt9v03x_finish_flag;
int mt9v03x_init(void);
void tft180_init(void);
void tft180_set_dir(int d);
void tft180_set_font(int f);
void tft180_clear(void);
void tft180_draw_line(uint16_t x,uint16_t y,uint16_t x2,uint16_t y2,uint16_t color);
void tft180_set_color(uint16_t pen,uint16_t bg);
void tft180_show_rgb565_image(uint16_t x,uint16_t y,const uint16_t *p,uint16_t w,uint16_t h,uint16_t dw,uint16_t dh,uint8_t color_mode);
void tft180_show_string(uint16_t x, uint16_t y, const char *s);
void tft180_show_gray_image(uint16_t x,uint16_t y,const uint8_t *p,uint16_t w,uint16_t h,uint16_t dw,uint16_t dh,uint8_t threshold);
void uart_write_buffer(int uart,const uint8_t *p,unsigned len);
void pit_disable(int timer);
void pit_ms_init(int timer,unsigned period);
#endif
