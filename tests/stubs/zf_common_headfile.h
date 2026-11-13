#ifndef UI_TEST_ZF_H
#define UI_TEST_ZF_H
#include <stdint.h>
#define TFT180_CROSSWISE 0
#define TFT180_6X8_FONT 0
#define TIM6_PIT 6
#define UART_3 3
extern uint8_t mt9v03x_image[120][188];
int mt9v03x_init(void);
void tft180_init(void);
void tft180_set_dir(int d);
void tft180_set_font(int f);
void tft180_clear(void);
void tft180_show_string(uint16_t x, uint16_t y, const char *s);
void tft180_show_gray_image(uint16_t x,uint16_t y,const uint8_t *p,uint16_t w,uint16_t h,uint16_t dw,uint16_t dh,uint8_t threshold);
void uart_write_buffer(int uart,const uint8_t *p,unsigned len);
void pit_disable(int timer);
void pit_ms_init(int timer,unsigned period);
#endif
