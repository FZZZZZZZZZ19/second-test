#ifndef MENU_DISPLAY_H
#define MENU_DISPLAY_H
#include <stdint.h>
#define MENU_DISPLAY_WIDTH 128u
#define MENU_DISPLAY_HEIGHT 160u
void menu_display_init(void);
void menu_display_clear(void);
void menu_display_line(uint16_t x,uint16_t y,uint16_t x2,uint16_t y2,uint16_t color);
void menu_display_rgb(uint16_t x,uint16_t y,const uint16_t *p,uint16_t w,uint16_t h,uint16_t dw,uint16_t dh,uint8_t unused);
void menu_display_camera(const uint8_t *pixels,uint16_t width,uint16_t height,uint16_t stride);
#endif
