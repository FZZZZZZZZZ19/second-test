#include "menu_display.h"
#include "St7735s.h"
#include <assert.h>
#include <stdio.h>
static uint8_t frame[120][188], bytes[128];
static uint8_t vision_frame[120][128];
static uint8_t *image;
static unsigned stride, crop;
static unsigned rows, count;
void st7735s_init(void) {}
void st7735s_clear(uint16_t c) { assert(c==65535); }
void st7735s_draw_line(uint16_t x,uint16_t y,uint16_t x2,uint16_t y2,uint16_t c) { (void)c; assert(x<128 && x2<128 && y<160 && y2<160); }
void st7735s_set_window(uint16_t x,uint16_t y,uint16_t x2,uint16_t y2) { assert(x<=x2 && y<=y2 && x2<128 && y2<160); }
void gpio_high(int pin) { (void)pin; }
void gpio_low(int pin) { (void)pin; }
void spi_write_8bit(int spi,uint8_t data) { assert(spi==SPI_2 && count<sizeof bytes); bytes[count++]=data; }
void st7735s_show_gray_image(uint16_t x,uint16_t y,uint8_t *p,uint16_t w,uint16_t h) {
    assert(x==0 && y==16+rows && w==128 && h==1);
    assert(p==image+rows*stride+crop); rows++;
}
int main(void) {
    image=&frame[0][0]; stride=188; crop=30;
    menu_display_camera(image,188,120,stride); assert(rows==120);
    menu_display_camera(&frame[0][0],127,120,188); assert(rows==120);
    rows=0; image=&vision_frame[0][0]; stride=128; crop=0;
    menu_display_camera(image,128,120,stride); assert(rows==120);
    uint16_t tile[4]={0x1234,0xabcd,0x5678,0xef01};
    menu_display_rgb(127,158,tile,2,2,4,4,0);
    assert(count==4 && bytes[0]==0x12 && bytes[1]==0x34 && bytes[2]==0x12 && bytes[3]==0x34);
    puts("PASS: real ST7735S adapter center crop, stride, RGB565 bytes, portrait clipping");
}
