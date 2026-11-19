#include "menu_display.h"
#include "St7735s.h"
#include <assert.h>
#include <stdio.h>
static uint8_t frame[120][188], bytes[128*120*2];
static uint8_t vision_frame[120][128];
static uint8_t *image;
static unsigned stride, crop;
static unsigned windows, count;
void st7735s_init(void) {}
void st7735s_clear(uint16_t c) { assert(c==65535); }
void st7735s_draw_line(uint16_t x,uint16_t y,uint16_t x2,uint16_t y2,uint16_t c) { (void)c; assert(x<128 && x2<128 && y<160 && y2<160); }
void st7735s_set_window(uint16_t x,uint16_t y,uint16_t x2,uint16_t y2) { assert(x<=x2 && y<=y2 && x2<128 && y2<160); windows++; }
void gpio_high(int pin) { (void)pin; }
void gpio_low(int pin) { (void)pin; }
void spi_write_8bit(int spi,uint8_t data) { assert(spi==SPI_2 && count<sizeof bytes); bytes[count++]=data; }
static void check_camera(void) {
    assert(windows==1 && count==128*120*2);
    for(unsigned y=0;y<120;y++) for(unsigned x=0;x<128;x++) {
        uint8_t gray=image[y*stride+crop+x];
        uint16_t color=((gray & 0xf8u)<<8) | ((gray & 0xfcu)<<3) | (gray>>3);
        unsigned i=(y*128+x)*2;
        assert(bytes[i]==(color>>8) && bytes[i+1]==(color & 255));
    }
}
int main(void) {
    for(unsigned y=0;y<120;y++) for(unsigned x=0;x<188;x++) frame[y][x]=(uint8_t)(y*7+x);
    for(unsigned y=0;y<120;y++) for(unsigned x=0;x<128;x++) vision_frame[y][x]=(uint8_t)(y+x*3);
    image=&frame[0][0]; stride=188; crop=30;
    menu_display_camera(image,188,120,stride); check_camera();
    menu_display_camera(image,127,120,stride); assert(windows==1 && count==128*120*2);
    windows=count=0; image=&vision_frame[0][0]; stride=128; crop=0;
    menu_display_camera(image,128,120,stride); check_camera();
    count=0;
    uint16_t tile[4]={0x1234,0xabcd,0x5678,0xef01};
    menu_display_rgb(127,158,tile,2,2,4,4,0);
    assert(count==4 && bytes[0]==0x12 && bytes[1]==0x34 && bytes[2]==0x12 && bytes[3]==0x34);
    puts("PASS: real ST7735S adapter single-window center crop, stride, RGB565 bytes, portrait clipping");
}
