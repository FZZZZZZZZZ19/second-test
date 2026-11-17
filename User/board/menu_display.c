/* ST7735S wrapper supplied by the user, physical portrait 128x160. */
#include "menu_display.h"
#include "St7735s.h"
void menu_display_init(void) { st7735s_init(); }
void menu_display_clear(void) { st7735s_clear(ST7735S_WHITE); }
void menu_display_line(uint16_t x,uint16_t y,uint16_t x2,uint16_t y2,uint16_t color) {
    if(x<128 && x2<128 && y<160 && y2<160) st7735s_draw_line(x,y,x2,y2,color);
}
void menu_display_rgb(uint16_t x,uint16_t y,const uint16_t *p,uint16_t w,uint16_t h,uint16_t dw,uint16_t dh,uint8_t unused) {
    (void)unused;
    if(!p || !w || !h || !dw || !dh || x>=128 || y>=160) return;
    uint16_t cw=dw<128-x?dw:128-x, ch=dh<160-y?dh:160-y;
    st7735s_set_window(x,y,x+cw-1,y+ch-1);
    gpio_high(ST7735S_DC_PIN); gpio_low(ST7735S_CS_PIN);
    for(unsigned j=0;j<ch;j++) for(unsigned i=0;i<cw;i++) {
        uint16_t c=p[(j*h/dh)*w+i*w/dw];
        spi_write_8bit(SPI_2,c>>8); spi_write_8bit(SPI_2,c&255);
    }
    gpio_high(ST7735S_CS_PIN);
}
void menu_display_camera(const uint8_t *pixels,uint16_t width,uint16_t height,uint16_t stride) {
    if(!pixels || width<128 || height<120 || stride<width) return;
    unsigned offset=(width-128)/2;
    /* Same 30..157 center crop as supplied main.c; stream each row to avoid
     * its 15 KiB display_buffer on this nearly full 32 KiB SRAM target. */
    for(unsigned row=0;row<120;row++)
        st7735s_show_gray_image(0,16+row,(uint8_t *)(pixels+row*stride+offset),128,1);
}
