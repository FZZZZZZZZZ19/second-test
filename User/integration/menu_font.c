/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "menu_font.h"
#include "menu_font_data.h"
#include "zf_common_headfile.h"
#include <stdint.h>
void menu_font_text(unsigned y, const char *text, unsigned scale, bool selected) {
    if (!text || (scale != 1 && scale != 2) || y + 16*scale > 128) return;
    uint16_t tile[8*16]; /* one glyph only, no extra frame buffer */
    bool ended = false;
    for (unsigned col = 0; col < 20/scale; col++) {
        unsigned char c = ' ';
        if (!ended) { c = (unsigned char)text[col]; if (!c) { ended = true; c = ' '; } }
        if (c < 32 || c > 126) c = '?';
        for (unsigned row = 0; row < 16; row++)
            for (unsigned x = 0; x < 8; x++) {
                bool ink = (menu_glyphs[c-32][row] & (0x80u >> x)) != 0;
                tile[row*8+x] = ink != selected ? 0x0000u : 0xffffu;
            }
        tft180_show_rgb565_image((uint16_t)(col*8*scale), (uint16_t)y, tile,
                                8,16,(uint16_t)(8*scale),(uint16_t)(16*scale),0);
    }
}

void menu_font_cell(unsigned x,unsigned y,unsigned width,unsigned height,const char *text,bool selected) {
    if (!text || !width || height<14 || x+width>160 || y+height>128) return;
    unsigned n=0;
    while (text[n] && n<width/7) n++;
    unsigned glyph_w=n>2?6:7, glyph_h=n>2?12:14;
    uint16_t tile[8*16];
    for(unsigned j=0;j<8*16;j++) tile[j]=selected?0x0000:0xffff;
    tft180_show_rgb565_image((uint16_t)x,(uint16_t)y,tile,8,16,(uint16_t)width,(uint16_t)height,0);
    for(unsigned c=0;c<n;c++) {
        unsigned char glyph=(unsigned char)text[c];
        if(glyph<32 || glyph>126) glyph='?';
        for(unsigned row=0;row<16;row++)for(unsigned col=0;col<8;col++) {
            bool ink=(menu_glyphs[glyph-32][row] & (0x80u>>col))!=0;
            tile[row*8+col]=ink!=selected?0x0000:0xffff;
        }
        tft180_show_rgb565_image((uint16_t)(x+(width-n*glyph_w)/2+c*glyph_w),(uint16_t)(y+(height-glyph_h)/2),tile,8,16,(uint16_t)glyph_w,(uint16_t)glyph_h,0);
    }
    tft180_draw_line((uint16_t)x,(uint16_t)y,(uint16_t)(x+width-1),(uint16_t)y,0);
    tft180_draw_line((uint16_t)x,(uint16_t)(y+height-1),(uint16_t)(x+width-1),(uint16_t)(y+height-1),0);
    tft180_draw_line((uint16_t)x,(uint16_t)y,(uint16_t)x,(uint16_t)(y+height-1),0);
    tft180_draw_line((uint16_t)(x+width-1),(uint16_t)y,(uint16_t)(x+width-1),(uint16_t)(y+height-1),0);
}
