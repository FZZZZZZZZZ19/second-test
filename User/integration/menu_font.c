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
