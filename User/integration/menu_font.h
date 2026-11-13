#ifndef MENU_FONT_H
#define MENU_FONT_H
#include <stdbool.h>
/* Bold sans-serif bitmap: 8x16; scale 2 gives a 16x32 menu cell. */
void menu_font_text(unsigned y, const char *text, unsigned scale, bool selected);
void menu_font_cell(unsigned x, unsigned y, unsigned width, unsigned height, const char *text, bool selected);
#endif
