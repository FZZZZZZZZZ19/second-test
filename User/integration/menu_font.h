#ifndef MENU_FONT_H
#define MENU_FONT_H
#include <stdbool.h>
/* Bold sans-serif bitmap: 8x16; scale 2 gives a 16x32 menu cell. */
void menu_font_text(unsigned y, const char *text, unsigned scale, bool selected);
#endif
