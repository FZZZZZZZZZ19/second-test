#ifndef MENU_MODEL_H
#define MENU_MODEL_H
#include "app_types.h"
typedef enum { MENU_MAIN, MENU_PARA, MENU_VISION, MENU_PARAMS, MENU_CONTROL,
               MENU_KEYPAD, MENU_RACE_IMAGE, MENU_IMAGE, MENU_LIVE, MENU_HARDWARE, MENU_CAMERA_CAL, MENU_PAGE_COUNT } MenuPage;
typedef enum { MENU_NONE, MENU_START, MENU_RUN30, MENU_VOFA, MENU_STOP,
               MENU_CAMERA, MENU_SAVE, MENU_APPLY_SAVE } MenuAction;
typedef enum { MENU_INPUT_OK, MENU_INPUT_INVALID, MENU_INPUT_SAVE_ERROR } MenuInputError;
typedef struct {
    MenuPage page, parent;
    unsigned selected, field;
    uint8_t cursor[MENU_PAGE_COUNT];
    char input[16];
    MenuInputError input_error;
    uint8_t candidate, stable;
    uint32_t candidate_ms;
} MenuModel;
void menu_init(MenuModel *m);
unsigned menu_count(const MenuModel *m);
const char *menu_label(const MenuModel *m, unsigned index);
const char *menu_title(const MenuModel *m);
MenuAction menu_confirm(MenuModel *m);
MenuAction menu_back(MenuModel *m);
MenuAction menu_key(MenuModel *m, uint8_t code, uint32_t now);
bool menu_apply_value(const MenuModel *m, AppParam *p);
float menu_param_value(const MenuModel *m, const AppParam *p);
void menu_float_text(char *out, unsigned size, float value);
void menu_value_text(const MenuModel *m,const AppParam *p,char *out,unsigned size);
void vofa_pack(const ControlSnapshot *s, uint8_t out[20]);
#endif
