/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "menu_model.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct { const char *name; size_t offset; float min, max, ui_scale; } Field;
#define F(name, member, lo, hi) {name, offsetof(AppParam, member), lo, hi, 1}
static const Field fields[] = {
    F("omega Kp", lateral_kp, 0, 10000), F("omega Kd", lateral_kd, 0, 10000),
    F("D filter s", lateral_tau_s, 0, 10),
    F("speed Kp (L+R)", kp_left, 0, 10000), F("speed Ki (L+R)", ki_left, 0, 10000),
    F("left Kp", kp_left, 0, 10000), F("left Ki", ki_left, 0, 10000),
    F("right Kp", kp_right, 0, 10000), F("right Ki", ki_right, 0, 10000),
    F("VOFA target m/s", debug_speed_mps, 0, 5),
    F("duty limit 0..0.8", duty_limit, .001f, .8f),
    F("speed limit m/s", speed_limit_mps, .001f, 5),
    F("normal speed m/s", speed_normal, 0, 5),
    F("left FF", ff_left, 0, 10000), F("right FF", ff_right, 0, 10000),
#define H(name, member, lo, hi) {name, offsetof(AppParam, member), lo, hi, 1000}
    H("body length mm", body_length_m, 0, 3000), H("body width mm", body_width_m, 0, 2000),
    H("wheelbase mm", axle_m, 10, 600), H("wheel track mm", track_m, 30, 600),
    H("eff track mm", effective_track_m, 0, 1500),
    H("L wheel dia mm", wheel_left_m, 10, 300), H("R wheel dia mm", wheel_right_m, 10, 300),
    F("L counts/rev", counts_left, 1, 1000000), F("R counts/rev", counts_right, 1, 1000000),
    H("cam offset mm", camera_forward_m, 0, 3000)
#undef H
};
static const char *const digits[] = {"1","2","3","DEL","4","5","6","CLR","7","8","9","OK",".","0","SAVE","BACK"};
void menu_init(MenuModel *m) { memset(m, 0, sizeof(*m)); m->candidate = m->stable = 7; }
unsigned menu_count(const MenuModel *m) {
    switch (m->page) {
    case MENU_MAIN: return 3;
    case MENU_PARA: return 5;
    case MENU_HARDWARE: return 11;
    case MENU_VISION: return 4;
    case MENU_PARAMS: return 13;
    case MENU_CONTROL: return 6;
    case MENU_KEYPAD: return 16;
    case MENU_LIVE: return 2;
    default: return 1;
    }
}
const char *menu_title(const MenuModel *m) {
    static const char *const titles[] = {"V1.50 MAIN","para","vision","control parameters",
        "control","number input","start / camera","image","motor test","hardware"};
    return m->page == MENU_KEYPAD ? fields[m->field].name : titles[m->page];
}
const char *menu_label(const MenuModel *m, unsigned i) {
    static const char *const main_items[] = {"start","para","control"};
    static const char *const para_items[] = {"vision","control","hardware","save to flash","back"};
    static const char *const control_items[] = {"run 30","image","vofa","VOFA target m/s","stop","back"};
    if (i >= menu_count(m)) return "";
    switch (m->page) {
    case MENU_MAIN: return main_items[i];
    case MENU_PARA: return para_items[i];
    case MENU_CONTROL: return control_items[i];
    case MENU_KEYPAD: return digits[i];
    case MENU_VISION: return i == 3 ? "back" : fields[i].name;
    case MENU_PARAMS: return i == 12 ? "back" : fields[i + 3].name;
    case MENU_HARDWARE: return i==10?"back":fields[15+i].name;
    case MENU_LIVE: return i ? "back" : "stop";
    default: return "back";
    }
}
static void enter(MenuModel *m, MenuPage p) {
    m->cursor[m->page]=(uint8_t)m->selected;
    m->page=p; m->selected=0;
}
static void return_to(MenuModel *m, MenuPage p) {
    m->cursor[m->page]=(uint8_t)m->selected;
    m->page=p; m->selected=m->cursor[p];
    if(m->selected>=menu_count(m)) m->selected=0;
}
static void edit(MenuModel *m, unsigned field) {
    m->parent = m->page; m->field = field; m->input[0] = 0; m->input_error = false;
    enter(m, MENU_KEYPAD);
}
MenuAction menu_back(MenuModel *m) {
    switch(m->page) {
    case MENU_MAIN: return MENU_NONE;
    case MENU_KEYPAD: return_to(m,m->parent); return MENU_NONE;
    case MENU_VISION: case MENU_PARAMS: case MENU_HARDWARE: return_to(m,MENU_PARA); return MENU_NONE;
    case MENU_PARA: return_to(m,MENU_MAIN); return MENU_NONE;
    case MENU_CONTROL: case MENU_RACE_IMAGE: return_to(m,MENU_MAIN); return MENU_STOP;
    case MENU_IMAGE: case MENU_LIVE: return_to(m,MENU_CONTROL); return MENU_STOP;
    case MENU_PAGE_COUNT: return MENU_NONE;
    }
    return MENU_NONE;
}
MenuAction menu_confirm(MenuModel *m) {
    unsigned i = m->selected;
    switch (m->page) {
    case MENU_MAIN:
        if (i == 0) { enter(m, MENU_RACE_IMAGE); return MENU_START; }
        enter(m, i == 1 ? MENU_PARA : MENU_CONTROL); break;
    case MENU_PARA:
        if (i < 2) enter(m, i ? MENU_PARAMS : MENU_VISION);
        else if (i == 2) enter(m,MENU_HARDWARE);
        else if (i == 3) return MENU_SAVE;
        else return menu_back(m);
        break;
    case MENU_VISION:
    case MENU_PARAMS:
    case MENU_HARDWARE:
        if (i == menu_count(m) - 1) return menu_back(m);
        else edit(m, i + (m->page == MENU_PARAMS ? 3 : m->page == MENU_HARDWARE ? 15 : 0));
        break;
    case MENU_CONTROL:
        if (i == 0 || i == 2) { enter(m, MENU_LIVE); return i ? MENU_VOFA : MENU_RUN30; }
        if (i == 1) { enter(m, MENU_IMAGE); return MENU_CAMERA; }
        if (i == 3) edit(m, 9);
        else { if (i == 5) return menu_back(m); return MENU_STOP; }
        break;
    case MENU_KEYPAD: {
        size_t n = strlen(m->input);
        m->input_error = false;
        const char *key = menu_label(m,i);
        if ((key[0] >= '0' && key[0] <= '9') || key[0] == '.') {
            if (n < sizeof(m->input) - 1 && (key[0] != '.' || !strchr(m->input, '.'))) {
                m->input[n] = key[0]; m->input[n+1] = 0;
            } else m->input_error = true;
        } else if (i == 3) { if (n) m->input[n-1] = 0; }
        else if (i == 7) m->input[0] = 0;
        else if (i == 11) return MENU_APPLY;
        else if (i == 14) return MENU_APPLY_SAVE;
        else if (i == 15) return menu_back(m);
        break;
    }
    case MENU_RACE_IMAGE: case MENU_IMAGE: case MENU_LIVE: return menu_back(m);
    case MENU_PAGE_COUNT: return MENU_NONE;
    }
    return MENU_NONE;
}
MenuAction menu_key(MenuModel *m, uint8_t code, uint32_t now) {
    bool keypad = m->page == MENU_KEYPAD;
    /* Installed orientation: up=5 down=4 left=2 right=6 center=1 KEY1/back=3. */
    if (!(code==1 || code==3 || code==4 || code==5 || (keypad && (code==2 || code==6)))) code=7;
    if (code != m->candidate) { m->candidate = code; m->candidate_ms = now; }
    if (now - m->candidate_ms < 40 || m->stable == code) return MENU_NONE;
    m->stable=code;
    if(code==7) return MENU_NONE;
    if(code==3) return menu_back(m);
    if(code==1) return menu_confirm(m);
    if(keypad) {
        int x=(int)(m->selected%4), y=(int)(m->selected/4);
        x+=code==6?1:code==2?-1:0;
        y+=code==4?1:code==5?-1:0;
        if(x>=0 && x<4 && y>=0 && y<4) m->selected=(unsigned)(y*4+x);
    } else if(code==4) m->selected=(m->selected+1)%menu_count(m);
    else if(code==5) m->selected=(m->selected+menu_count(m)-1)%menu_count(m);
    return MENU_NONE;
}

float menu_param_value(const MenuModel *m, const AppParam *p) {
    float value; memcpy(&value, (const char *)p + fields[m->field].offset, sizeof value); return value*fields[m->field].ui_scale;
}
bool menu_apply_value(const MenuModel *m, AppParam *p) {
    char *end;
    if (!m->input[0]) return false;
    float value = strtof(m->input, &end);
    const Field *f = &fields[m->field];
    if (*end || !isfinite(value) || value < f->min || value > f->max) return false;
    value/=f->ui_scale; /* hardware UI uses millimetres; AppParam remains SI */
    if (f->offset==offsetof(AppParam,effective_track_m) && value>0 && value<.03f) return false;
    if ((f->offset==offsetof(AppParam,counts_left) || f->offset==offsetof(AppParam,counts_right)) && floorf(value)!=value) return false;
    if ((m->field == 9 && value > p->speed_limit_mps) ||
        (m->field == 11 && (value < p->debug_speed_mps || value < p->speed_normal ||
                          value < p->speed_corner || value < p->speed_element)) ||
        (m->field == 12 && value > p->speed_limit_mps)) return false;
    memcpy((char *)p + f->offset, &value, sizeof value);
    if (m->field == 3) p->kp_right = value;
    if (m->field == 4) p->ki_right = value;
    return true;
}
void menu_float_text(char *out, unsigned size, float value) {
    /* Integer formatting works with the existing nano printf (no float linker flag). */
    if (!isfinite(value) || fabsf(value) > 1000000) { snprintf(out, size, "invalid"); return; }
    long milli = (long)(fabsf(value) * 1000 + .5f);
    snprintf(out, size, "%s%ld.%03ld", value < 0 ? "-" : "", milli / 1000, milli % 1000);
}
void vofa_pack(const ControlSnapshot *s, uint8_t out[20]) {
    _Static_assert(sizeof(float) == 4, "float32 required");
    float values[4] = {s->targets.left_mps, s->feedback.speed_mps[0],
                       s->targets.right_mps, s->feedback.speed_mps[1]};
    if(s->mode==RUN_DUTY_DEBUG) for(unsigned i=0;i<4;i++) values[i]=NAN;
    for (unsigned i = 0; i < 4; i++) {
        uint32_t bits; memcpy(&bits, &values[i], 4);
        for (unsigned j = 0; j < 4; j++) out[4*i+j] = (uint8_t)(bits >> (8*j));
    }
    out[16] = 0; out[17] = 0; out[18] = 0x80; out[19] = 0x7f;
}
