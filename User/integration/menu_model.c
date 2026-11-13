/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "menu_model.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct { const char *name; size_t offset; float min, max; } Field;
#define F(name, member, lo, hi) {name, offsetof(AppParam, member), lo, hi}
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
    F("left FF", ff_left, 0, 10000), F("right FF", ff_right, 0, 10000)
};
static const char *const digits[] = {"1","2","3","DEL","4","5","6","CLR","7","8","9","OK",".","0","","BACK"};
void menu_init(MenuModel *m) { memset(m, 0, sizeof(*m)); m->candidate = m->stable = 7; }
unsigned menu_count(const MenuModel *m) {
    switch (m->page) {
    case MENU_MAIN: return 3;
    case MENU_PARA: return 4;
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
        "control","number input","start / camera","image","motor test"};
    return m->page == MENU_KEYPAD ? fields[m->field].name : titles[m->page];
}
const char *menu_label(const MenuModel *m, unsigned i) {
    static const char *const main_items[] = {"start","para","control"};
    static const char *const para_items[] = {"vision","control","save to flash","back"};
    static const char *const control_items[] = {"run 30","image","vofa","VOFA target m/s","stop","back"};
    if (i >= menu_count(m)) return "";
    switch (m->page) {
    case MENU_MAIN: return main_items[i];
    case MENU_PARA: return para_items[i];
    case MENU_CONTROL: return control_items[i];
    case MENU_KEYPAD: return digits[i];
    case MENU_VISION: return i == 3 ? "back" : fields[i].name;
    case MENU_PARAMS: return i == 12 ? "back" : fields[i + 3].name;
    case MENU_LIVE: return i ? "back" : "stop";
    default: return "back";
    }
}
static void enter(MenuModel *m, MenuPage p) { m->page = p; m->selected = 0; }
static void edit(MenuModel *m, unsigned field) {
    m->parent = m->page; m->field = field; m->input[0] = 0; m->input_error = false;
    enter(m, MENU_KEYPAD);
}
MenuAction menu_confirm(MenuModel *m) {
    unsigned i = m->selected;
    switch (m->page) {
    case MENU_MAIN:
        if (i == 0) { enter(m, MENU_RACE_IMAGE); return MENU_START; }
        enter(m, i == 1 ? MENU_PARA : MENU_CONTROL); break;
    case MENU_PARA:
        if (i < 2) enter(m, i ? MENU_PARAMS : MENU_VISION);
        else if (i == 2) return MENU_SAVE;
        else enter(m, MENU_MAIN);
        break;
    case MENU_VISION:
    case MENU_PARAMS:
        if (i == menu_count(m) - 1) enter(m, MENU_PARA);
        else edit(m, i + (m->page == MENU_PARAMS ? 3 : 0));
        break;
    case MENU_CONTROL:
        if (i == 0 || i == 2) { enter(m, MENU_LIVE); return i ? MENU_VOFA : MENU_RUN30; }
        if (i == 1) { enter(m, MENU_IMAGE); return MENU_CAMERA; }
        if (i == 3) edit(m, 9);
        else { if (i == 5) enter(m, MENU_MAIN); return MENU_STOP; }
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
        else if (i == 15) enter(m, m->parent);
        break;
    }
    case MENU_RACE_IMAGE: enter(m, MENU_MAIN); return MENU_STOP;
    case MENU_IMAGE: case MENU_LIVE: enter(m, MENU_CONTROL); return MENU_STOP;
    }
    return MENU_NONE;
}
MenuAction menu_key(MenuModel *m, uint8_t code, uint32_t now) {
    bool keypad = m->page == MENU_KEYPAD;
    /* 74HC148: up=4 down=5 left=6 right=2 center=1; none=7. */
    if (keypad) code = (code==1 || code==2 || code==4 || code==5 || code==6) ? code : 7;
    else code = code == 3 ? 3 : 7;
    if (code != m->candidate) { m->candidate = code; m->candidate_ms = now; }
    if (now - m->candidate_ms >= 40 && m->stable != code) {
        m->stable = code;
        if (keypad) {
            if (code == 7) return MENU_NONE;
            m->held = true; /* consume release even if center leaves the keypad */
            if (code == 1) return menu_confirm(m);
            int dx = code==2 ? 1 : code==6 ? -1 : 0;
            int dy = code==5 ? 1 : code==4 ? -1 : 0;
            int x=(int)(m->selected%4), y=(int)(m->selected/4);
            for (unsigned skip=0; skip<2; skip++) {
                x+=dx; y+=dy;
                if (x<0 || x>3 || y<0 || y>3) break;
                unsigned next=(unsigned)(y*4+x);
                if (next!=14) { m->selected=next; break; }
            }
            return MENU_NONE;
        }
        if (code == 3) { m->pressed_ms = now; m->held = false; }
        else if (!m->held) { m->selected = (m->selected + 1) % menu_count(m); }
    }
    if (!keypad && m->stable == 3 && m->candidate == 3 && !m->held && now - m->pressed_ms >= 800) {
        m->held = true;
        return menu_confirm(m);
    }
    return MENU_NONE;
}
float menu_param_value(const MenuModel *m, const AppParam *p) {
    float value; memcpy(&value, (const char *)p + fields[m->field].offset, sizeof value); return value;
}
bool menu_apply_value(const MenuModel *m, AppParam *p) {
    char *end;
    if (!m->input[0]) return false;
    float value = strtof(m->input, &end);
    const Field *f = &fields[m->field];
    if (*end || !isfinite(value) || value < f->min || value > f->max) return false;
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
    if (!isfinite(value) || fabsf(value) > 100000) { snprintf(out, size, "invalid"); return; }
    long milli = (long)(fabsf(value) * 1000 + .5f);
    snprintf(out, size, "%s%ld.%03ld", value < 0 ? "-" : "", milli / 1000, milli % 1000);
}
void vofa_pack(const ControlSnapshot *s, uint8_t out[20]) {
    _Static_assert(sizeof(float) == 4, "float32 required");
    float values[4] = {s->targets.left_mps, s->feedback.speed_mps[0],
                       s->targets.right_mps, s->feedback.speed_mps[1]};
    for (unsigned i = 0; i < 4; i++) {
        uint32_t bits; memcpy(&bits, &values[i], 4);
        for (unsigned j = 0; j < 4; j++) out[4*i+j] = (uint8_t)(bits >> (8*j));
    }
    out[16] = 0; out[17] = 0; out[18] = 0x80; out[19] = 0x7f;
}
