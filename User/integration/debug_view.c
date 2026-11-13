/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "menu_model.h"
#include "menu_font.h"
#include "zf_common_headfile.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static MenuModel menu;
static MenuPage drawn_page;
static uint32_t last_screen, last_image, last_vofa, start_ms, quiet_since;
static bool vofa_enabled, start_pending;
static char message[27] = "KEY1 click / hold";
static char row_cache[8][21];
static uint8_t row_style[8];
static bool row_valid[8];
static void clear_screen(void) { tft180_clear(); memset(row_valid, 0, sizeof row_valid); }
static void styled_line(unsigned row, const char *text, unsigned scale, bool selected) {
    if (row >= 8 || row*16 + scale*16 > 128) return;
    char clipped[21];
    snprintf(clipped, sizeof clipped, "%.*s", (int)(20/scale), text);
    uint8_t style = (uint8_t)(scale | (selected ? 4 : 0));
    if (row_valid[row] && row_style[row] == style && !strcmp(row_cache[row], clipped)) return;
    menu_font_text(row*16, clipped, scale, selected);
    strcpy(row_cache[row], clipped); row_style[row] = style; row_valid[row] = true;
}
static void line(unsigned row, const char *text) { styled_line(row,text,1,false); }
static void note(const char *text) { snprintf(message, sizeof message, "%.26s", text); }
static bool stationary(const ControlSnapshot *s) {
    return s->mode == RUN_STATIC_DEBUG && s->duty[0] == 0 && s->duty[1] == 0 &&
        s->feedback.delta[0] == 0 && s->feedback.delta[1] == 0 &&
        (!s->feedback.valid || (fabsf(s->feedback.speed_mps[0]) < .01f && fabsf(s->feedback.speed_mps[1]) < .01f));
}
static void stop_all(const ControlSnapshot *s) {
    start_pending = false; vofa_enabled = false;
    app_exchange_request(s->mode == RUN_FAULT ? REQ_RESET : REQ_HALT);
    (void)camera_frame_set_running(false);
    note("motor+camera stopped");
}
void debug_view_init(void) {
    menu_init(&menu);
    drawn_page = (MenuPage)255;
    tft180_init();
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(0x0000, 0xffff);
    clear_screen();
    /* UART3 is kept exclusively binary once VOFA is enabled. */
}
static void action(MenuAction a, const ControlSnapshot *s) {
    switch (a) {
    case MENU_START:
        if (s->mode != RUN_STATIC_DEBUG) { note("stop before start"); break; }
        if (!camera_frame_set_running(true)) { note("camera init failed"); break; }
        start_pending = true; start_ms = board_time_ms();
        note("checking start..."); break;
    case MENU_RUN30:
    case MENU_VOFA:
        if (s->mode != RUN_STATIC_DEBUG) { note("blocked: stop first"); break; }
        if (a == MENU_VOFA && (!s->feedback.valid || !param_valid(&g_app_param) ||
            g_app_param.debug_speed_mps <= 0 ||
            (g_app_param.calibration_mask & 7u) != 7u)) {
            note("set speed / enc CAL"); break;
        }
        vofa_enabled = a == MENU_VOFA;
        app_exchange_request(a == MENU_VOFA ? REQ_SPEED_DEBUG : REQ_RUN30);
        note(a == MENU_VOFA ? "VOFA speed PI" : "run30: OPEN LOOP"); break;
    case MENU_CAMERA:
        note(camera_frame_set_running(true) ? "camera starting" : "camera init failed"); break;
    case MENU_STOP: stop_all(s); break;
    case MENU_APPLY: {
        AppParam candidate = g_app_param;
        if (s->mode != RUN_STATIC_DEBUG || !menu_apply_value(&menu, &candidate)) {
            menu.input_error = true; note("bad value / stop"); break;
        }
        BoardIrqToken token = board_irq_save();
        g_app_param = candidate;
        board_irq_restore(token);
        menu.selected = 13;
        (void)menu_confirm(&menu); /* keypad back -> parent, no motor command */
        note("RAM set; save para"); break;
    }
    case MENU_SAVE: {
        if (!stationary(s) || board_time_ms() - quiet_since < 500 || !param_storable(&g_app_param)) {
            note("save: wait/check"); break; }
        (void)camera_frame_set_running(false);
        motor_brake_zero();
        pit_disable(TIM6_PIT);
        bool ok = param_store_save(&g_app_param);
        /* Flash can stall longer than the 16-bit microsecond clock wrap. */
        board_clock_init();
        (void)side_encoder_init();
        app_control_init();
        pit_ms_init(TIM6_PIT, APP_CONTROL_PERIOD_MS);
        menu.candidate = menu.stable = 3;
        menu.held = true; last_screen = 0; last_vofa = 0;
        note(ok ? "saved to flash" : "FLASH SAVE FAILED"); break;
    }
    default: break;
    }
}
static void render(const ControlSnapshot *s, uint32_t now) {
    if (menu.page != drawn_page) { clear_screen(); drawn_page = menu.page; }
    if (menu.page != MENU_LIVE) line(0, menu_title(&menu));
    char text[40], a[12], b[12];
    if (menu.page == MENU_MAIN) {
        for (unsigned i = 0; i < 3; i++) {
            snprintf(text, sizeof text, "%c%s", i == menu.selected ? '>' : ' ', menu_label(&menu,i));
            styled_line(1+i*2,text,2,i==menu.selected);
        }
        line(7, "K1:next HOLD:ok"); return;
    }
    if (menu.page == MENU_RACE_IMAGE || menu.page == MENU_IMAGE) {
        line(1, message);
        if (now - last_image > 1000) line(4, "no fresh frame");
        snprintf(text, sizeof text, "M%u >back HOLD", (unsigned)s->mode);
        styled_line(7,text,1,true); return;
    }
    if (menu.page != MENU_KEYPAD) line(1, message);
    if (menu.page == MENU_KEYPAD) {
        menu_float_text(a, sizeof a, menu_param_value(&menu, &g_app_param));
        snprintf(text, sizeof text, "old %s", a); line(1, text);
        snprintf(text, sizeof text, "new %s_", menu.input); line(2, text);
        line(3, menu.input_error ? "INVALID: edit/back" : "0..9 . del ok back");
        snprintf(text, sizeof text, "key %u/14", menu.selected+1); line(4,text);
        snprintf(text, sizeof text, "[%s]", menu_label(&menu, menu.selected));
        styled_line(5,text,2,true);
    } else if (menu.page == MENU_LIVE) {
        snprintf(text,sizeof text,"%s M%u E%u F%u", vofa_enabled ? "VOFA" : "test",
                 (unsigned)s->mode,s->feedback.valid,(unsigned)s->reason); line(0,text);
        menu_float_text(a, sizeof a, s->feedback.speed_mps[0]);
        snprintf(text, sizeof text, "L m/s %s", a); line(2, text);
        menu_float_text(a, sizeof a, s->feedback.speed_mps[1]);
        snprintf(text, sizeof text, "R m/s %s", a); line(3, text);
        menu_float_text(a, sizeof a, s->targets.left_mps);
        menu_float_text(b, sizeof b, s->targets.right_mps);
        snprintf(text, sizeof text, "ref L%s R%s", a, b); line(4, text);
        snprintf(text, sizeof text, "PWM L%ld%% R%ld%%", (long)(s->duty[0]*100), (long)(s->duty[1]*100)); line(5, text);
        styled_line(6, menu.selected == 0 ? ">stop" : " stop",1,menu.selected==0);
        styled_line(7, menu.selected == 1 ? ">back" : " back",1,menu.selected==1);
        return;
    } else {
        unsigned first = menu.selected / 5 * 5;
        for (unsigned row = 0; row < 5; row++) {
            unsigned i = first + row;
            snprintf(text, sizeof text, "%c%s", i == menu.selected ? '>' : ' ', menu_label(&menu, i));
            styled_line(row + 2,text,1,i==menu.selected);
        }
    }
    line(7, "K1:next HOLD:ok");
}
void debug_view_poll(const ControlSnapshot *s) {
    uint32_t now = board_time_ms();
    if (!stationary(s)) quiet_since = now;
    MenuPage before = menu.page;
    MenuAction a = menu_key(&menu, board_key_code(), now);
    if (before == MENU_MAIN && menu.page == MENU_PARA && s->mode != RUN_STATIC_DEBUG) {
        menu.page = MENU_MAIN; menu.selected = 1; note("stop before para");
    }
    action(a, s);
    if (start_pending) {
        if (s->mode == RUN_RUNNING) { start_pending = false; note("race running"); }
        else if (s->mode == RUN_FAULT || now - start_ms > 2500) {
            start_pending = false; note("blocked: CAL/vision");
            app_exchange_request(REQ_HALT);
        } else if (s->mode == RUN_STATIC_DEBUG && s->config_ok) {
            app_exchange_request(REQ_READY);
        } else if (s->mode == RUN_READY) app_exchange_request(REQ_START);
    }
    if (vofa_enabled && now - last_vofa >= 20) {
        last_vofa = now;
        uint8_t packet[20]; vofa_pack(s, packet);
        uart_write_buffer(UART_3, packet, sizeof packet);
    }
    if (now - last_screen >= 100 || menu.page != drawn_page) {
        last_screen = now; render(s, now);
    }
}
void debug_view_frame(const FrameView *f, const ControlSnapshot *s, const VisionFeatures *v) {
    (void)s; (void)v;
    uint32_t now = board_time_ms();
    if ((menu.page == MENU_RACE_IMAGE || menu.page == MENU_IMAGE) && now - last_image >= 200) {
        last_image = now;
        if (menu.page != drawn_page) { clear_screen(); drawn_page = menu.page; }
        tft180_show_gray_image(0, 32, f->pixels, f->width, f->height, 160, 80, 0);
        row_valid[4] = false;
    }
}
