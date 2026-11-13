/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "menu_model.h"
#include "zf_common_headfile.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static MenuModel menu;
static MenuPage drawn_page;
static uint32_t last_screen, last_image, last_vofa, start_ms, quiet_since;
static bool vofa_enabled, start_pending;
static char message[27] = "KEY1 click / hold";
static void line(unsigned row, const char *text) {
    char padded[27];
    snprintf(padded, sizeof padded, "%-26.26s", text);
    tft180_show_string(0, (uint16_t)(row * 12), padded);
}
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
    note("stopped; camera paused");
}
void debug_view_init(void) {
    menu_init(&menu);
    drawn_page = (MenuPage)255;
    tft180_init();
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_set_font(TFT180_6X8_FONT);
    tft180_clear();
    /* UART3 is kept exclusively binary once VOFA is enabled. */
}
static void action(MenuAction a, const ControlSnapshot *s) {
    switch (a) {
    case MENU_START:
        if (s->mode != RUN_STATIC_DEBUG) { note("start blocked: stop first"); break; }
        if (!camera_frame_set_running(true)) { note("camera init failed"); break; }
        start_pending = true; start_ms = board_time_ms();
        note("checking race conditions"); break;
    case MENU_RUN30:
    case MENU_VOFA:
        if (s->mode != RUN_STATIC_DEBUG) { note("blocked: stop first"); break; }
        if (a == MENU_VOFA && (!s->feedback.valid || !param_valid(&g_app_param) ||
            g_app_param.debug_speed_mps <= 0 ||
            (g_app_param.calibration_mask & 7u) != 7u)) {
            note("set speed / encoder CAL"); break;
        }
        vofa_enabled = a == MENU_VOFA;
        app_exchange_request(a == MENU_VOFA ? REQ_SPEED_DEBUG : REQ_RUN30);
        note(a == MENU_VOFA ? "VOFA speed PI" : "run 30: OPEN LOOP"); break;
    case MENU_CAMERA:
        note(camera_frame_set_running(true) ? "camera starting" : "camera init failed"); break;
    case MENU_STOP: stop_all(s); break;
    case MENU_APPLY: {
        AppParam candidate = g_app_param;
        if (s->mode != RUN_STATIC_DEBUG || !menu_apply_value(&menu, &candidate)) {
            menu.input_error = true; note("invalid / stop first"); break;
        }
        BoardIrqToken token = board_irq_save();
        g_app_param = candidate;
        board_irq_restore(token);
        menu.selected = 13;
        (void)menu_confirm(&menu); /* keypad back -> parent, no motor command */
        note("applied RAM; save in para"); break;
    }
    case MENU_SAVE: {
        if (!stationary(s) || board_time_ms() - quiet_since < 500 || !param_storable(&g_app_param)) {
            note("save blocked: wait / value"); break; }
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
        note(ok ? "saved; survives reset" : "FLASH SAVE FAILED"); break;
    }
    default: break;
    }
}
static void render(const ControlSnapshot *s, uint32_t now) {
    if (menu.page != drawn_page) { tft180_clear(); drawn_page = menu.page; }
    line(0, menu_title(&menu));
    char text[40], a[12], b[12];
    if (menu.page == MENU_RACE_IMAGE || menu.page == MENU_IMAGE) {
        line(1, message);
        if (now - last_image > 1000) line(4, "no fresh camera frame");
        snprintf(text, sizeof text, "state %u >back HOLD", (unsigned)s->mode);
        line(9, text); return;
    }
    line(1, message);
    if (menu.page == MENU_KEYPAD) {
        menu_float_text(a, sizeof a, menu_param_value(&menu, &g_app_param));
        snprintf(text, sizeof text, "old %s", a); line(2, text);
        snprintf(text, sizeof text, "new %s_", menu.input); line(3, text);
        line(4, menu.input_error ? "INVALID: edit or back" : "0..9 . del ok back");
        snprintf(text, sizeof text, "> [%s]", menu_label(&menu, menu.selected)); line(6, text);
    } else if (menu.page == MENU_LIVE) {
        menu_float_text(a, sizeof a, s->feedback.speed_mps[0]);
        menu_float_text(b, sizeof b, s->feedback.speed_mps[1]);
        snprintf(text, sizeof text, "m/s L%s R%s", a, b); line(2, text);
        menu_float_text(a, sizeof a, s->targets.left_mps);
        menu_float_text(b, sizeof b, s->targets.right_mps);
        snprintf(text, sizeof text, "ref L%s R%s", a, b); line(3, text);
        snprintf(text, sizeof text, "EN L%ld%% R%ld%%", (long)(s->duty[0]*100), (long)(s->duty[1]*100)); line(4, text);
        snprintf(text, sizeof text, "mode %u fault %u enc %u", (unsigned)s->mode, (unsigned)s->reason, s->feedback.valid); line(5, text);
        line(6, menu.selected == 0 ? ">stop" : " stop");
        line(7, menu.selected == 1 ? ">back" : " back");
    } else {
        unsigned first = menu.selected / 6 * 6;
        for (unsigned row = 0; row < 6; row++) {
            unsigned i = first + row;
            snprintf(text, sizeof text, "%c%s", i == menu.selected ? '>' : ' ', menu_label(&menu, i));
            line(row + 2, text);
        }
    }
    line(9, "KEY1 click=next hold=OK");
}
void debug_view_poll(const ControlSnapshot *s) {
    uint32_t now = board_time_ms();
    if (!stationary(s)) quiet_since = now;
    MenuPage before = menu.page;
    MenuAction a = menu_key(&menu, board_key_code(), now);
    if (before == MENU_MAIN && menu.page == MENU_PARA && s->mode != RUN_STATIC_DEBUG) {
        menu.page = MENU_MAIN; menu.selected = 1; note("stop before parameters");
    }
    action(a, s);
    if (start_pending) {
        if (s->mode == RUN_RUNNING) { start_pending = false; note("race running"); }
        else if (s->mode == RUN_FAULT || now - start_ms > 2500) {
            start_pending = false; note("start blocked: CAL/vision");
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
        if (menu.page != drawn_page) { tft180_clear(); drawn_page = menu.page; }
        tft180_show_gray_image(0, 24, f->pixels, f->width, f->height, 160, 80, 0);
    }
}
