/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "menu_model.h"
#include "menu_font.h"
#include "maintenance.h"
#include "zf_common_headfile.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static MenuModel menu;
static MenuPage drawn_page;
static uint32_t last_screen, last_image, last_vofa, start_ms, quiet_since;
static bool vofa_enabled, start_pending, vofa_open_loop;
static char message[27] = "U/D OK; K1 back";
static char row_cache[8][21];
static uint8_t row_style[8];
static bool row_valid[8];
static unsigned grid_selected = 16;
static void clear_screen(void) { tft180_clear(); memset(row_valid, 0, sizeof row_valid); grid_selected = 16; }
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
static bool stationary(const ControlSnapshot *s) { return maintenance_stop_check(s)==MAINT_OK; }
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
static bool save_parameters(const AppParam *candidate,const ControlSnapshot *s) {
    MaintenanceCheck check=maintenance_save_check(s,candidate);
    if(check!=MAINT_OK) {
        note(check==MAINT_MODE?"SAVE: STOP FIRST":check==MAINT_OUTPUT?"SAVE: PWM NOT ZERO":
             check==MAINT_MOTION?"SAVE: WHEELS MOVE":"SAVE: BAD PARAM");
        return false;
    }
    if(board_time_ms()-quiet_since<500) { note("SAVE: WAIT 0.5S"); return false; }
    (void)camera_frame_set_running(false);
    motor_brake_zero(); pit_disable(TIM6_PIT);
    bool ok=param_store_save(candidate);
    if(ok) {
        BoardIrqToken token=board_irq_save(); g_app_param=*candidate; board_irq_restore(token);
    }
    board_clock_init(); (void)side_encoder_init(); app_control_init();
    pit_ms_init(TIM6_PIT,APP_CONTROL_PERIOD_MS);
    /* Keep the current pressed key stable across clock reset: no second save until release. */
    menu.candidate_ms=0; quiet_since=0; last_screen=0; last_vofa=0;
    note(ok?"saved to flash":"FLASH SAVE FAILED");
    return ok;
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
        vofa_enabled = a == MENU_VOFA;
        bool closed_ready=s->feedback.valid && param_valid(&g_app_param) &&
            g_app_param.debug_speed_mps>0 && (g_app_param.calibration_mask&7u)==7u;
        vofa_open_loop=vofa_enabled && !closed_ready;
        app_exchange_request(vofa_enabled && closed_ready?REQ_SPEED_DEBUG:REQ_RUN30);
        note(vofa_open_loop?"VOFA open30: no m/s":vofa_enabled?"VOFA speed PI":"run30: OPEN LOOP");
        break;
    case MENU_CAMERA:
        note(camera_frame_set_running(true) ? "camera starting" : "camera init failed"); break;
    case MENU_STOP: stop_all(s); break;
    case MENU_APPLY:
    case MENU_APPLY_SAVE: {
        AppParam candidate=g_app_param;
        bool save=a==MENU_APPLY_SAVE;
        if(s->mode!=RUN_STATIC_DEBUG || ((!save || menu.input[0]) && !menu_apply_value(&menu,&candidate))) {
            menu.input_error=MENU_INPUT_INVALID; note("bad value / stop"); break;
        }
        if(save) {
            if(!save_parameters(&candidate,s)) { menu.input_error=MENU_INPUT_SAVE_ERROR; break; }
        } else {
            BoardIrqToken token=board_irq_save(); g_app_param=candidate; board_irq_restore(token);
            note("RAM set; save para");
        }
        (void)menu_back(&menu);
        break;
    }
    case MENU_SAVE: (void)save_parameters(&g_app_param,s); break;
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
        line(7, "U/D OK K1:BACK"); return;
    }
    if (menu.page == MENU_RACE_IMAGE || menu.page == MENU_IMAGE) {
        line(1, message);
        if (now - last_image > 1000) line(4, "no fresh frame");
        snprintf(text, sizeof text, "M%u >back OK/K1", (unsigned)s->mode);
        styled_line(7,text,1,true); return;
    }
    if (menu.page != MENU_KEYPAD) line(1, message);
    if (menu.page == MENU_KEYPAD) {
        menu_float_text(a, sizeof a, menu_param_value(&menu, &g_app_param));
        snprintf(text, sizeof text, "old %s", a); line(1, text);
        snprintf(text, sizeof text, "new %s_", menu.input); line(2, text);
        if (menu.input_error) line(1, menu.input_error==MENU_INPUT_SAVE_ERROR?message:"invalid: edit again");
        if (grid_selected != menu.selected) {
            for (unsigned i=0; i<16; i++) {
                if (grid_selected<16 && i!=grid_selected && i!=menu.selected) continue;
                menu_font_cell(4+(i%4)*30,52+(i/4)*18,30,18,
                               menu_label(&menu,i),i==menu.selected);
            }
            grid_selected=menu.selected;
        }
        return;
    } else if (menu.page == MENU_LIVE) {
        snprintf(text,sizeof text,"%s M%u E%u F%u", vofa_enabled ? "VOFA" : "test",
                 (unsigned)s->mode,s->feedback.valid,(unsigned)s->reason); line(0,text);
        if(vofa_enabled && vofa_open_loop) {
            snprintf(text,sizeof text,"L cnt %+ld",(long)s->feedback.delta[0]); line(2,text);
            snprintf(text,sizeof text,"R cnt %+ld",(long)s->feedback.delta[1]); line(3,text);
            line(4,"m/s N/A (open loop)");
        } else {
            menu_float_text(a, sizeof a, s->feedback.speed_mps[0]);
            snprintf(text, sizeof text, "L m/s %s", a); line(2, text);
            menu_float_text(a, sizeof a, s->feedback.speed_mps[1]);
            snprintf(text, sizeof text, "R m/s %s", a); line(3, text);
            menu_float_text(a, sizeof a, s->targets.left_mps);
            menu_float_text(b, sizeof b, s->targets.right_mps);
            snprintf(text, sizeof text, "ref L%s R%s", a, b); line(4, text);
        }
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
    line(7, "U/D OK K1:BACK");
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
