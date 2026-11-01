/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "zf_common_headfile.h"
#include <math.h>
#include <stdio.h>
static uint32_t candidate_ms, last_screen, last_log;
static uint8_t candidate = 7, stable = 7;
static bool saved;
static bool image_page;
static uint32_t last_image;
void debug_view_init(void) {
    tft180_init();
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_set_font(TFT180_6X8_FONT);
    tft180_clear();
    printf("\r\nsecond-test v" APP_VERSION ": STATIC; calibrate before READY\r\n");
}
static void line(unsigned row, const char *message) {
    char padded[27];
    snprintf(padded, sizeof(padded), "%-25.25s", message);
    tft180_show_string(0, (uint16_t)(row * 16), padded);
}
void debug_view_poll(const ControlSnapshot *s) {
    uint32_t now = board_time_ms();
    uint8_t key = board_key_code();
    if (key != candidate) {
        candidate = key;
        candidate_ms = now;
        saved = false;
    }
    if (now - candidate_ms >= 40 && stable != candidate) {
        stable = candidate;
        if (stable == 0)
            app_exchange_request(s->mode == RUN_FAULT ? REQ_RESET : REQ_STOP);
        else if (stable == 1 && s->mode == RUN_STATIC_DEBUG)
            app_exchange_request(REQ_READY);
        else if (stable == 3 && s->mode == RUN_READY)
            app_exchange_request(REQ_START);
        else if (stable == 2 && s->mode == RUN_STATIC_DEBUG) {
            image_page = !image_page;
            tft180_clear();
        }
    }
    if (stable == 5 && now - candidate_ms > 2000 && !saved && s->mode == RUN_STATIC_DEBUG &&
        s->feedback.valid && fabsf(s->feedback.speed_mps[0]) < .01f &&
        fabsf(s->feedback.speed_mps[1]) < .01f && param_valid(&g_app_param)) {
        saved = true;
        motor_brake_zero();
        pit_disable(TIM6_PIT);
        (void)board_camera_hold_after_frame_isr();
        bool ok = param_store_save(&g_app_param);
        printf("PARAM save=%d; reboot\r\n", ok);
        NVIC_SystemReset();
    }
    if (now - last_screen >= 100 && (!image_page || s->mode != RUN_STATIC_DEBUG)) {
        last_screen = now;
        char text[40];
        snprintf(text, sizeof(text), "Mode %u stop %u", (unsigned)s->mode, (unsigned)s->reason);
        line(0, text);
        snprintf(text, sizeof(text), "CAM %u CAL %u", s->camera_ok, s->config_ok);
        line(1, text);
        snprintf(text, sizeof(text), "L %+ld R %+ld", (long)s->feedback.delta[0],
                 (long)s->feedback.delta[1]);
        line(2, text);
        snprintf(text, sizeof(text), "cm/s L%ld R%ld", (long)(s->feedback.speed_mps[0] * 100),
                 (long)(s->feedback.speed_mps[1] * 100));
        line(3, text);
        snprintf(text, sizeof(text), "EN %% L%ld R%ld", (long)(s->duty[0] * 100),
                 (long)(s->duty[1] * 100));
        line(4, text);
        snprintf(text, sizeof(text), "age %lu ms", (unsigned long)s->vision_age_ms);
        line(5, text);
        line(6, "MID ready K1 go K2 stop");
    }
    if (s->mode != RUN_RUNNING && s->mode != RUN_STOPPING && now - last_log >= 2000) {
        last_log = now;
        printf("mode=%u camera=%u cal=%u dL=%ld dR=%ld\r\n", (unsigned)s->mode, s->camera_ok,
               s->config_ok, (long)s->feedback.delta[0], (long)s->feedback.delta[1]);
    }
}
void debug_view_frame(const FrameView *f, const ControlSnapshot *s, const VisionFeatures *v) {
    uint32_t now = board_time_ms();
    if (image_page && s->mode == RUN_STATIC_DEBUG && now - last_image >= 200) {
        last_image = now;
        tft180_show_gray_image(0, 0, f->pixels, f->width, f->height, 160, 100,
                               v->boundary.threshold);
        tft180_show_string(0, 104, "R: return, binary view");
    }
}
