/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include <math.h>
#include <string.h>
static uint16_t previous[2];
static SideFeedback feedback;
bool side_encoder_init(void) {
    board_encoder_init();
    memset(&feedback, 0, sizeof(feedback));
    for (unsigned i = 0; i < 2; i++)
        previous[i] = board_encoder_raw16((SideId)i);
    return true;
}
bool side_encoder_sample(float dt, uint32_t now, SideFeedback *out) {
    feedback.valid = param_valid(&g_app_param) && isfinite(dt) && dt > 0 && dt <= .05f;
    for (unsigned i = 0; i < 2; i++) {
        uint16_t raw = board_encoder_raw16((SideId)i);
        int32_t d = app_count_delta(raw, previous[i]);
        previous[i] = raw;
        d *= g_app_param.encoder_sign[i];
        feedback.delta[i] = d;
        feedback.count[i] += d;
        if (feedback.valid) {
            float diameter = i ? g_app_param.wheel_right_m : g_app_param.wheel_left_m;
            float cpr = i ? g_app_param.counts_right : g_app_param.counts_left;
            float distance = 3.14159265358979323846f * diameter * (float)d / cpr;
            float speed = distance / dt;
            if (!isfinite(speed) || fabsf(speed) > g_app_param.speed_limit_mps * 4 + 1) {
                feedback.valid = false;
                continue;
            }
            feedback.speed_mps[i] +=
                dt / (g_app_param.encoder_tau_s + dt) * (speed - feedback.speed_mps[i]);
            feedback.rolling_m[i] += distance;
        }
    }
    feedback.stamp_ms = now;
    *out = feedback;
    return out->valid;
}
