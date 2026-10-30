/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "track_profile.h"
#include "vehicle_calibration.h"
#include <math.h>
#include <string.h>
static ControlSnapshot status;
static MotionRequest good;
static SpeedPiState pi[2];
static MotionPlanState plan;
static uint32_t previous_us;
static volatile bool initialized, camera_ok;
void app_control_camera_ready(bool ok) {
    BoardIrqToken t = board_irq_save();
    camera_ok = ok;
    board_irq_restore(t);
}
void app_control_init(void) {
    memset(&status, 0, sizeof(status));
    status.mode = RUN_STATIC_DEBUG;
    status.session_id = 1;
    memset(&good, 0, sizeof(good));
    memset(pi, 0, sizeof(pi));
    memset(&plan, 0, sizeof(plan));
    previous_us = board_time_us();
    initialized = true;
    app_exchange_publish_control(&status);
}
static void fault(StopReason r) {
    status.mode = RUN_FAULT;
    status.reason = r;
    motor_brake_zero();
    memset(pi, 0, sizeof(pi));
    memset(&plan, 0, sizeof(plan));
    memset(&status.targets, 0, sizeof(status.targets));
    status.duty[0] = status.duty[1] = 0;
}
void app_control_tick_isr(void) {
    board_clock_tick();
    if (!initialized)
        return;
    uint32_t start = board_time_us(), now = board_time_ms(), elapsed = start - previous_us;
    previous_us = start;
    float dt = (float)elapsed * .000001f;
    status.stamp_ms = now;
    status.camera_ok = camera_ok;
    status.config_ok = param_can_arm(&g_app_param, CAL_FULL_RACE != 0) && camera_ok &&
                       (!CAL_FULL_RACE || (APP_ROUTE_LENGTH > 0 && APP_FINISH_DISTANCE_M > 0));
    if (elapsed == 0 || elapsed > 20000) {
        if (status.mode == RUN_RUNNING || status.mode == RUN_STOPPING)
            fault(STOP_TIMING);
        dt = .005f;
    }
    (void)side_encoder_sample(dt, now, &status.feedback);
    MotionRequest incoming;
    app_exchange_read_motion(&incoming);
    if (incoming.session_id == status.session_id && incoming.valid && incoming.confidence >= 40) {
        if (!isfinite(incoming.omega_raw) || !isfinite(incoming.speed_cap_mps)) {
            if (status.mode == RUN_RUNNING)
                fault(STOP_NONFINITE);
        } else if ((uint32_t)(now - incoming.capture_ms) <= (uint32_t)g_app_param.hard_timeout_ms)
            good = incoming;
    }
    status.vision_age_ms = good.valid ? now - good.capture_ms : UINT32_MAX;
    bool fresh = good.valid && good.session_id == status.session_id &&
                 status.vision_age_ms <= (uint32_t)g_app_param.soft_timeout_ms &&
                 status.feedback.valid;
    uint32_t requests = app_exchange_take_requests();
    if (run_manager_step(&status.mode, &status.reason, requests,
                         status.config_ok && status.feedback.valid &&
                             fabsf(status.feedback.speed_mps[0]) < .03f &&
                             fabsf(status.feedback.speed_mps[1]) < .03f,
                         fresh, &status.session_id)) {
        memset(pi, 0, sizeof(pi));
        memset(&plan, 0, sizeof(plan));
        if (status.mode == RUN_READY || status.mode == RUN_STATIC_DEBUG)
            memset(&good, 0, sizeof(good));
    }
    if (status.mode == RUN_RUNNING || status.mode == RUN_STOPPING) {
        if (!status.feedback.valid)
            fault(STOP_ENCODER);
        else if (!good.valid || good.session_id != status.session_id ||
                 status.vision_age_ms > (uint32_t)g_app_param.hard_timeout_ms)
            fault(STOP_VISION);
        else if (incoming.session_id == status.session_id && incoming.stop_requested) {
            status.mode = RUN_STOPPING;
            status.reason = STOP_TRACK;
        }
        if (status.mode == RUN_RUNNING || status.mode == RUN_STOPPING) {
            bool stopping = status.mode == RUN_STOPPING;
            float speed = good.speed_cap_mps, omega = good.omega_raw;
            if (status.vision_age_ms > (uint32_t)g_app_param.soft_timeout_ms) {
                speed = 0;
                stopping = true;
            }
            bool ok =
                motion_plan_step(&plan, &g_app_param, speed, omega, dt, stopping, &status.targets);
            if (ok)
                ok = speed_pi_step(&pi[0], plan.left, status.feedback.speed_mps[0], dt,
                                   g_app_param.kp_left, g_app_param.ki_left, g_app_param.ff_left,
                                   g_app_param.integral_limit, g_app_param.duty_limit,
                                   &status.duty[0]);
            if (ok)
                ok = speed_pi_step(&pi[1], plan.right, status.feedback.speed_mps[1], dt,
                                   g_app_param.kp_right, g_app_param.ki_right, g_app_param.ff_right,
                                   g_app_param.integral_limit, g_app_param.duty_limit,
                                   &status.duty[1]);
            if (!ok)
                fault(STOP_NONFINITE);
            else if (stopping && plan.left < .001f && plan.right < .001f) {
                motor_brake_zero();
                status.duty[0] = status.duty[1] = 0;
                memset(pi, 0, sizeof(pi));
                if (status.mode == RUN_STOPPING) {
                    status.mode = RUN_STATIC_DEBUG;
                    status.session_id++;
                    memset(&good, 0, sizeof(good));
                }
            } else
                motor_apply_forward(status.duty[0], status.duty[1]);
        }
    } else {
        motor_brake_zero();
        status.duty[0] = status.duty[1] = 0;
        memset(pi, 0, sizeof(pi));
        memset(&plan, 0, sizeof(plan));
    }
    if (board_time_us() - start > 4000) {
        status.overruns++;
        if (status.overruns >= 3 && status.mode == RUN_RUNNING)
            fault(STOP_TIMING);
    }
    app_exchange_publish_control(&status);
}
