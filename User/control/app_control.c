/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "track_profile.h"
#include "vehicle_calibration.h"
#include "yaw_control.h"
#include <math.h>
#include <string.h>
static uint32_t debug_duty_ticks;
static ControlSnapshot status;
static MotionRequest good;
static SpeedPiState pi[2];
static MotionPlanState plan;
static uint32_t previous_us, tune_since;
static uint32_t tune_retry_last_frame;
static unsigned tune_retry_frames;
static bool tune_continuous;
static enum { SEARCH_OFF, SEARCH_WAITING, SEARCH_MOVING } tune_search;
static float tune_search_origin_m[2];
static volatile bool initialized, camera_ok;
static GyroMeasurement gyro_latest;
static float yaw_integral;
static GyroCornerTurn gyro_corner;
void app_control_publish_gyro(const GyroMeasurement *measurement) {
    BoardIrqToken token = board_irq_save();
    gyro_latest = *measurement;
    board_irq_restore(token);
}
void app_control_camera_ready(bool ok) {
    BoardIrqToken t = board_irq_save();
    camera_ok = ok;
    board_irq_restore(t);
}
void app_control_init(void) {
    debug_duty_ticks = 3000;
    memset(&status, 0, sizeof(status));
    status.mode = RUN_STATIC_DEBUG;
    status.session_id = 1;
    memset(&good, 0, sizeof(good));
    memset(pi, 0, sizeof(pi));
    memset(&plan, 0, sizeof(plan));
    yaw_integral = 0;
    gyro_corner = (GyroCornerTurn){0};
    previous_us = board_time_us();
    initialized = true;
    app_exchange_publish_control(&status);
}
static void fault(StopReason r) {
    status.mode = RUN_FAULT;
    status.reason = r;
    status.tune_remaining_ms = 0;
    yaw_integral = 0;
    motor_brake_zero();
    memset(pi, 0, sizeof(pi));
    memset(&plan, 0, sizeof(plan));
    memset(&status.targets, 0, sizeof(status.targets));
    status.duty[0] = status.duty[1] = 0;
}
static void tune_wait(uint32_t now, StopReason reason, bool search) {
    status.mode = RUN_TUNE_WAIT;
    status.reason = reason;
    tune_since = now;
    tune_search = search ? SEARCH_WAITING : SEARCH_OFF;
    memcpy(tune_search_origin_m, status.feedback.rolling_m, sizeof tune_search_origin_m);
    tune_retry_frames = 0;
    status.session_id++;
    memset(&good, 0, sizeof(good));
    status.vision_age_ms = UINT32_MAX;
    yaw_integral = 0;
}
static void tune_search_step(uint32_t now, bool recent_frame, bool path_valid, bool stopped) {
    if (tune_search == SEARCH_OFF)
        return;
    float left_m = fabsf(status.feedback.rolling_m[0] - tune_search_origin_m[0]);
    float right_m = fabsf(status.feedback.rolling_m[1] - tune_search_origin_m[1]);
    if (!(left_m < APP_START_SEARCH_DISTANCE_M && right_m < APP_START_SEARCH_DISTANCE_M) ||
        now - tune_since >= APP_START_SEARCH_MAX_MS ||
        (tune_search == SEARCH_MOVING && !recent_frame)) {
        tune_search = SEARCH_OFF;
    } else if (tune_search == SEARCH_WAITING && stopped && recent_frame && !path_valid) {
        tune_search = SEARCH_MOVING;
    }
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
    status.gyro = gyro_latest;
    if (status.gyro.status == GYRO_READY && now - status.gyro.stamp_ms > APP_GYRO_TIMEOUT_MS)
        status.gyro.status = GYRO_STALE;
    bool gyro_enabled = (g_app_param.flags & APP_FLAG_GYRO_LOOP) != 0;
    bool gyro_ready = status.gyro.status == GYRO_READY &&
                      isfinite(status.gyro.rate_radps) && isfinite(status.gyro.angle_rad);
    status.omega_target_radps = status.omega_command_radps = 0;
    if (!gyro_enabled || (status.mode != RUN_RUNNING && status.mode != RUN_TUNE_RUNNING)) {
        yaw_integral = 0;
        gyro_corner = (GyroCornerTurn){0};
    }
    status.config_ok = param_can_arm(&g_app_param, CAL_FULL_RACE != 0) && camera_ok &&
                       (!gyro_enabled || gyro_ready) && CAL_VISION_V2_VALIDATED &&
                       !CAL_FULL_RACE;
    bool timing_ok = elapsed != 0 && elapsed <= 20000;
    if (!timing_ok) {
        if (status.mode == RUN_RUNNING || status.mode == RUN_STOPPING ||
            status.mode == RUN_DUTY_DEBUG || status.mode == RUN_SPEED_DEBUG ||
            status.mode == RUN_TUNE_RUNNING ||
            (status.mode == RUN_TUNE_WAIT && tune_search == SEARCH_MOVING))
            fault(STOP_TIMING);
        dt = .005f;
    }
    (void)side_encoder_sample(dt, now, &status.feedback);
    MotionRequest incoming;
    app_exchange_read_motion(&incoming);
    if (incoming.session_id == status.session_id && incoming.valid && incoming.confidence >= 40) {
        if (!isfinite(incoming.omega_raw) || !isfinite(incoming.speed_cap_mps)) {
            if (status.mode == RUN_RUNNING || status.mode == RUN_TUNE_RUNNING)
                fault(STOP_NONFINITE);
        } else if ((uint32_t)(now - incoming.capture_ms) <= (uint32_t)g_app_param.hard_timeout_ms)
            good = incoming;
    }
    status.vision_age_ms = good.valid ? now - good.capture_ms : UINT32_MAX;
    bool fresh = good.valid && good.session_id == status.session_id &&
                 status.vision_age_ms <= (uint32_t)g_app_param.soft_timeout_ms &&
                 status.feedback.valid;
    uint32_t requests = app_exchange_take_requests();
    if (requests & REQ_HALT) {
        motor_brake_zero();
        memset(pi, 0, sizeof(pi)); memset(&plan, 0, sizeof(plan));
        memset(&good, 0, sizeof(good)); memset(&status.targets, 0, sizeof(status.targets));
        status.duty[0] = status.duty[1] = 0;
        if (status.mode != RUN_FAULT) status.mode = RUN_STATIC_DEBUG;
        status.reason = STOP_LOCAL;
        status.session_id++;
        requests = 0;
    } else if (status.mode == RUN_STATIC_DEBUG && (requests & (REQ_TRACK_TUNE | REQ_TRACK_START))) {
        tune_continuous = (requests & REQ_TRACK_START) != 0;
        tune_wait(now, STOP_NONE, false);
        if (!camera_ok || !param_valid(&g_app_param)) fault(STOP_CONFIG);
        else if (gyro_enabled && !gyro_ready) fault(STOP_GYRO);
        else if (!status.feedback.valid || !(fabsf(status.feedback.speed_mps[0]) < .03f) ||
                 !(fabsf(status.feedback.speed_mps[1]) < .03f)) fault(STOP_ENCODER);
        requests = 0;
    } else if (status.mode == RUN_STATIC_DEBUG && (requests & (REQ_RUN30 | REQ_RUN5 | REQ_SPEED_DEBUG))) {
        bool speed_test = (requests & REQ_SPEED_DEBUG) != 0;
        if (!speed_test || (status.feedback.valid && param_valid(&g_app_param) && g_app_param.debug_speed_mps > 0)) {
            debug_duty_ticks = (requests & REQ_RUN5) ? 500 : 3000;
            status.mode = speed_test ? RUN_SPEED_DEBUG : RUN_DUTY_DEBUG;
            status.reason = STOP_NONE;
            memset(pi, 0, sizeof(pi)); memset(&plan, 0, sizeof(plan));
            memset(&status.targets, 0, sizeof(status.targets));
            status.session_id++;
        }
        requests = 0;
    }
    if (status.mode == RUN_READY && gyro_enabled && !gyro_ready)
        fault(STOP_GYRO);
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
    status.tune_remaining_ms = 0;
    if (status.mode == RUN_TUNE_WAIT || status.mode == RUN_TUNE_RUNNING) {
        uint32_t duration_ms = (uint32_t)(g_app_param.tune_duration_s * 1000.0f + .5f);
        bool tune_fresh = fresh && good.session_id == status.session_id &&
                          status.vision_age_ms <= APP_TUNE_FRAME_AGE_MS;
        bool current = incoming.session_id == status.session_id;
        bool usable = current && incoming.valid && incoming.confidence >= 40;
        if ((requests & REQ_STOP) || (current && incoming.stop_requested) ||
            (status.mode == RUN_TUNE_RUNNING && !tune_continuous && now - tune_since >= duration_ms)) {
            status.mode = RUN_STATIC_DEBUG;
            status.reason = (requests & REQ_STOP) ? STOP_LOCAL :
                            (current && incoming.stop_requested) ? STOP_TRACK : STOP_TUNE_DONE;
            status.session_id++;
            memset(&good, 0, sizeof(good));
        } else if (gyro_enabled && !gyro_ready) fault(STOP_GYRO);
        else if (!camera_ok) fault(STOP_CONFIG);
        else if (!status.feedback.valid) fault(STOP_ENCODER);
        else if (status.mode == RUN_TUNE_WAIT) {
            bool retry = status.reason == STOP_VISION;
            bool stopped = fabsf(status.feedback.speed_mps[0]) < .03f &&
                           fabsf(status.feedback.speed_mps[1]) < .03f;
            bool can_start = tune_fresh && usable && incoming.frame_id == good.frame_id &&
                             isfinite(incoming.omega_raw) && isfinite(incoming.speed_cap_mps) &&
                             good.speed_cap_mps > 0;
            bool search_frame = timing_ok && current && now - incoming.capture_ms <= APP_TUNE_FRAME_AGE_MS &&
                                (!incoming.valid || can_start);
            tune_search_step(now, search_frame, incoming.valid, stopped);
            can_start = can_start && (stopped || tune_search == SEARCH_MOVING);
            if (retry) {
                if (!can_start)
                    tune_retry_frames = 0;
                else if (!tune_retry_frames || incoming.frame_id != tune_retry_last_frame) {
                    tune_retry_last_frame = incoming.frame_id;
                    tune_retry_frames++;
                }
                can_start = can_start && tune_retry_frames >= APP_TUNE_RETRY_FRAMES;
            }
            if (can_start) {
                status.mode = RUN_TUNE_RUNNING;
                status.reason = STOP_NONE;
                tune_since = now;
            } else if (!retry && now - tune_since >= APP_TUNE_WAIT_MS)
                tune_wait(now, STOP_VISION, false);
        } else if (!tune_fresh || !usable)
            tune_wait(now, STOP_VISION, tune_continuous);
        if (status.mode == RUN_TUNE_RUNNING && !tune_continuous)
            status.tune_remaining_ms = duration_ms - (now - tune_since);
    }
    bool searching = status.mode == RUN_TUNE_WAIT && tune_search == SEARCH_MOVING;
    if (status.mode == RUN_DUTY_DEBUG) {
        /* Explicit open-loop diagnostic command, independent of racing duty_limit. */
        status.duty[0] = status.duty[1] = debug_duty_ticks / 10000.0f;
        board_motor_write_en(SIDE_LEFT, debug_duty_ticks);
        board_motor_write_en(SIDE_RIGHT, debug_duty_ticks);
    } else if (status.mode == RUN_SPEED_DEBUG || searching) {
        if (!status.feedback.valid || !param_valid(&g_app_param)) fault(STOP_ENCODER);
        else {
            float speed = searching ? fminf(APP_START_SEARCH_SPEED_MPS, g_app_param.tune_speed_mps)
                                    : g_app_param.debug_speed_mps;
            bool ok = motion_plan_step(&plan, &g_app_param, speed, 0,
                                       dt, false, STEERING_NORMAL, &status.targets);
            if (ok) ok = speed_pi_step(&pi[0], plan.left, status.feedback.speed_mps[0], dt,
                g_app_param.kp_left, g_app_param.ki_left, g_app_param.ff_left,
                g_app_param.integral_limit, g_app_param.duty_limit, &status.duty[0]);
            if (ok) ok = speed_pi_step(&pi[1], plan.right, status.feedback.speed_mps[1], dt,
                g_app_param.kp_right, g_app_param.ki_right, g_app_param.ff_right,
                g_app_param.integral_limit, g_app_param.duty_limit, &status.duty[1]);
            if (!ok) fault(STOP_NONFINITE);
            else motor_apply_forward(status.duty[0], status.duty[1]);
        }
    } else if (status.mode == RUN_RUNNING || status.mode == RUN_STOPPING || status.mode == RUN_TUNE_RUNNING) {
        if (!status.feedback.valid)
            fault(STOP_ENCODER);
        else if (!good.valid || good.session_id != status.session_id ||
                 status.vision_age_ms > (uint32_t)g_app_param.hard_timeout_ms)
            fault(STOP_VISION);
        else if (incoming.session_id == status.session_id && incoming.stop_requested) {
            status.mode = RUN_STOPPING;
            status.reason = STOP_TRACK;
        }
        if (status.mode == RUN_RUNNING || status.mode == RUN_STOPPING || status.mode == RUN_TUNE_RUNNING) {
            bool stopping = status.mode == RUN_STOPPING;
            float speed = good.speed_cap_mps, omega = good.omega_raw;
            if (status.mode == RUN_TUNE_RUNNING) speed = fminf(speed, g_app_param.tune_speed_mps);
            if (status.vision_age_ms > (uint32_t)g_app_param.soft_timeout_ms) {
                speed = 0;
                stopping = true;
            }
            bool ok = true;
            StopReason failure = STOP_NONFINITE;
            SteeringMode steering = good.sharp_turn ? STEERING_SHARP_TURN :
                                    good.recentering ? STEERING_RECENTER : STEERING_NORMAL;
            if (gyro_enabled && gyro_ready && !stopping && speed > 0 &&
                gyro_corner_step(&gyro_corner, good.corner_direction, good.frame_id, now,
                                 status.gyro.angle_rad,
                                 .5f * (status.feedback.rolling_m[0] + status.feedback.rolling_m[1]),
                                 &omega)) {
                speed = fminf(speed, g_app_param.speed_corner);
                steering = STEERING_SHARP_TURN;
            }
            status.omega_target_radps = omega;
            if (gyro_enabled && !stopping) {
                ok = gyro_ready && yaw_control_step(&yaw_integral, &g_app_param, omega,
                         status.gyro.rate_radps, dt, status.targets.limited, &omega);
                if (!ok)
                    failure = STOP_GYRO;
            } else {
                yaw_integral = 0;
                gyro_corner = (GyroCornerTurn){0};
            }
            status.omega_command_radps = ok ? omega : 0;
            if (ok)
                ok = motion_plan_step(&plan, &g_app_param, speed, omega, dt, stopping,
                                      steering, &status.targets);
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
                fault(failure);
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
        memset(&status.targets, 0, sizeof(status.targets));
        motor_brake_zero();
        status.duty[0] = status.duty[1] = 0;
        memset(pi, 0, sizeof(pi));
        memset(&plan, 0, sizeof(plan));
    }
    uint32_t used_us = board_time_us() - start;
    if (used_us > status.control_max_us)
        status.control_max_us = used_us;
    if (used_us > 4000) {
        status.overruns++;
        if (status.overruns >= 3 && (status.mode == RUN_RUNNING || status.mode == RUN_DUTY_DEBUG ||
                                   status.mode == RUN_SPEED_DEBUG || status.mode == RUN_TUNE_RUNNING || searching))
            fault(STOP_TIMING);
    }
    app_exchange_publish_control(&status);
}
