/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "track_profile.h"
#include "vehicle_calibration.h"
#include "yaw_control.h"
#include <math.h>
#include <string.h>
static ControlSnapshot status;
static MotionRequest good;
static SpeedPiState pi[2];
static MotionPlanState plan;
static uint32_t previous_us, imu_stamp;
static bool imu_online, imu_seen;
static GyroCalibration calibration;
static YawState yaw;
void app_control_imu_ready(bool ok) {
    imu_online = ok;
    imu_stamp = board_time_ms();
}

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
    memset(&calibration, 0, sizeof(calibration));
    memset(&yaw, 0, sizeof(yaw));
    imu_online = false;
    imu_seen = false;
    previous_us = board_time_us();
    initialized = true;
    app_exchange_publish_control(&status);
}
static void fault(StopReason r) {
    yaw.integral = 0;
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
                       CAL_VISION_V2_VALIDATED &&
                       (!CAL_FULL_RACE || (APP_ROUTE_LENGTH > 0 && APP_FINISH_DISTANCE_M > 0));
    if (elapsed == 0 || elapsed > 20000) {
        if (status.mode == RUN_RUNNING || status.mode == RUN_STOPPING)
            fault(STOP_TIMING);
        dt = .005f;
    }
    (void)side_encoder_sample(dt, now, &status.feedback);
    status.omega_target = good.valid ? good.omega_raw : 0;
    float imu_timeout = g_app_param.gyro_timeout_ms;
    if (!isfinite(imu_timeout) || imu_timeout < 10 || imu_timeout > 50)
        imu_online = false;
    if (imu_online) {
        float z = 0, norm = 0, accel = 0;
        uint32_t read_start = board_time_us();
        int result = board_imu_read(!calibration.ready, &z, &norm, &accel);
        status.imu_read_us = board_time_us() - read_start;
        if (status.imu_read_us > status.imu_max_us)
            status.imu_max_us = status.imu_read_us;
        if (result < 0 || status.imu_read_us >= 2000)
            imu_online = false;
        else if (result > 0) {
            imu_seen = true;
            imu_stamp = now;
            if (!calibration.ready)
                (void)gyro_calibrate_step(&calibration, &g_app_param, z, norm, accel,
                                          status.mode == RUN_STATIC_DEBUG &&
                                              status.feedback.delta[0] == 0 &&
                                              status.feedback.delta[1] == 0,
                                          now);
            if (calibration.failed ||
                !gyro_measure(z, calibration.bias, g_app_param.gyro_sign, &status.omega_measured))
                imu_online = false;
        }
        if ((uint32_t)(now - imu_stamp) > (uint32_t)imu_timeout)
            imu_online = false;
    }
    status.imu_ok = imu_online && imu_seen;
    status.imu_calibrated = calibration.ready;
    status.imu_cal_samples = (uint16_t)calibration.count;
    status.gyro_bias = calibration.bias;
    status.config_ok = status.config_ok && status.imu_ok && calibration.ready;
    if (!imu_online &&
        (status.mode == RUN_READY || status.mode == RUN_RUNNING || status.mode == RUN_STOPPING))
        fault(STOP_IMU);
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
        yaw.integral = 0;
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
            status.omega_target = omega;
            bool ok = yaw_control_step(&yaw, &g_app_param, omega, status.omega_measured, dt,
                                       stopping, &omega);
            if (ok)
                ok = motion_plan_step(&plan, &g_app_param, speed, omega, dt, stopping,
                                      &status.targets);
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
        yaw.integral = 0;
        memset(pi, 0, sizeof(pi));
        memset(&plan, 0, sizeof(plan));
    }
    uint32_t tick_us = board_time_us() - start;
    if (tick_us > status.tick_max_us)
        status.tick_max_us = tick_us;
    if (board_time_us() - start > 4000) {
        status.overruns++;
        if (status.overruns >= 3 && status.mode == RUN_RUNNING)
            fault(STOP_TIMING);
    }
    app_exchange_publish_control(&status);
}
