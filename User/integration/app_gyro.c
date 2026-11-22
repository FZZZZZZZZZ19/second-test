/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "yaw_control.h"
#include <math.h>

static GyroEstimator estimator;
static bool device_ok;
static uint32_t last_poll;
static uint32_t read_max_us;

void app_gyro_init(void) {
    device_ok = board_imu_init();
    app_control_publish_gyro(&(GyroMeasurement){
        .status = device_ok ? GYRO_CALIBRATING : GYRO_ERROR
    });
}

void app_gyro_zero_angle(void) {
    estimator.angle_rad = 0;
}

void app_gyro_poll(const ControlSnapshot *s) {
    uint32_t now = board_time_ms();
    if (!device_ok || now - last_poll < APP_GYRO_PERIOD_MS)
        return;
    last_poll = now;
    bool stationary = s->mode == RUN_STATIC_DEBUG &&
        (!s->feedback.valid || (fabsf(s->feedback.speed_mps[0]) < .03f &&
                               fabsf(s->feedback.speed_mps[1]) < .03f));
    if (!estimator.ready && !stationary) {
        estimator = (GyroEstimator){0};
        app_control_publish_gyro(&(GyroMeasurement){
            .status = GYRO_CALIBRATING, .read_max_us = read_max_us
        });
        return;
    }
    uint32_t started = board_time_us();
    float z, norm, accel;
    int result = board_imu_read(!estimator.ready, &z, &norm, &accel);
    uint32_t elapsed = board_time_us() - started;
    if (elapsed > read_max_us)
        read_max_us = elapsed;
    if (!result)
        return;
    now = board_time_ms();
    bool valid = result > 0 && gyro_estimator_step(&estimator, z, norm, accel, stationary, now);
    if (!valid) {
        if (!estimator.ready)
            estimator = (GyroEstimator){0};
        /* Never integrate across a transfer failure. */
        estimator.stamp_ms = now - APP_GYRO_TIMEOUT_MS - 1u;
    }
    float sign = (g_app_param.flags & APP_FLAG_GYRO_INVERT) ? -1 : 1;
    app_control_publish_gyro(&(GyroMeasurement){
        .stamp_ms = now, .read_max_us = read_max_us, .rate_radps = sign * estimator.rate_radps,
        .angle_rad = sign * estimator.angle_rad, .calibration_samples = estimator.count,
        .status = !valid ? GYRO_ERROR : estimator.ready ? GYRO_READY : GYRO_CALIBRATING
    });
}
