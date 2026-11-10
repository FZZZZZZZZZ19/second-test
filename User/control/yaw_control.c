/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "yaw_control.h"
#include "core_math.h"
#include <math.h>
#include <string.h>
bool gyro_measure(float raw, float bias, float sign, float *out) {
    if (!out)
        return false;
    *out = 0;
    if (!isfinite(raw) || !isfinite(bias) || (sign != 1 && sign != -1))
        return false;
    *out = sign * (raw - bias);
    return isfinite(*out);
}
bool gyro_calibrate_step(GyroCalibration *s, const AppParam *p, float z, float norm, float accel,
                         bool stationary, uint32_t now) {
    if (!s || !p || s->failed)
        return false;
    if (!isfinite(z) || !isfinite(norm) || !isfinite(accel)) {
        s->failed = true;
        s->ready = false;
        return false;
    }
    if (s->ready)
        return true;
    if (!stationary || norm > p->gyro_still_radps || fabsf(accel - 1) > p->gyro_accel_tol_g) {
        memset(s, 0, sizeof(*s));
        return false;
    }
    if (s->seen && (uint32_t)(now - s->stamp_ms) < 4)
        return false;
    if (s->seen && (uint32_t)(now - s->stamp_ms) > (uint32_t)p->gyro_timeout_ms)
        memset(s, 0, sizeof(*s));
    s->seen = true;
    s->stamp_ms = now;
    s->count++;
    float delta = z - s->mean;
    s->mean += delta / s->count;
    s->m2 += delta * (z - s->mean);
    if (s->count >= GYRO_CALIBRATION_SAMPLES) {
        if (s->m2 / (s->count - 1) > p->gyro_noise_radps * p->gyro_noise_radps) {
            memset(s, 0, sizeof(*s));
            return false;
        }
        s->bias = s->mean;
        s->ready = true;
    }
    return s->ready;
}
bool yaw_control_step(YawState *s, const AppParam *p, float target, float measured, float dt,
                      bool stopping, float *out) {
    if (!s || !p || !out)
        return false;
    *out = 0;
    if (!isfinite(target) || !isfinite(measured) || !isfinite(dt) || dt <= 0 || dt > .05f ||
        !isfinite(s->integral) || !isfinite(p->yaw_kp) || p->yaw_kp < 0 || !isfinite(p->yaw_ki) ||
        p->yaw_ki < 0 || !isfinite(p->yaw_i_limit) || p->yaw_i_limit < 0 ||
        !isfinite(p->yaw_output_limit) || p->yaw_output_limit <= 0) {
        s->integral = 0;
        return false;
    }
    if (stopping) {
        s->integral = 0;
        *out = target;
        return true;
    }
    float e = target - measured, inc = p->yaw_ki * e * dt;
    float base = target + p->yaw_kp * e;
    float candidate = app_clamp(s->integral + inc, -p->yaw_i_limit, p->yaw_i_limit);
    float raw = base + candidate;
    if (!isfinite(base) || !isfinite(inc) || !isfinite(raw)) {
        s->integral = 0;
        return false;
    }
    if (!((raw > p->yaw_output_limit && inc > 0) || (raw < -p->yaw_output_limit && inc < 0)))
        s->integral = candidate;
    *out = app_clamp(base + s->integral, -p->yaw_output_limit, p->yaw_output_limit);
    return isfinite(*out);
}
