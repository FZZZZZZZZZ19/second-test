/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "yaw_control.h"
#include "core_math.h"
#include <math.h>

bool gyro_corner_step(GyroCornerTurn *turn, int8_t direction, uint32_t frame_id,
                      uint32_t now, float angle_rad, float distance_m, float *omega) {
    const float quarter_turn_rad = 1.57079633f;
    const float finish_margin_rad = .08726646f; /* Hand back to vision at 85 degrees. */
    const uint32_t max_turn_ms = 2500u;
    if (!isfinite(angle_rad) || !isfinite(distance_m)) {
        *turn = (GyroCornerTurn){0};
        return false;
    }
    if (!turn->active) {
        if (!direction) {
            if (!turn->direction || frame_id != turn->finish_frame)
                turn->direction = 0;
            return false;
        }
        /* A second cue may follow immediately. A repeated same-side L needs
         * a new entry after the cue cleared, or forward progress past the exit. */
        if (turn->direction && (frame_id == turn->finish_frame ||
            (direction == turn->direction && distance_m - turn->finish_distance_m < .030f)))
            return false;
        turn->direction = direction;
        turn->entry_angle_rad = angle_rad;
        turn->entry_ms = now;
        turn->active = true;
    }
    float remaining = quarter_turn_rad - turn->direction * (angle_rad - turn->entry_angle_rad);
    if (remaining <= finish_margin_rad || now - turn->entry_ms >= max_turn_ms) {
        turn->active = false;
        turn->finish_frame = frame_id;
        turn->finish_distance_m = distance_m;
        return false;
    }
    *omega = turn->direction * app_clamp(4 * remaining, .65f, 3.0f);
    return true;
}

bool gyro_estimator_step(GyroEstimator *s, float z, float norm, float accel,
                         bool stationary, uint32_t now) {
    if (!isfinite(z) || !isfinite(norm) || !isfinite(accel))
        return false;
    uint32_t elapsed = now - s->stamp_ms;
    if (s->ready) {
        float rate = z - s->mean;
        if (elapsed <= APP_GYRO_TIMEOUT_MS)
            s->angle_rad += .0005f * (float)elapsed * (s->rate_radps + rate);
        s->rate_radps = rate;
        s->stamp_ms = now;
        return isfinite(s->rate_radps) && isfinite(s->angle_rad);
    }
    if (!stationary || norm > .08f || fabsf(accel - 1) > .1f) {
        *s = (GyroEstimator){0};
        return true;
    }
    if (s->count && elapsed > APP_GYRO_TIMEOUT_MS)
        *s = (GyroEstimator){0};
    s->stamp_ms = now;
    s->count++;
    float delta = z - s->mean;
    s->mean += delta / s->count;
    s->m2 += delta * (z - s->mean);
    if (s->count == APP_GYRO_CAL_SAMPLES) {
        if (s->m2 / (s->count - 1) <= .0001f)
            s->ready = true;
        else
            *s = (GyroEstimator){0};
    }
    return true;
}

bool yaw_control_step(float *integral, const AppParam *p, float target, float measured,
                      float dt, bool planner_limited, float *command) {
    if (!isfinite(target) || !isfinite(measured) || !isfinite(dt) || dt <= 0 || dt > .05f ||
        !isfinite(*integral) || !isfinite(p->yaw_kp) || !isfinite(p->yaw_ki) ||
        p->yaw_kp < 0 || p->yaw_kp > APP_YAW_GAIN_MAX ||
        p->yaw_ki < 0 || p->yaw_ki > APP_YAW_GAIN_MAX)
        return false;
    float error = target - measured;
    float inc = p->yaw_ki * error * dt;
    float proportional = p->yaw_kp * error;
    float candidate = app_clamp(*integral + inc, -.5f, .5f);
    float correction = proportional + candidate;
    if (!isfinite(correction) || !isfinite(inc))
        return false;
    bool unwinding = fabsf(candidate) < fabsf(*integral);
    if ((!planner_limited || unwinding) &&
        !((correction > 2 && inc > 0) || (correction < -2 && inc < 0)))
        *integral = candidate;
    *command = target + app_clamp(proportional + *integral, -2, 2);
    return isfinite(*command);
}
