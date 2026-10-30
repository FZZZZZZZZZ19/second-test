/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_core.h"
#include <math.h>
bool speed_pi_step(SpeedPiState *s, float target, float measured, float dt, float kp, float ki,
                   float ff, float ilim, float limit, float *out) {
    if (!s || !out)
        return false;
    if (!isfinite(target) || !isfinite(measured) || !isfinite(dt) || !isfinite(kp) ||
        !isfinite(ki) || !isfinite(ff) || !isfinite(ilim) || !isfinite(limit) ||
        !isfinite(s->integral) || dt <= 0 || dt > .05f || target < 0 || kp < 0 || ki < 0 ||
        ff < 0 || ilim < 0 || limit <= 0 || limit > 1) {
        *out = 0;
        s->integral = 0;
        return false;
    }
    float e = target - measured, inc = ki * e * dt,
          candidate = app_clamp(s->integral + inc, -ilim, ilim);
    float base = ff * target + kp * e, raw = base + candidate;
    if (!((raw > limit && inc > 0) || (raw < 0 && inc < 0)))
        s->integral = candidate;
    *out = app_clamp(base + s->integral, 0, limit);
    s->last_output = *out;
    return isfinite(*out);
}
