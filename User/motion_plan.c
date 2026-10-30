/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_core.h"
#include <math.h>
#include <string.h>
bool motion_plan_step(MotionPlanState *s, const AppParam *p, float speed, float omega, float dt,
                      bool stop, SideTargets *out) {
    if (!s || !p || !out)
        return false;
    memset(out, 0, sizeof(*out));
    float b = param_track(p);
    if (!isfinite(speed) || !isfinite(omega) || !isfinite(dt) || !isfinite(b) || b <= 0 ||
        dt <= 0 || dt > .05f || !isfinite(s->left) || !isfinite(s->right))
        return false;
    float target = stop ? 0 : app_clamp(speed, 0, p->speed_limit_mps);
    float oldv = (s->left + s->right) * .5f;
    float v = app_clamp(target, oldv - p->decel_mps2 * dt, oldv + p->accel_mps2 * dt);
    v = fmaxf(v, 0);
    if (stop) {
        omega = oldv > .001f ? (s->right - s->left) / b * (v / oldv) : 0;
    }
    float wlim = 2 * (1 - p->inner_ratio) * v / b;
    float w = app_clamp(omega, -wlim, wlim);
    if (v < .0001f) {
        v = 0;
        w = 0;
    }
    float l = v - w * b * .5f, r = v + w * b * .5f, peak = fmaxf(l, r),
          scale = peak > p->speed_limit_mps ? p->speed_limit_mps / peak : 1;
    l *= scale;
    r *= scale;
    float dl = l - s->left, dr = r - s->right, lam = 1;
    float al = (dl >= 0 ? p->accel_mps2 : p->decel_mps2) * dt,
          ar = (dr >= 0 ? p->accel_mps2 : p->decel_mps2) * dt;
    if (fabsf(dl) > al)
        lam = fminf(lam, al / fabsf(dl));
    if (fabsf(dr) > ar)
        lam = fminf(lam, ar / fabsf(dr));
    s->left = fmaxf(0, s->left + lam * dl);
    s->right = fmaxf(0, s->right + lam * dr);
    out->left_mps = s->left;
    out->right_mps = s->right;
    out->model_v_mps = (s->left + s->right) * .5f;
    out->model_omega_radps = (s->right - s->left) / b;
    out->limited = (w != omega || scale < 1 || lam < 1);
    return isfinite(s->left) && isfinite(s->right);
}
