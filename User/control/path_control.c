/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "control_api.h"
#include <math.h>
#include <string.h>
void path_control_reset(PathControlState *s, uint32_t session) {
    memset(s, 0, sizeof(*s));
    s->session = session;
}
bool control_path_step(PathControlState *s, const VisionOutput *v, const AppParam *p,
                       MotionRequest *o) {
    if (!s || !v || !p || !o)
        return false;
    memset(o, 0, sizeof(*o));
    o->frame_id = v->frame_id;
    o->capture_ms = v->capture_ms;
    o->session_id = v->session_id;
    o->state = v->state;
    o->confidence = v->confidence;
    o->stop_requested = v->stop_requested;
    if (!v->valid || v->session_id != s->session || v->frame_id == s->previous_frame ||
        !isfinite(v->lateral_m) || !isfinite(v->forward_m) || v->forward_m <= -VISION_FORWARD_OFFSET_M ||
        !isfinite(v->curvature) || !isfinite(v->heading_rad) ||
        !isfinite(p->heading_gain_per_s) || p->heading_gain_per_s < 0 ||
        p->heading_gain_per_s > APP_HEADING_GAIN_MAX ||
        !isfinite(p->lateral_kp) || p->lateral_kp < 0 ||
        !isfinite(p->lateral_kd) || p->lateral_kd < 0 ||
        !isfinite(p->lateral_tau_s) || p->lateral_tau_s < 0 ||
        !isfinite(p->speed_limit_mps) || p->speed_limit_mps <= 0 ||
        !isfinite(v->speed_cap_mps) || v->speed_cap_mps < 0)
        return false;
    float e = v->lateral_m;
    uint32_t elapsed = v->capture_ms - s->previous_ms;
    float dt = (float)elapsed * .001f;
    if (!s->initialized || v->reset_derivative || elapsed < 5 || elapsed > 200)
        s->derivative = 0;
    else
        s->derivative += dt / (p->lateral_tau_s + dt) * ((e - s->error) / dt - s->derivative);
    s->error = e;
    s->previous_frame = v->frame_id;
    s->previous_ms = v->capture_ms;
    s->initialized = true;
    float speed = fminf(v->speed_cap_mps, p->speed_limit_mps);
    o->omega_raw = -speed * v->curvature - p->heading_gain_per_s * v->heading_rad
                   - (p->lateral_kp * e + p->lateral_kd * s->derivative);
    o->speed_cap_mps = v->speed_cap_mps;
    o->corner_direction = v->corner_direction;
    const float sharp_turn_heading_rad = .78539816f; /* 45 degrees */
    o->sharp_turn = v->near_transverse || fabsf(v->heading_rad) >= sharp_turn_heading_rad;
    if (o->sharp_turn)
        o->omega_raw *= 1.5f;
    else if (!v->corner_direction && v->has_p0 && isfinite(v->p0_x_px) && isfinite(p->center_x_px)) {
        const float center_deadband_px = 3.0f;
        const float centering_radps_per_px = .09f;
        const float max_centering_omega = 1.8f;
        float offset_px = v->p0_x_px - p->center_x_px;
        float correction = fmaxf(fabsf(offset_px) - center_deadband_px, 0) * centering_radps_per_px;
        o->omega_raw -= copysignf(fminf(correction, max_centering_omega), offset_px);
        o->recentering = correction > 0;
    }
    o->valid = isfinite(o->omega_raw);
    return o->valid;
}
