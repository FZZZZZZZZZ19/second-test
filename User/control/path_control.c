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
        !isfinite(v->direction_rad) || !isfinite(v->distance_px) || v->distance_px <= 0 ||
        !isfinite(v->speed_cap_mps) || v->speed_cap_mps < 0)
        return false;
    float e = v->direction_rad;
    uint32_t elapsed = v->capture_ms - s->previous_ms;
    float dt = (float)elapsed * .001f;
    if (!s->initialized || v->reset_derivative || elapsed < 5 || elapsed > 200)
        s->derivative = 0;
    else
        s->derivative += dt / (p->visual_tau_s + dt) * ((e - s->error) / dt - s->derivative);
    s->error = e;
    s->previous_frame = v->frame_id;
    s->previous_ms = v->capture_ms;
    s->initialized = true;
    o->direction_rad = v->direction_rad;
    o->distance_px = v->distance_px;
    o->omega_raw = -(p->visual_kp * e + p->visual_kd * s->derivative);
    o->speed_cap_mps = v->speed_cap_mps;
    o->valid = isfinite(o->omega_raw);
    return o->valid;
}
