/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_core.h"
#include <math.h>
#include <string.h>
void path_control_reset(PathControlState *s, uint32_t session) {
    memset(s, 0, sizeof(*s));
    s->session = session;
}
bool path_control_step(PathControlState *s, const VisionFeatures *f, const TrackDecision *d,
                       const AppParam *p, MotionRequest *o) {
    if (!s || !f || !d || !p || !o)
        return false;
    memset(o, 0, sizeof(*o));
    o->frame_id = f->frame_id;
    o->capture_ms = f->capture_ms;
    o->session_id = s->session;
    o->state = d->state;
    o->confidence = f->confidence;
    o->stop_requested = d->stop;
    if (!d->valid || d->path_index >= f->path_count || s->previous_frame == f->frame_id)
        return false;
    const PathCandidate *q = &f->paths[d->path_index];
    float sum = 0, weight = 0;
    for (unsigned i = 2; i < APP_ROWS - 3; i++)
        if (q->valid_rows & (1u << i)) {
            float w = i < APP_ROWS / 2 ? 2.0f : 1.0f;
            sum += w * q->x[i];
            weight += w;
        }
    if (weight == 0)
        return false;
    float e = (sum / weight - p->center_x_px) / (APP_WIDTH * .5f);
    uint32_t elapsed = f->capture_ms - s->previous_ms;
    float dt = (float)elapsed * .001f;
    if (!s->initialized || d->reset_d || elapsed < 5 || elapsed > 200)
        s->derivative = 0;
    else {
        s->derivative += dt / (p->visual_tau_s + dt) * ((e - s->error) / dt - s->derivative);
    }
    s->error = e;
    s->previous_frame = f->frame_id;
    s->previous_ms = f->capture_ms;
    s->initialized = true;
    o->omega_raw = -(p->visual_kp * e + p->visual_kd * s->derivative);
    o->speed_cap_mps = d->speed_cap;
    o->valid = isfinite(o->omega_raw) && isfinite(o->speed_cap_mps);
    return o->valid;
}
