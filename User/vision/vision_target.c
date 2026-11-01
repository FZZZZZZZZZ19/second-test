/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "vision_api.h"
#include <math.h>
#include <string.h>
bool vision_target_step(const VisionFeatures *f, const TrackDecision *d, const AppParam *p,
                        VisionOutput *o) {
    if (!f || !d || !p || !o)
        return false;
    memset(o, 0, sizeof(*o));
    o->frame_id = f->frame_id;
    o->capture_ms = f->capture_ms;
    o->state = d->state;
    o->confidence = f->confidence;
    o->stop_requested = d->stop;
    o->reset_derivative = d->reset_d;
    o->speed_cap_mps = d->speed_cap;
    if (!d->valid || d->path_index >= f->path_count)
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
    if (q->target_valid) {
        float tx = q->target_x, ty = q->target_y;
        /* Nearby folds: aim at the next corner on the selected side. */
        int best = -1;
        unsigned best_y = 0;
        for (unsigned i = 0; i < f->boundary.corner_count; i++) {
            VisionPoint c = f->boundary.corners[i];
            if (c.y > 20 && c.y < APP_HEIGHT - 12 && c.y > best_y &&
                ((tx >= p->center_x_px) == (f->boundary.corner_side[i] != 0))) {
                best = (int)i;
                best_y = c.y;
            }
        }
        if (best >= 0 && d->state == TRACK_NORMAL && !f->cross && !f->split) {
            VisionPoint c = f->boundary.corners[best];
            tx = (tx + c.x) * .5f;
            ty = (ty + c.y) * .5f;
        }
        float dx = tx - p->center_x_px, dy = fmaxf(1, (APP_HEIGHT - 1) - ty);
        o->direction_rad = atan2f(dx, dy);
        o->distance_px = sqrtf(dx * dx + dy * dy);
    } else {
        o->direction_rad = atan2f(sum / weight - p->center_x_px, APP_HEIGHT * .5f);
        o->distance_px = APP_HEIGHT * .5f;
    }
    o->valid = isfinite(o->direction_rad) && isfinite(o->distance_px) && isfinite(o->speed_cap_mps);
    return o->valid;
}
