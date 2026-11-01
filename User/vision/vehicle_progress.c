/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "vision_api.h"
#include <math.h>
bool vehicle_progress_rear_clear(const ElementAnchor *a, const AppParam *p, const SideFeedback *fb,
                                 bool stable, uint32_t now) {
    if (!a || !p || !fb || !a->valid || !fb->valid || !stable || !isfinite(a->x_exit_m))
        return false;
    float ds = (fb->rolling_m[0] + fb->rolling_m[1]) * .5f - a->s_anchor_m;
    return ds > a->x_exit_m + p->axle_m * .5f + p->clear_distance_m &&
           (uint32_t)(now - a->stamp_ms) >= (uint32_t)p->release_ms;
}
