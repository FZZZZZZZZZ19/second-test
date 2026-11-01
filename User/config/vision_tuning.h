#ifndef VISION_TUNING_H
#define VISION_TUNING_H
#include "vehicle_param.h"
static inline void vision_param_defaults(AppParam *p) {
    p->center_x_px = 93.5f;
    p->element_timeout_s = 20;
    p->confirm_ms = 80;
    p->release_ms = 120;
    p->clear_distance_m = .12f;
    p->island_min_distance_m = .5f;
    p->island_min_turn_rad = 5.4f;
    p->width_near_px = 20;
    p->width_far_px = 5;
    p->row_jump_px = 28;
    p->threshold = 130;
}
#endif
