#ifndef CONTROL_TUNING_H
#define CONTROL_TUNING_H
#include "vehicle_param.h"
static inline void control_param_defaults(AppParam *p) {
    p->encoder_sign[0] = 1;
    p->encoder_sign[1] = 1;
    p->encoder_tau_s = .025f;
    p->kp_left = .2f;
    p->kp_right = .2f;
    p->ki_left = .4f;
    p->ki_right = .4f;
    p->integral_limit = .2f;
    p->duty_limit = .25f;
    p->speed_limit_mps = .4f;
    p->accel_mps2 = .3f;
    p->decel_mps2 = .6f;
    p->inner_ratio = .15f;
    p->visual_kp = 1.5f;
    p->visual_kd = .05f;
    p->visual_tau_s = .05f;
    p->speed_normal = .2f;
    p->speed_corner = .12f;
    p->speed_element = .1f;
    p->soft_timeout_ms = 150;
    p->hard_timeout_ms = 600;
}
#endif
