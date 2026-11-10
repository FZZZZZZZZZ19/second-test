#ifndef CONTROL_TUNING_H
#define CONTROL_TUNING_H
#include "vehicle_param.h"
static inline void control_param_defaults(AppParam *p) {
    p->gyro_sign = 1;
    p->yaw_kp = 0.0f; /* tune only after sign/timing validation */
    p->yaw_ki = 0.0f;
    p->yaw_i_limit = .5f; /* rad/s */
    p->yaw_output_limit = 5.0f; /* rad/s, still subject to motion_plan limits */
    p->gyro_timeout_ms = 20;
    p->gyro_still_radps = .08f;
    p->gyro_accel_tol_g = .1f;
    p->gyro_noise_radps = .01f;
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
    p->lateral_kp = 0.0f; /* rad/(s*m), tune on vehicle */
    p->lateral_kd = 0.0f; /* rad/m, tune on vehicle */
    p->lateral_tau_s = .05f;
    p->speed_normal = .2f;
    p->speed_corner = .12f;
    p->speed_element = .1f;
    p->soft_timeout_ms = 150;
    p->hard_timeout_ms = 600;
}
#endif
