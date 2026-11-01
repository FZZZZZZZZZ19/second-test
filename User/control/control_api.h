#ifndef CONTROL_API_H
#define CONTROL_API_H
#include "core_math.h"
#include "vision_control.h"
bool speed_pi_step(SpeedPiState *s, float target, float measured, float dt, float kp, float ki,
                   float ff, float integral_limit, float duty_limit, float *out);
bool motion_plan_step(MotionPlanState *s, const AppParam *p, float speed, float omega, float dt,
                      bool stop, SideTargets *out);
void path_control_reset(PathControlState *s, uint32_t session);
bool control_path_step(PathControlState *s, const VisionOutput *v, const AppParam *p,
                       MotionRequest *out);
#endif
