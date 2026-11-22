#ifndef YAW_CONTROL_H
#define YAW_CONTROL_H
#include "app_types.h"

typedef struct {
    float mean, m2, rate_radps, angle_rad;
    uint32_t stamp_ms;
    uint16_t count;
    bool ready;
} GyroEstimator;

bool gyro_estimator_step(GyroEstimator *s, float z, float norm, float accel,
                         bool stationary, uint32_t now);
bool yaw_control_step(float *integral, const AppParam *p, float target, float measured,
                      float dt, bool planner_limited, float *command);
#endif
