#ifndef YAW_CONTROL_H
#define YAW_CONTROL_H
#include "app_types.h"

typedef struct {
    float mean, m2, rate_radps, angle_rad;
    uint32_t stamp_ms;
    uint16_t count;
    bool ready;
} GyroEstimator;

typedef struct {
    float entry_angle_rad, finish_distance_m;
    uint32_t entry_ms, finish_frame;
    int8_t direction;
    bool active;
} GyroCornerTurn;

bool gyro_corner_step(GyroCornerTurn *turn, int8_t direction, uint32_t frame_id,
                      uint32_t now, float angle_rad, float distance_m, float *omega);
bool gyro_cached_turn_step(float goal_rad, float entry_rad, float measured_rad, float *omega);

bool gyro_estimator_step(GyroEstimator *s, float z, float norm, float accel,
                         bool stationary, uint32_t now);
bool yaw_control_step(float *integral, const AppParam *p, float target, float measured,
                      float dt, bool planner_limited, float *command);
#endif
