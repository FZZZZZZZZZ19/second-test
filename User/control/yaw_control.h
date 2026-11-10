#ifndef YAW_CONTROL_H
#define YAW_CONTROL_H
#include "vehicle_param.h"
#define GYRO_CALIBRATION_SAMPLES 800u
typedef struct {
    float integral;
} YawState;
typedef struct {
    unsigned count;
    float mean, m2, bias;
    uint32_t stamp_ms;
    bool ready, failed, seen;
} GyroCalibration;
bool gyro_calibrate_step(GyroCalibration *s, const AppParam *p, float z, float gyro_norm,
                         float accel_norm, bool stationary, uint32_t now);
bool gyro_measure(float raw_radps, float bias_radps, float sign, float *measured);
bool yaw_control_step(YawState *s, const AppParam *p, float target, float measured, float dt,
                      bool stopping, float *command);
#endif
