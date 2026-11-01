#ifndef VEHICLE_PARAM_H
#define VEHICLE_PARAM_H
#include "app_config.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* All speed/distance values are SI units; gain outputs use duty 0..1. */
#define APP_FLOAT_FIELDS(X)                                                                        \
    X(wheel_left_m)                                                                                \
    X(wheel_right_m)                                                                               \
    X(track_m)                                                                                     \
    X(effective_track_m)                                                                           \
    X(axle_m) X(counts_left) X(counts_right) X(encoder_tau_s) X(kp_left) X(ki_left) X(kp_right)    \
        X(ki_right) X(ff_left) X(ff_right) X(integral_limit) X(duty_limit) X(speed_limit_mps)      \
            X(accel_mps2) X(decel_mps2) X(inner_ratio) X(visual_kp) X(visual_kd) X(visual_tau_s)   \
                X(center_x_px) X(speed_normal) X(speed_corner) X(speed_element) X(soft_timeout_ms) \
                    X(hard_timeout_ms) X(element_timeout_s) X(clear_distance_m)                    \
                        X(island_min_distance_m) X(island_min_turn_rad) X(camera_forward_m)        \
                            X(row_distance_near_m) X(row_distance_far_m) X(width_near_px)          \
                                X(width_far_px) X(row_jump_px) X(confirm_ms) X(release_ms)
typedef struct {
#define DECL_FIELD(n) float n;
    APP_FLOAT_FIELDS(DECL_FIELD)
#undef DECL_FIELD
    uint32_t calibration_mask;
    uint32_t flags; /* bit0: verified race-element profile, bit1: capture timing validated */
    uint8_t threshold;
    int8_t encoder_sign[2];
} AppParam;
void param_defaults(AppParam *p);
bool param_valid(const AppParam *p);
bool param_can_arm(const AppParam *p, bool full_race);
float param_track(const AppParam *p);
bool param_encode(const AppParam *p, uint32_t sequence, uint8_t out[APP_PARAM_RECORD_BYTES]);
bool param_decode(const uint8_t data[APP_PARAM_RECORD_BYTES], AppParam *p, uint32_t *sequence);
bool param_store_load(AppParam *p);
bool param_store_save(const AppParam *p); /* static maintenance only */
#endif
