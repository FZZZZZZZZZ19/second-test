#ifndef APP_CORE_H
#define APP_CORE_H
#include "app_types.h"
float app_clamp(float x, float lo, float hi);
int32_t app_count_delta(uint16_t now, uint16_t previous);
uint32_t app_crc32(const uint8_t *data, size_t size);
bool speed_pi_step(SpeedPiState *s, float target, float measured, float dt, float kp, float ki,
                   float ff, float integral_limit, float duty_limit, float *out);
bool motion_plan_step(MotionPlanState *s, const AppParam *p, float speed, float omega, float dt,
                      bool stop, SideTargets *out);
bool vision_line_process(const FrameView *f, const AppParam *p, VisionFeatures *out);
bool vision_boundary_process(const FrameView *f, BoundaryFeatures *out);
bool vision_component_pixel(unsigned x, unsigned y);
void track_element_reset(TrackContext *s, uint32_t session, bool race, uint8_t y_count);
bool track_element_step(TrackContext *s, const VisionFeatures *f, const AppParam *p,
                        const SideFeedback *fb, bool commit, bool yaw_valid, float yaw,
                        TrackDecision *out);
bool vehicle_progress_rear_clear(const ElementAnchor *a, const AppParam *p, const SideFeedback *fb,
                                 bool stable, uint32_t now);
void path_control_reset(PathControlState *s, uint32_t session);
bool path_control_step(PathControlState *s, const VisionFeatures *f, const TrackDecision *d,
                       const AppParam *p, MotionRequest *out);
#endif
