#ifndef VISION_API_H
#define VISION_API_H
#include "core_math.h"
#include "vision_control.h"
bool vision_line_process(const FrameView *f, const AppParam *p, VisionFeatures *out);
bool vision_boundary_process(const FrameView *f, BoundaryFeatures *out);
bool vision_component_pixel(unsigned x, unsigned y);
void track_element_reset(TrackContext *s, uint32_t session, bool race, uint8_t y_count);
bool track_element_step(TrackContext *s, const VisionFeatures *f, const AppParam *p,
                        const SideFeedback *fb, bool commit, bool yaw_valid, float yaw,
                        TrackDecision *out);
bool vehicle_progress_rear_clear(const ElementAnchor *a, const AppParam *p, const SideFeedback *fb,
                                 bool stable, uint32_t now);
bool vision_target_step(const VisionFeatures *f, const TrackDecision *d, const AppParam *p,
                        VisionOutput *out);
bool vision_pipeline_step(TrackContext *track, const FrameView *frame, const AppParam *p,
                          const SideFeedback *feedback, bool commit, uint32_t session, bool race,
                          VisionFeatures *features, VisionOutput *out);
#endif
