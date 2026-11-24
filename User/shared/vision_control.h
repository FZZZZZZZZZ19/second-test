#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H
#include "app_types.h"
#include <stdbool.h>
#include <stdint.h>
#define VISION_FORWARD_OFFSET_M 0.0355f

typedef struct {
    uint8_t state, flag;
    bool available;
} VisionElementDebug;

typedef struct {
    uint32_t frame_id, capture_ms, session_id;
    float direction_rad, distance_px, speed_cap_mps;
    TrackState state;
    uint8_t confidence;
    bool valid, stop_requested, reset_derivative;
    /* Vehicle ground coordinates: right positive, forward positive; curvature right positive. */
    float lateral_m, forward_m, curvature; /* forward_m is signed after origin shift. */
    float heading_rad; /* Ground direction from near to middle sample, right positive. */
    VisionElementDebug element_debug;
    bool near_transverse; /* Close segment is sideways and uses heading-only steering geometry. */
    bool heading_only; /* Incomplete or degenerate geometry uses direction without curvature. */
} VisionOutput;
#endif
