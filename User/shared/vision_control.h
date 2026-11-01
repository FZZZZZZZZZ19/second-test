#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H
#include "app_types.h"
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t frame_id, capture_ms, session_id;
    float direction_rad, distance_px, speed_cap_mps;
    TrackState state;
    uint8_t confidence;
    bool valid, stop_requested, reset_derivative;
} VisionOutput;
#endif
