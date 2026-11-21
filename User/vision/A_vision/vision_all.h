#ifndef TEAM_VISION_ALL_H
#define TEAM_VISION_ALL_H

#include "vision_control.h"

typedef struct { int x, y; } TeamPixel;
void team_vision_reset(void);
bool team_vision_points(const FrameView *frame, TeamPixel points[3], int *element, int *stage);
bool team_vision_step(const FrameView *frame, const AppParam *params, uint32_t session,
                      VisionOutput *out, VisionPoint preview[3]);
#endif
