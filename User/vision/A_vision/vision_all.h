#ifndef TEAM_VISION_ALL_H
#define TEAM_VISION_ALL_H

#include "vision_control.h"

typedef struct { int x, y; } TeamPixel;
void team_vision_reset(void);
bool team_vision_points(const FrameView *frame, int preview_px, TeamPixel points[3],
                        TeamPixel *near_mid, int *element, int *stage);
bool team_vision_step(const FrameView *frame, const AppParam *params, uint32_t session,
                      VisionOutput *out, VisionPoint preview[3]);
#endif
