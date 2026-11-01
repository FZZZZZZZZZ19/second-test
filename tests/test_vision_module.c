#include "vision_api.h"
#include "vision_tuning.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint8_t image[APP_WIDTH * APP_HEIGHT];
int main(void) {
    AppParam p = {0};
    vision_param_defaults(&p);
    p.speed_normal = .2f;
    for (unsigned y = 0; y < APP_HEIGHT; y++)
        for (unsigned x = 110; x < 120; x++)
            image[y * APP_WIDTH + x] = 200;
    FrameView frame = {image, APP_WIDTH, APP_HEIGHT, APP_WIDTH, 1, 100};
    SideFeedback progress = {0};
    TrackContext track = {0};
    VisionFeatures features;
    VisionOutput out;
    assert(vision_pipeline_step(&track, &frame, &p, &progress, false, 7, false, &features, &out));
    assert(out.valid && out.session_id == 7 && out.direction_rad > 0 && out.distance_px > 0);
    memset(image, 0, sizeof(image));
    frame.frame_id++;
    frame.capture_ms += 100;
    assert(!vision_pipeline_step(&track, &frame, &p, &progress, false, 7, false, &features, &out));
    assert(!out.valid && out.frame_id == 2 && out.session_id == 7);
    puts("PASS: standalone vision, no control or board objects linked");
    return 0;
}
