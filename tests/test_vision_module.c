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
    p.ground_h0=.001f; p.ground_h2=-.0935f; p.ground_h4=-.004f; p.ground_h5=.6f;
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
    assert(out.valid && out.session_id == 7 && out.lateral_m > 0 && out.forward_m > 0 && fabsf(out.curvature)<.001f);
    AppParam missing=p; missing.ground_h0=0; missing.ground_h2=0;
    TrackDecision decision={0}; decision.valid=true; decision.speed_cap=.2f;
    assert(!vision_target_step(&features,&decision,&missing,&out) && !out.valid);
    missing=p; missing.ground_h5=NAN;
    decision.stop=true;
    assert(!vision_target_step(&features,&decision,&missing,&out) && out.stop_requested);
    decision.stop=false;
    /* Synthetic calibrated parabolic paths: verify curvature sign and scale. */
    VisionFeatures curve={0}; curve.path_count=1;
    for(unsigned i=0;i<APP_ROWS;i++) {
        float z=.6f-.004f*(APP_HEIGHT-3u-i*APP_ROW_STEP);
        curve.paths[0].valid_rows|=1u<<i;
        curve.paths[0].x[i]=93.5f+100*z*z;
    }
    assert(vision_target_step(&curve,&decision,&p,&out) && out.curvature>0);
    float k=out.curvature;
    for(unsigned i=0;i<APP_ROWS;i++) curve.paths[0].x[i]=187-curve.paths[0].x[i];
    assert(vision_target_step(&curve,&decision,&p,&out) && fabsf(out.curvature+k)<1e-4f);
    AppParam scaled=p; scaled.ground_h0*=2; scaled.ground_h2*=2;
    scaled.ground_h4*=2; scaled.ground_h5*=2;
    assert(vision_target_step(&curve,&decision,&scaled,&out) && fabsf(out.curvature+k/2)<1e-4f);
    curve.paths[0].valid_rows=3;
    assert(!vision_target_step(&curve,&decision,&p,&out));
    memset(image, 0, sizeof(image));
    frame.frame_id++;
    frame.capture_ms += 100;
    assert(!vision_pipeline_step(&track, &frame, &p, &progress, false, 7, false, &features, &out));
    assert(!out.valid && out.frame_id == 2 && out.session_id == 7);
    puts("PASS: standalone vision, no control or board objects linked");
    return 0;
}
