#include "A_vision/vision_all.h"
#include "control_api.h"
#include "vehicle_calibration.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t pixels[APP_HEIGHT][APP_WIDTH+4];

static void stripe(unsigned center) {
    memset(pixels,15,sizeof pixels);
    for (unsigned y=0;y<APP_HEIGHT;y++)
        memset(&pixels[y][center-5],220,11);
}

int main(void) {
    AppParam params;
    param_defaults(&params);
    params.ground_h0=CAL_GROUND_H0; params.ground_h1=CAL_GROUND_H1;
    params.ground_h2=CAL_GROUND_H2; params.ground_h3=CAL_GROUND_H3;
    params.ground_h4=CAL_GROUND_H4; params.ground_h5=CAL_GROUND_H5;
    params.ground_h6=CAL_GROUND_H6; params.ground_h7=CAL_GROUND_H7;
    params.lateral_kp=1;
    FrameView frame={&pixels[0][0],APP_WIDTH,APP_HEIGHT,sizeof pixels[0],1,100};
    VisionOutput out;
    stripe(112);
    team_vision_reset();
    assert(team_vision_step(&frame,&params,9,&out));
    assert(pixels[0][112]==220 && pixels[0][0]==15);
    assert(out.session_id==9 && out.frame_id==1 && out.capture_ms==100);
    assert(out.lateral_m>0 && out.forward_m>0 && fabsf(out.curvature)<.02f);
    PathControlState control;
    MotionRequest request;
    path_control_reset(&control,9);
    assert(control_path_step(&control,&out,&params,&request));
    assert(request.omega_raw<0 && request.speed_cap_mps==params.speed_normal);
    frame.frame_id++;
    params.ground_h2 -= .2f;
    assert(team_vision_step(&frame,&params,9,&out) && out.lateral_m<0);
    memset(pixels,0,sizeof pixels);
    frame.frame_id++;
    assert(!team_vision_step(&frame,&params,9,&out) && !out.valid && out.frame_id==3);
    assert(!control_path_step(&control,&out,&params,&request) && !request.valid);
    stripe(112);
    params.ground_h0=NAN;
    assert(!team_vision_step(&frame,&params,9,&out) && !out.valid);
    frame.stride=128;
    assert(!team_vision_step(&frame,&params,9,&out));
    puts("PASS: teammate vision to real steering, config projection, invalid frame rejection");
    return 0;
}
