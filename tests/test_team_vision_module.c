#include "A_vision/vision_all.h"
#include "control_api.h"
#include "vision_api.h"
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
    VisionPoint preview[3];
    assert(team_vision_step(&frame,&params,9,&out,preview));
    assert(out.element_debug.available && out.element_debug.state==0 && out.element_debug.flag==0);
    assert(pixels[0][112]==220 && pixels[0][0]==15);
    assert(out.session_id==9 && out.frame_id==1 && out.capture_ms==100);
    float u=preview[0].x, v=preview[0].y;
    float original_z=(params.ground_h3*u+params.ground_h4*v+params.ground_h5) /
                     (params.ground_h6*u+params.ground_h7*v+1);
    assert(out.lateral_m>0 && original_z>.0355f && out.forward_m>0);
    assert(fabsf(out.forward_m-(original_z-.0355f))<1e-6f && fabsf(out.curvature)<.02f);
    float straight_heading=out.heading_rad;
    PathControlState control;
    MotionRequest request;
    path_control_reset(&control,9);
    assert(control_path_step(&control,&out,&params,&request));
    assert(request.omega_raw<0 && request.speed_cap_mps==params.speed_normal);
    /* The far bend must not trigger near curvature while the car is on the straight. */
    for (unsigned y=0;y<85;y++) {
        memset(pixels[y],15,sizeof pixels[y]);
        unsigned bend=(85-y)/2;
        if (bend>35) bend=35;
        memset(&pixels[y][107+bend],220,11);
    }
    frame.frame_id++;
    team_vision_reset();
    assert(team_vision_step(&frame,&params,9,&out,preview));
    float px[3],py[3];
    for (unsigned i=0;i<3;i++) { px[i]=preview[i].x; py[i]=preview[i].y; }
    VisionOutput full={0};
    assert(vision_project_path(&params,px,py,VISION_FORWARD_OFFSET_M,&full));
    assert(fabsf(full.curvature)>.1f && fabsf(out.curvature)<.02f);
    assert(fabsf(out.heading_rad-straight_heading)<.001f);

    /* At entry the near point can be centred while the ground path
     * already heads into the bend. An offset PD cannot see that angle. */
    AppParam entry=params;
    entry.ground_h0=.001f; entry.ground_h1=0; entry.ground_h2=-.094f;
    entry.ground_h3=0; entry.ground_h4=-.002f; entry.ground_h5=.25f;
    entry.ground_h6=0; entry.ground_h7=0;
    const float entry_y[3]={100,80,60};
    for (int turn=-1;turn<=1;turn++) {
        const float entry_x[3]={94,94+20*turn,94+40*turn};
        VisionOutput projected={.session_id=9,.frame_id=1,.speed_cap_mps=1};
        assert(vision_project_path(&entry,entry_x,entry_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(fabsf(projected.curvature)<.001f && fabsf(projected.lateral_m)<.001f);
        path_control_reset(&control,9);
        assert(control_path_step(&control,&projected,&entry,&request));
        if (turn)
            assert(-turn*request.omega_raw>.8f);
        else
            assert(fabsf(request.omega_raw)<.001f);
    }
    stripe(112);
    frame.frame_id++;
    params.ground_h2 -= .2f;
    assert(team_vision_step(&frame,&params,9,&out,NULL) && out.lateral_m<0);
    memset(pixels,0,sizeof pixels);
    frame.frame_id++;
    assert(!team_vision_step(&frame,&params,9,&out,NULL) && !out.valid && out.frame_id==4);
    assert(!out.element_debug.available);
    assert(!control_path_step(&control,&out,&params,&request) && !request.valid);
    stripe(112);
    params.ground_h0=NAN;
    assert(!team_vision_step(&frame,&params,9,&out,NULL) && !out.valid);
    assert(out.element_debug.available); /* Classification still exists when ground projection fails. */
    frame.stride=128;
    assert(!team_vision_step(&frame,&params,9,&out,NULL));
    puts("PASS: teammate vision to real steering, config projection, invalid frame rejection");
    return 0;
}
