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
    assert(out.has_p0 && out.p0_x_px==u);
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
    AppParam short_preview=params, long_preview=params;
    short_preview.turn_preview_px=35;
    long_preview.turn_preview_px=90;
    VisionPoint short_points[3], long_points[3];
    VisionOutput short_out, long_out;
    team_vision_reset();
    assert(team_vision_step(&frame,&short_preview,9,&short_out,short_points));
    team_vision_reset();
    assert(team_vision_step(&frame,&long_preview,9,&long_out,long_points));
    assert(short_points[0].x==long_points[0].x && short_points[0].y==long_points[0].y);
    assert(short_points[1].y>long_points[1].y && short_points[2].y>long_points[2].y);
    assert(fabsf(long_out.heading_rad-straight_heading)>fabsf(short_out.heading_rad-straight_heading));

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
    for (int turn=-1;turn<=1;turn+=2) {
        VisionOutput projected={.session_id=9,.frame_id=1,.speed_cap_mps=1};
        const float far_x[3]={94,94,94+60*turn}, far_y[3]={100,80,80};
        assert(vision_project_path(&entry,far_x,far_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(!projected.near_transverse && fabsf(projected.heading_rad)<1e-5f);
        assert(projected.corner_direction==-turn);
        path_control_reset(&control,9);
        assert(control_path_step(&control,&projected,&entry,&request) && request.corner_direction==-turn);
        assert(projected.speed_cap_mps==1);
        const float distant_corner_y[3]={80,40,40};
        assert(vision_project_path(&entry,far_x,distant_corner_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(projected.corner_direction==0);
        const float far_transverse_y[3]={100,80,94};
        assert(vision_project_path(&entry,far_x,far_transverse_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(!projected.near_transverse && fabsf(projected.heading_rad)<1e-5f);
        const float far_reverse_y[3]={100,80,96};
        assert(vision_project_path(&entry,far_x,far_reverse_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(projected.heading_only && !projected.near_transverse && projected.curvature==0);
        assert(fabsf(projected.heading_rad)<1e-5f && projected.speed_cap_mps==entry.speed_corner);
        const float near_x[3]={94,94+30*turn,94+60*turn}, near_y[3]={100,100,100};
        assert(vision_project_path(&entry,near_x,near_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(projected.near_transverse && projected.curvature==0);
        assert(fabsf(projected.heading_rad-turn*1.57079633f)<1e-5f);
        path_control_reset(&control,9);
        assert(control_path_step(&control,&projected,&entry,&request));
        assert(-turn*request.omega_raw>0 && request.speed_cap_mps==entry.speed_corner);
        const float slight_back_y[3]={100,101,101};
        assert(vision_project_path(&entry,near_x,slight_back_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(projected.near_transverse && fabsf(projected.heading_rad-turn*1.57079633f)<1e-5f);
        const float near_transverse_y[3]={100,114,114};
        assert(vision_project_path(&entry,near_x,near_transverse_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(projected.near_transverse && projected.curvature==0);
        const float reverse_y[3]={100,116,116};
        assert(vision_project_path(&entry,near_x,reverse_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(projected.heading_only && projected.curvature==0 && turn*projected.heading_rad>1.5f);
        const float mostly_backward_x[3]={94,94+10*turn,94+60*turn};
        assert(vision_project_path(&entry,mostly_backward_x,near_transverse_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(projected.heading_only && projected.curvature==0 && turn*projected.heading_rad>1.5f);
        const float folded_x[3]={94,94+60*turn,94+20*turn};
        assert(vision_project_path(&entry,folded_x,near_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(projected.heading_only && projected.curvature==0 && turn*projected.heading_rad>1.5f);
        const float duplicate_x[3]={94,94,94+60*turn};
        assert(vision_project_path(&entry,duplicate_x,near_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(projected.heading_only && projected.curvature==0 && turn*projected.heading_rad>1.5f);
        const float missing_far_x[3]={94,94+30*turn,APP_WIDTH};
        assert(vision_project_path(&entry,missing_far_x,near_y,VISION_FORWARD_OFFSET_M,&projected));
        assert(projected.heading_only && projected.curvature==0 && turn*projected.heading_rad>1.5f);
        path_control_reset(&control,9);
        assert(control_path_step(&control,&projected,&entry,&request));
        assert(-turn*request.omega_raw>0 && request.speed_cap_mps==entry.speed_corner);
    }
    const float one_point_x[3]={94,APP_WIDTH,APP_WIDTH}, repeated_x[3]={94,94,94};
    const float no_points_x[3]={APP_WIDTH,APP_WIDTH,APP_WIDTH}, same_y[3]={100,100,100};
    VisionOutput fallback={.speed_cap_mps=1};
    assert(vision_project_path(&entry,one_point_x,same_y,VISION_FORWARD_OFFSET_M,&fallback));
    assert(fallback.heading_only && fallback.curvature==0 && fallback.heading_rad==0);
    assert(vision_project_path(&entry,repeated_x,same_y,VISION_FORWARD_OFFSET_M,&fallback));
    assert(fallback.heading_only && fallback.curvature==0 && fallback.heading_rad==0);
    assert(!vision_project_path(&entry,no_points_x,same_y,VISION_FORWARD_OFFSET_M,&fallback));
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
