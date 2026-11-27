#include "A_vision/vision_all.h"
#include "control_api.h"
#include <assert.h>
#include <stdio.h>

static int raw_state, fresh_flag;

bool team_vision_points(const FrameView *frame, int preview_px, TeamPixel points[3],
                        TeamPixel *near_mid, int *element, int *stage) {
    (void)frame;
    (void)preview_px;
    points[0]=(TeamPixel){94,100};
    points[1]=(TeamPixel){94,80};
    points[2]=(TeamPixel){94,60};
    *near_mid=(TeamPixel){94,90};
    *element=raw_state;
    *stage=fresh_flag;
    return true;
}

int main(void) {
    const TrackState expected[]={
        TRACK_NORMAL, TRACK_CROSS_PASS, TRACK_Y_MERGE, TRACK_ISLAND_ENTER,
        TRACK_ISLAND_EXIT, TRACK_Y_CLEAR, TRACK_NORMAL
    };
    AppParam p;
    param_defaults(&p);
    p.ground_h0=.001f;
    p.ground_h2=-.094f;
    p.ground_h4=-.002f;
    p.ground_h5=.25f;
    FrameView frame={.frame_id=1,.capture_ms=100};
    for (raw_state=0;raw_state<(int)(sizeof expected/sizeof expected[0]);raw_state++) {
        for (fresh_flag=0;fresh_flag<=1;fresh_flag++) {
            VisionOutput out;
            assert(team_vision_step(&frame,&p,9,&out,NULL));
            assert(out.state==expected[raw_state]);
            assert(out.speed_cap_mps==(raw_state ? p.speed_element : p.speed_normal));
            assert(out.element_debug.available && out.element_debug.state==raw_state);
            assert(out.element_debug.flag==fresh_flag && out.reused_path==!fresh_flag);
            PathControlState control;
            MotionRequest request;
            path_control_reset(&control,9);
            assert(control_path_step(&control,&out,&p,&request));
            assert(request.state==expected[raw_state] && request.reused_path==!fresh_flag);
        }
    }
    puts("PASS: v1.8 element and frame freshness mappings through the control interface");
    return 0;
}
