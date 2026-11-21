#include "A_vision/vision_all.h"
#include "vision_api.h"

bool team_vision_step(const FrameView *frame, const AppParam *params, uint32_t session,
                      VisionOutput *out, VisionPoint preview[3]) {
    if (preview)
        for (unsigned i=0;i<3;i++) preview[i]=(VisionPoint){255,255};
    if (!out) return false;
    *out=(VisionOutput){.session_id=session,.state=TRACK_DEGRADED};
    if (!frame || !params) return false;
    out->frame_id=frame->frame_id;
    out->capture_ms=frame->capture_ms;
    TeamPixel points[3];
    int element, stage;
    if (!team_vision_points(frame,points,&element,&stage)) return false;

    out->state=TRACK_NORMAL;
    out->speed_cap_mps=params->speed_normal;
    switch (element) {
    case 1:
        out->speed_cap_mps=params->speed_corner;
        break;
    case 2:
        out->state=stage==2 ? TRACK_Y_COMMITTED : TRACK_Y_MERGE;
        break;
    case 3:
        out->state=TRACK_CROSS_PASS;
        break;
    case 4:
        out->state=stage<=6 ? TRACK_ISLAND_ENTER :
                   stage<=8 ? TRACK_ISLAND_FOLLOW : TRACK_ISLAND_EXIT;
        break;
    }
    if (element>=2) out->speed_cap_mps=params->speed_element;
    float u[3],v[3];
    for (unsigned i=0;i<3;i++) {
        u[i]=points[i].x;
        v[i]=points[i].y;
        if (preview && u[i]>=0 && u[i]<APP_WIDTH && v[i]>=0 && v[i]<APP_HEIGHT)
            preview[i]=(VisionPoint){points[i].x,points[i].y};
    }
    if (!vision_project_path(params,u,v,VISION_FORWARD_OFFSET_M,out)) {
        out->state=TRACK_DEGRADED;
        return false;
    }
    out->confidence=60;
    return true;
}
