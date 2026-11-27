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
    TeamPixel points[3], near_mid;
    int element, stage;
    float preview_px = params->turn_preview_px;
    if (!(preview_px >= APP_TURN_PREVIEW_MIN_PX && preview_px <= APP_TURN_PREVIEW_MAX_PX))
        return false;
    bool has_path = team_vision_points(frame,(int)preview_px,points,&near_mid,&element,&stage);
    if (element >= 0)
        out->element_debug = (VisionElementDebug){(uint8_t)element, (uint8_t)stage, true};
    if (!has_path) return false;
    out->reused_path = stage == 0;

    out->state=TRACK_NORMAL;
    out->speed_cap_mps=(element>=1 && element<=6) ? params->speed_element : params->speed_normal;
    /* The second output from v1.8 is a fresh/reused flag, not an element phase. */
    switch (element) {
    case 1:
        out->state=TRACK_CROSS_PASS;
        break;
    case 2:
        out->state=TRACK_Y_MERGE;
        break;
    case 3:
        out->state=TRACK_ISLAND_ENTER;
        break;
    case 4:
        out->state=TRACK_ISLAND_EXIT;
        break;
    case 5:
        out->state=TRACK_Y_CLEAR;
        break;
    }
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
    out->has_p0=u[0]>=0 && u[0]<APP_WIDTH && v[0]>=0 && v[0]<APP_HEIGHT;
    out->p0_x_px=u[0];
    /* Only use the near section for feedforward on a complete path that supports curvature. */
    float near_u[3]={u[0],near_mid.x,u[1]};
    float near_v[3]={v[0],near_mid.y,v[1]};
    VisionOutput near={.speed_cap_mps=out->speed_cap_mps};
    out->curvature=!out->near_transverse && !out->heading_only &&
                   vision_project_path(params,near_u,near_v,VISION_FORWARD_OFFSET_M,&near)
                   ? near.curvature : 0;
    out->confidence=60;
    return true;
}
