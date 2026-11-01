/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "vehicle_calibration.h"
#include "zf_common_headfile.h"
AppParam g_app_param;
static VisionFeatures features;
static TrackContext track;
static PathControlState path;
static uint32_t pipeline_session;
bool app_boot_init(void) {
    board_clock_init();
    board_controls_init();
    param_defaults(&g_app_param);
    g_app_param.wheel_left_m = CAL_WHEEL_LEFT_M;
    g_app_param.wheel_right_m = CAL_WHEEL_RIGHT_M;
    g_app_param.track_m = CAL_TRACK_M;
    g_app_param.axle_m = CAL_AXLE_M;
    g_app_param.counts_left = CAL_COUNTS_LEFT;
    g_app_param.counts_right = CAL_COUNTS_RIGHT;
    g_app_param.row_distance_near_m = CAL_ROW_NEAR_M;
    g_app_param.row_distance_far_m = CAL_ROW_FAR_M;
    g_app_param.camera_forward_m = CAL_CAMERA_FORWARD_M;
    g_app_param.calibration_mask = CAL_MASK;
    g_app_param.flags = (CAL_CAPTURE_VALIDATED ? 2u : 0u) | (CAL_FULL_RACE ? 1u : 0u);
    (void)param_store_load(&g_app_param);
    (void)side_encoder_init();
    app_control_init();
    pit_ms_init(TIM6_PIT, APP_CONTROL_PERIOD_MS);
    bool ok = camera_frame_init();
    app_control_camera_ready(ok);
    debug_view_init();
    return ok;
}
void app_main_poll(void) {
    ControlSnapshot snapshot;
    app_exchange_read_control(&snapshot);
    if (pipeline_session != snapshot.session_id) {
        pipeline_session = snapshot.session_id;
        track_element_reset(&track, pipeline_session, CAL_FULL_RACE != 0, 0);
        path_control_reset(&path, pipeline_session);
    }
    FrameView frame;
    if (camera_frame_acquire(&frame)) {
        VisionOutput observation;
        (void)vision_pipeline_step(&track, &frame, &g_app_param, &snapshot.feedback,
                                   snapshot.mode == RUN_RUNNING, snapshot.session_id,
                                   CAL_FULL_RACE != 0, &features, &observation);
        MotionRequest request;
        (void)control_path_step(&path, &observation, &g_app_param, &request);
        app_exchange_publish_motion(&request);
        debug_view_frame(&frame, &snapshot, &features);
        camera_frame_release(frame.frame_id);
    }
    debug_view_poll(&snapshot);
}
