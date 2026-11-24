/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "../vision/A_vision/vision_all.h"
#include "vehicle_calibration.h"
#include "zf_common_headfile.h"
#include <math.h>
AppParam g_app_param;
static PathControlState path;
static uint32_t pipeline_session;
static TrackState previous_track_state=TRACK_DEGRADED;
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
    g_app_param.ground_h0 = CAL_GROUND_H0;
    g_app_param.ground_h1 = CAL_GROUND_H1;
    g_app_param.ground_h2 = CAL_GROUND_H2;
    g_app_param.ground_h3 = CAL_GROUND_H3;
    g_app_param.ground_h4 = CAL_GROUND_H4;
    g_app_param.ground_h5 = CAL_GROUND_H5;
    g_app_param.ground_h6 = CAL_GROUND_H6;
    g_app_param.ground_h7 = CAL_GROUND_H7;
    g_app_param.calibration_mask = CAL_MASK;
    g_app_param.flags = (CAL_CAPTURE_VALIDATED ? 2u : 0u) | (CAL_FULL_RACE ? 1u : 0u);
    (void)param_store_load(&g_app_param);
    (void)side_encoder_init();
    app_control_init();
    pit_ms_init(TIM6_PIT, APP_CONTROL_PERIOD_MS);
    /* Supplied working example initializes ST7735S before the camera. */
    debug_view_init();
    system_delay_ms(1000);
    bool ok = camera_frame_init();
    app_control_camera_ready(ok);
    (void)camera_frame_set_running(false);
    app_gyro_init();
    return ok;
}
void app_main_poll(void) {
    ControlSnapshot snapshot;
    app_exchange_read_control(&snapshot);
    app_gyro_poll(&snapshot);
    if (pipeline_session != snapshot.session_id) {
        pipeline_session = snapshot.session_id;
        team_vision_reset();
        previous_track_state=TRACK_DEGRADED;
        path_control_reset(&path, pipeline_session);
    }
    FrameView frame;
    if (camera_frame_acquire(&frame)) {
        VisionOutput observation;
        VisionPoint preview[3];
        (void)team_vision_step(&frame, &g_app_param, snapshot.session_id, &observation, preview);
        if (snapshot.mode != RUN_RUNNING && snapshot.mode != RUN_TUNE_RUNNING)
            team_vision_reset();
        if ((snapshot.mode == RUN_TUNE_WAIT || snapshot.mode == RUN_TUNE_RUNNING) &&
            observation.valid && !observation.stop_requested)
            observation.speed_cap_mps = observation.near_transverse || observation.heading_only
                ? fminf(g_app_param.tune_speed_mps, observation.speed_cap_mps)
                : g_app_param.tune_speed_mps;
        observation.reset_derivative=observation.state!=previous_track_state;
        previous_track_state=observation.state;
        MotionRequest request;
        (void)control_path_step(&path, &observation, &g_app_param, &request);
        app_exchange_publish_motion(&request);
        debug_view_frame(&frame, &snapshot, preview, observation.element_debug);
        camera_frame_release(frame.frame_id);
    }
    debug_view_poll(&snapshot);
}
