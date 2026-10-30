/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "track_profile.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned assertions, groups;
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        assertions++;                                                                              \
        if (!(x)) {                                                                                \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x);                           \
            exit(1);                                                                               \
        }                                                                                          \
    } while (0)
#define NEAR(a, b) CHECK(fabsf((a) - (b)) < .0001f)
static AppParam fixture(void) {
    AppParam p;
    param_defaults(&p);
    p.wheel_left_m = p.wheel_right_m = .065f;
    p.track_m = .14f;
    p.axle_m = .08f;
    p.counts_left = p.counts_right = 1000;
    p.row_distance_near_m = .05f;
    p.row_distance_far_m = .5f;
    p.camera_forward_m = .1f;
    p.calibration_mask = 31;
    p.flags = 2;
    return p;
}
static void counter_and_crc(void) {
    for (unsigned prev = 0; prev < 65536; prev++) {
        CHECK(app_count_delta((uint16_t)(prev + 123), (uint16_t)prev) == 123);
        CHECK(app_count_delta((uint16_t)(prev - 123), (uint16_t)prev) == -123);
    }
    CHECK(app_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u);
    groups++;
}
static void parameters(void) {
    AppParam p = fixture(), q;
    uint8_t data[APP_PARAM_RECORD_BYTES];
    uint32_t sequence;
    CHECK(param_valid(&p));
    CHECK(param_can_arm(&p, false));
    CHECK(!param_can_arm(&p, true));
    p.calibration_mask &= ~APP_CAL_DIRECTION;
    CHECK(!param_can_arm(&p, false));
    p.calibration_mask = 31;
    CHECK(param_encode(&p, 0xffffffffu, data));
    CHECK(param_decode(data, &q, &sequence));
    CHECK(sequence == 0xffffffffu);
    NEAR(q.counts_left, p.counts_left);
    for (unsigned i = 0; i < APP_PARAM_RECORD_BYTES; i++) {
        data[i] ^= 1;
        CHECK(!param_decode(data, &q, &sequence));
        data[i] ^= 1;
    }
    p.wheel_left_m = NAN;
    CHECK(!param_valid(&p));
    param_defaults(&p);
    CHECK(!param_can_arm(&p, false));
    groups++;
}
static void pi_and_motion(void) {
    SpeedPiState pi = {0};
    float out = 0;
    for (unsigned i = 0; i < 1000; i++)
        CHECK(speed_pi_step(&pi, 1, 0, .005f, 1, 2, 0, .5f, .25f, &out));
    NEAR(out, .25f);
    NEAR(pi.integral, 0);
    CHECK(speed_pi_step(&pi, 0, 1, .005f, 1, 2, 0, .5f, .25f, &out));
    NEAR(out, 0);
    CHECK(!speed_pi_step(&pi, NAN, 0, .005f, 1, 2, 0, .5f, .25f, &out));
    NEAR(out, 0);
    CHECK(!speed_pi_step(&pi, 0, 0, 0, 1, 2, 0, .5f, .25f, &out));
    AppParam p = fixture();
    MotionPlanState plan = {0};
    SideTargets target;
    for (unsigned i = 0; i < 1500; i++) {
        float a = plan.left, b = plan.right;
        float requested = i < 500 ? .3f : 0, omega = i % 2 ? 100 : -100;
        CHECK(motion_plan_step(&plan, &p, requested, omega, .005f, false, &target));
        CHECK(plan.left >= 0 && plan.right >= 0 && plan.left <= p.speed_limit_mps + .00001f &&
              plan.right <= p.speed_limit_mps + .00001f);
        CHECK(plan.left - a <= p.accel_mps2 * .005f + .00001f &&
              plan.right - b <= p.accel_mps2 * .005f + .00001f);
        CHECK(a - plan.left <= p.decel_mps2 * .005f + .00001f &&
              b - plan.right <= p.decel_mps2 * .005f + .00001f);
    }
    NEAR(plan.left, 0);
    NEAR(plan.right, 0);
    p.speed_limit_mps = 1;
    p.accel_mps2 = p.decel_mps2 = 100;
    plan.left = plan.right = .6f;
    CHECK(motion_plan_step(&plan, &p, .6f, 1, .005f, false, &target));
    NEAR(plan.left, .53f);
    NEAR(plan.right, .67f);
    CHECK(!motion_plan_step(&plan, &p, NAN, 1, .005f, false, &target));
    for (unsigned i = 0; i < 5; i++)
        CHECK(motion_plan_step(&plan, &p, 0, 100, .005f, true, &target));
    NEAR(target.model_omega_radps, 0);
    groups++;
}
static uint8_t pixels[APP_WIDTH * APP_HEIGHT];
static void stripe(int x, int width) {
    for (unsigned y = 0; y < APP_HEIGHT; y++)
        for (int j = x; j < x + width; j++)
            if (j >= 0 && j < (int)APP_WIDTH)
                pixels[y * APP_WIDTH + (unsigned)j] = 255;
}
static void vision_and_path(void) {
    AppParam p = fixture();
    FrameView f = {pixels, APP_WIDTH, APP_HEIGHT, APP_WIDTH, 1, 100};
    VisionFeatures v;
    memset(pixels, 0, sizeof(pixels));
    stripe(108, 12);
    CHECK(vision_line_process(&f, &p, &v));
    CHECK(v.confidence == 100 && v.path_count == 1 && !v.split && !v.cross);
    TrackDecision d = {0};
    d.valid = true;
    d.speed_cap = .2f;
    PathControlState s;
    path_control_reset(&s, 7);
    MotionRequest request;
    CHECK(path_control_step(&s, &v, &d, &p, &request));
    CHECK(request.omega_raw < 0 && request.session_id == 7 && request.capture_ms == 100);
    CHECK(!path_control_step(&s, &v, &d, &p, &request));
    memset(pixels, 0, sizeof(pixels));
    f.frame_id++;
    CHECK(vision_line_process(&f, &p, &v));
    CHECK(v.confidence == 0);
    memset(pixels, 255, sizeof(pixels));
    f.frame_id++;
    CHECK(vision_line_process(&f, &p, &v));
    CHECK(v.confidence == 0);
    memset(pixels, 0, sizeof(pixels));
    for (int x = 10; x < 160; x += 30)
        stripe(x, 6);
    f.frame_id++;
    CHECK(vision_line_process(&f, &p, &v));
    CHECK(v.overflow && v.confidence == 0);
    memset(pixels, 0, sizeof(pixels));
    stripe(90, 9);
    for (unsigned y = 48; y < 68; y++)
        memset(pixels + y * APP_WIDTH, 255, APP_WIDTH);
    f.frame_id++;
    CHECK(vision_line_process(&f, &p, &v));
    CHECK(v.cross && !v.split);
    memset(pixels, 0, sizeof(pixels));
    for (unsigned y = 0; y < APP_HEIGHT; y++) {
        int gap = y > 80 ? 0 : (80 - (int)y) / 2;
        for (int x = 90 - gap; x < 99 - gap; x++)
            pixels[y * APP_WIDTH + (unsigned)x] = 255;
        for (int x = 90 + gap; x < 99 + gap; x++)
            pixels[y * APP_WIDTH + (unsigned)x] = 255;
    }
    f.frame_id++;
    CHECK(vision_line_process(&f, &p, &v));
    CHECK(v.split && v.path_count >= 2);
    groups++;
}
static VisionFeatures synthetic(uint32_t id, uint32_t ms, bool split) {
    VisionFeatures f = {0};
    f.frame_id = id;
    f.capture_ms = ms;
    f.confidence = 100;
    f.split = split;
    f.path_count = split ? 2 : 1;
    for (unsigned j = 0; j < f.path_count; j++) {
        f.paths[j].valid_rows = (1u << APP_ROWS) - 1;
        f.paths[j].count = APP_ROWS;
        for (unsigned i = 0; i < APP_ROWS; i++)
            f.paths[j].x[i] = split ? (j ? 130 : 50) : 94;
    }
    return f;
}
static void states_and_progress(void) {
    AppParam p = fixture();
    SideFeedback fb = {0};
    fb.valid = true;
    TrackContext t;
    TrackDecision d;
    track_element_reset(&t, 1, true, 1);
    t.configured = false;
    VisionFeatures f = synthetic(1, 100, true);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    CHECK(d.stop && t.state == TRACK_DEGRADED);
    track_element_reset(&t, 1, true, 1);
    t.configured = true;
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    CHECK(t.state == TRACK_Y_CONFIRM);
    f = synthetic(2, 200, true);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    CHECK(t.state == TRACK_Y_COMMITTED && d.path_index == 0);
    f = synthetic(3, 300, false);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    f = synthetic(4, 500, false);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    CHECK(t.state == TRACK_Y_MERGE);
    f = synthetic(5, 600, false);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    CHECK(t.state == TRACK_Y_CLEAR);
    f = synthetic(6, 800, false);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    CHECK(t.visits[0] == 0);
    fb.rolling_m[0] = fb.rolling_m[1] = 1;
    f = synthetic(7, 1000, false);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    CHECK(t.passed_mask[0] == 1 && t.visits[0] == 1);
    f = synthetic(8, 1100, true);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    f = synthetic(9, 1300, true);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    CHECK(t.choice == 1 && d.path_index == 1);
    track_element_reset(&t, 2, true, 1);
    t.state = TRACK_ISLAND_FOLLOW;
    t.entry_seen = true;
    t.entered_ms = 1000;
    t.entry_s = 1;
    f = synthetic(10, 1400, true);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    CHECK(t.state == TRACK_ISLAND_FOLLOW);
    fb.rolling_m[0] = fb.rolling_m[1] = 2;
    f = synthetic(11, 1500, true);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    f = synthetic(12, 1700, true);
    CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
    CHECK(t.state == TRACK_ISLAND_EXIT);
    if (APP_ROUTE_LENGTH > 0) {
        track_element_reset(&t, 2, true, 1);
        t.state = TRACK_Y_CLEAR;
        t.y_id = 0;
        t.choice = 1;
        t.passed_mask[0] = 1;
        t.event_index = 1;
        t.anchor.valid = true;
        t.anchor.stamp_ms = 1000;
        f = synthetic(13, 1900, false);
        CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
        CHECK(t.event_index == 2 && t.passed_mask[0] == 3 && t.finishing && !d.stop);
        fb.rolling_m[0] = fb.rolling_m[1] = 3;
        f = synthetic(14, 2100, false);
        CHECK(track_element_step(&t, &f, &p, &fb, true, false, 0, &d));
        CHECK(d.stop);
    }
    RunMode m = RUN_READY;
    StopReason reason = STOP_NONE;
    uint32_t session = 1;
    CHECK(!run_manager_step(&m, &reason, REQ_START, true, false, &session));
    CHECK(m == RUN_READY);
    CHECK(!run_manager_step(&m, &reason, REQ_START | REQ_STOP, true, true, &session));
    CHECK(m == RUN_STATIC_DEBUG);
    CHECK(run_manager_step(&m, &reason, REQ_READY, true, false, &session));
    CHECK(m == RUN_READY);
    CHECK(run_manager_step(&m, &reason, REQ_START, true, true, &session));
    CHECK(m == RUN_RUNNING);
    CHECK(!run_manager_step(&m, &reason, REQ_STOP, true, true, &session));
    CHECK(m == RUN_STOPPING);
    groups++;
}
int main(void) {
    counter_and_crc();
    parameters();
    pi_and_motion();
    vision_and_path();
    states_and_progress();
    printf("PASS: %u groups, %u assertions\n", groups, assertions);
    return 0;
}
