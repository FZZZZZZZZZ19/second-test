/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Exercise the production TIM6 body with deterministic board and I/O stubs. */
#define VEHICLE_CALIBRATION_H
#define CAL_FULL_RACE 0
#define CAL_VISION_V2_VALIDATED 1
#include "../User/control/app_control.c"
#include <assert.h>
#include <stdio.h>

AppParam g_app_param;
static uint32_t micros, request;
static int imu_result;
static float gyro;
static MotionRequest incoming;
static ControlSnapshot snapshot;
static bool braking;

void board_clock_tick(void) { micros += 5000; }
uint32_t board_time_us(void) { return micros; }
uint32_t board_time_ms(void) { return micros / 1000; }
BoardIrqToken board_irq_save(void) { return 0; }
void board_irq_restore(BoardIrqToken token) { (void)token; }
int board_imu_read(bool full, float *z, float *norm, float *accel) {
    (void)full;
    micros += 600;
    *z = gyro;
    *norm = fabsf(gyro);
    *accel = 1;
    return imu_result;
}
bool side_encoder_sample(float dt, uint32_t now, SideFeedback *feedback) {
    (void)dt;
    *feedback = (SideFeedback){.stamp_ms = now, .valid = true};
    return true;
}
void app_exchange_read_motion(MotionRequest *out) { *out = incoming; }
void app_exchange_publish_control(const ControlSnapshot *out) { snapshot = *out; }
uint32_t app_exchange_take_requests(void) {
    uint32_t result = request;
    request = 0;
    return result;
}
void motor_brake_zero(void) { braking = true; }
void motor_apply_forward(float left, float right) {
    (void)left;
    (void)right;
    braking = false;
}

static void boot(bool imu_ok) {
    micros = request = 0;
    imu_result = 1;
    gyro = .01f;
    incoming = (MotionRequest){0};
    app_control_init();
    app_control_camera_ready(true);
    app_control_imu_ready(imu_ok);
}
static void arm_and_start(void) {
    for (unsigned i = 0; i < 799; i++)
        app_control_tick_isr();
    request = REQ_READY;
    app_control_tick_isr();
    assert(snapshot.imu_calibrated && snapshot.imu_cal_samples == 800);
    assert(snapshot.mode == RUN_READY);
    incoming = (MotionRequest){.session_id = snapshot.session_id,
                               .capture_ms = board_time_ms(), .speed_cap_mps = .2f,
                               .omega_raw = .2f, .confidence = 100, .valid = true};
    request = REQ_START;
    app_control_tick_isr();
    assert(snapshot.mode == RUN_RUNNING && !braking);
    assert(fabsf(snapshot.omega_measured) < 1e-5f && snapshot.imu_max_us == 600);
}
int main(void) {
    param_defaults(&g_app_param);
    g_app_param.wheel_left_m = g_app_param.wheel_right_m = .065f;
    g_app_param.track_m = g_app_param.axle_m = .14f;
    g_app_param.counts_left = g_app_param.counts_right = 1000;
    g_app_param.row_distance_near_m = .05f;
    g_app_param.row_distance_far_m = .5f;
    g_app_param.calibration_mask = APP_CAL_REQUIRED;
    g_app_param.flags = 2;
    assert(param_can_arm(&g_app_param, false));

    boot(false);
    request = REQ_READY;
    app_control_tick_isr();
    assert(snapshot.mode == RUN_STATIC_DEBUG && !snapshot.config_ok && braking);
    boot(true);
    request = REQ_READY;
    app_control_tick_isr();
    assert(snapshot.mode == RUN_STATIC_DEBUG && !snapshot.config_ok && braking);
    boot(true);
    arm_and_start();
    imu_result = -1;
    app_control_tick_isr();
    assert(snapshot.mode == RUN_FAULT && snapshot.reason == STOP_IMU && braking);
    assert(!snapshot.config_ok);

    boot(true);
    arm_and_start();
    imu_result = 0;
    for (unsigned i = 0; i < 5 && snapshot.mode == RUN_RUNNING; i++)
        app_control_tick_isr();
    assert(snapshot.mode == RUN_FAULT && snapshot.reason == STOP_IMU && braking);
    puts("PASS: TIM6 IMU initialization, calibration gate, read failure and stale-sample stop");
    return 0;
}
