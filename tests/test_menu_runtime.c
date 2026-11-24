
#include "app_runtime.h"
#include "yaw_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
AppParam g_app_param;
static uint32_t clock_us, requests;
static ControlSnapshot output;
static SideFeedback feedback;
static MotionRequest motion;
static float duty[2];
static float gyro_raw=.02f, gyro_norm=.02f;
static int gyro_result=1;
static unsigned gyro_reads;
BoardIrqToken board_irq_save(void) { return 0; }
void board_irq_restore(BoardIrqToken t) { (void)t; }
void board_clock_tick(void) {}
uint32_t board_time_us(void) { return clock_us; }
uint32_t board_time_ms(void) { return clock_us/1000; }
void app_exchange_publish_control(const ControlSnapshot *s) { output=*s; }
void app_exchange_read_motion(MotionRequest *r) { *r=motion; }
uint32_t app_exchange_take_requests(void) { uint32_t r=requests; requests=0; return r; }
bool side_encoder_sample(float dt, uint32_t now, SideFeedback *s) { (void)dt;(void)now;*s=feedback;return s->valid; }
bool board_imu_init(void) { return true; }
int board_imu_read(bool calibration,float *z,float *norm,float *accel) {
    (void)calibration;
    gyro_reads++;
    *z=gyro_raw; *norm=gyro_norm; *accel=1;
    return gyro_result;
}
void motor_brake_zero(void) { duty[0]=duty[1]=0; }
void motor_apply_forward(float l,float r) { duty[0]=l;duty[1]=r; }
void board_motor_write_en(SideId side,uint32_t d) { duty[side]=d/10000.f; }
static void tick(uint32_t req) { clock_us+=5000; requests=req; app_control_tick_isr(); }
static void publish_tune_frame(void) {
    static uint32_t frame_id;
    motion=(MotionRequest){.session_id=output.session_id,.frame_id=++frame_id,.capture_ms=board_time_ms(),
                           .valid=true,.confidence=60,.speed_cap_mps=1,.omega_raw=-.4f};
}
static void publish_lost_frame(void) {
    publish_tune_frame();
    motion.valid=false;
}
static void check_continuous_start(void) {
    feedback.valid=true;
    app_control_camera_ready(true);
    g_app_param.tune_duration_s=APP_TUNE_MIN_S;
    publish_tune_frame(); tick(REQ_TRACK_START);
    assert(output.mode==RUN_TUNE_WAIT && duty[0]==0 && duty[1]==0);
    publish_tune_frame(); tick(0);
    for (unsigned i=0;i<(unsigned)(APP_TUNE_MAX_S*1000)/APP_CONTROL_PERIOD_MS+1;i++) {
        publish_tune_frame(); tick(0);
        assert(output.mode==RUN_TUNE_RUNNING && output.reason==STOP_NONE && output.tune_remaining_ms==0);
    }
    assert(duty[0]>0 && duty[1]>0);
    feedback.speed_mps[0]=feedback.speed_mps[1]=.12f;
    motion.valid=false; tick(0);
    assert(output.mode==RUN_TUNE_WAIT && output.reason==STOP_VISION && duty[0]==0 && duty[1]==0);
    publish_lost_frame(); tick(0);
    assert(duty[0]==0 && duty[1]==0); /* Wait for the previous run to slow down. */
    feedback.speed_mps[0]=feedback.speed_mps[1]=0;
    for (unsigned i=0;i<80;i++) { publish_lost_frame(); tick(0); }
    assert(output.mode==RUN_TUNE_WAIT && duty[0]>0 && duty[1]>0);
    assert(fabsf(output.targets.left_mps-APP_START_SEARCH_SPEED_MPS)<1e-5f);
    assert(output.targets.left_mps==output.targets.right_mps);
    feedback.speed_mps[0]=feedback.speed_mps[1]=APP_START_SEARCH_SPEED_MPS;
    publish_tune_frame(); tick(0);
    for (unsigned i=0;i<5;i++) tick(0);
    assert(output.mode==RUN_TUNE_WAIT); /* One repeated frame cannot restore steering. */
    for (unsigned i=1;i<APP_TUNE_RETRY_FRAMES;i++) { publish_tune_frame(); tick(0); }
    for (unsigned i=0;i<200;i++) { publish_tune_frame(); tick(0); }
    assert(output.mode==RUN_TUNE_RUNNING && output.reason==STOP_NONE && output.tune_remaining_ms==0);
    publish_lost_frame(); tick(0);
    feedback.speed_mps[0]=feedback.speed_mps[1]=0;
    publish_lost_frame(); tick(0);
    assert(duty[0]>0 && duty[1]>0);
    feedback.rolling_m[0]+=APP_START_SEARCH_DISTANCE_M;
    publish_lost_frame(); tick(0);
    assert(output.mode==RUN_TUNE_WAIT && duty[0]==0 && duty[1]==0);
    publish_lost_frame(); tick(0);
    assert(duty[0]==0 && duty[1]==0); /* Travel limit does not restart the search. */
    for (unsigned i=0;i<APP_TUNE_RETRY_FRAMES;i++) { publish_tune_frame(); tick(0); }
    assert(output.mode==RUN_TUNE_RUNNING);
    publish_lost_frame(); tick(0);
    for (unsigned i=0;i<=APP_START_SEARCH_MAX_MS/APP_CONTROL_PERIOD_MS;i++) {
        publish_lost_frame(); tick(0);
    }
    assert(output.mode==RUN_TUNE_WAIT && duty[0]==0 && duty[1]==0); /* Stalled encoder distance. */
    for (unsigned i=0;i<APP_TUNE_RETRY_FRAMES;i++) { publish_tune_frame(); tick(0); }
    publish_lost_frame(); tick(0);
    publish_lost_frame(); tick(0);
    assert(duty[0]>0 && duty[1]>0);
    for (unsigned i=0;i<=APP_TUNE_FRAME_AGE_MS/APP_CONTROL_PERIOD_MS;i++) tick(0);
    assert(output.mode==RUN_TUNE_WAIT && duty[0]==0 && duty[1]==0); /* Camera stops delivering frames. */
    for (unsigned i=0;i<APP_TUNE_RETRY_FRAMES;i++) { publish_tune_frame(); tick(0); }
    publish_lost_frame(); tick(0);
    publish_lost_frame(); tick(0);
    assert(duty[0]>0 && duty[1]>0);
    tick(REQ_HALT);
    assert(output.mode==RUN_STATIC_DEBUG && duty[0]==0 && duty[1]==0);
}
static void check_tune(void) {
    feedback.valid=true;
    g_app_param.tune_duration_s=4.5f;
    g_app_param.tune_speed_mps=.65f;
    app_control_camera_ready(true);
    publish_tune_frame(); /* A valid frame from before the request cannot start the car. */
    tick(REQ_TRACK_TUNE);
    assert(!output.config_ok && output.mode==RUN_TUNE_WAIT && duty[0]==0 && duty[1]==0);
    tick(0);
    assert(output.mode==RUN_TUNE_WAIT && duty[0]==0 && duty[1]==0);
    publish_tune_frame(); tick(0);
    assert(output.mode==RUN_TUNE_RUNNING && output.tune_remaining_ms==4500);
    for (unsigned i=1;i<900;i++) {
        publish_tune_frame(); tick(0);
        assert(output.mode==RUN_TUNE_RUNNING && output.targets.model_v_mps<=g_app_param.tune_speed_mps+.001f);
    }
    assert(output.targets.left_mps>output.targets.right_mps); /* right turn */
    assert(fabsf(output.targets.model_v_mps-g_app_param.tune_speed_mps)<.001f);
    publish_tune_frame(); tick(0);
    assert(output.mode==RUN_STATIC_DEBUG && output.reason==STOP_TUNE_DONE && duty[0]==0 && duty[1]==0);
    tick(REQ_TRACK_TUNE); publish_tune_frame(); tick(0);
    uint32_t old_session=output.session_id;
    motion.valid=false; tick(0);
    assert(output.mode==RUN_TUNE_WAIT && output.reason==STOP_VISION && duty[0]==0 && duty[1]==0);
    assert(output.session_id!=old_session);
    publish_lost_frame(); tick(0);
    assert(duty[0]==0 && duty[1]==0); /* Timed tune retries from a standstill. */
    motion.session_id=old_session;
    motion.valid=true; tick(0);
    assert(output.mode==RUN_TUNE_WAIT); /* Ignore frames from the interrupted run. */
    feedback.speed_mps[0]=.1f;
    publish_tune_frame(); tick(0);
    assert(output.mode==RUN_TUNE_WAIT && duty[0]==0 && duty[1]==0);
    feedback.speed_mps[0]=0;
    publish_tune_frame(); tick(0);
    for (unsigned i=0;i<5;i++) tick(0);
    assert(output.mode==RUN_TUNE_WAIT); /* Re-reading one frame cannot restart. */
    motion.valid=false; tick(0);
    for (unsigned i=1;i<APP_TUNE_RETRY_FRAMES;i++) {
        publish_tune_frame(); tick(0);
        assert(output.mode==RUN_TUNE_WAIT && duty[0]==0 && duty[1]==0);
    }
    publish_tune_frame(); tick(0);
    assert(output.mode==RUN_TUNE_RUNNING && output.reason==STOP_NONE && output.tune_remaining_ms==4500);
    for (unsigned i=0;i<=APP_TUNE_FRAME_AGE_MS/5;i++) tick(0);
    assert(output.mode==RUN_TUNE_WAIT && output.reason==STOP_VISION && duty[0]==0 && duty[1]==0);
    for (unsigned i=1;i<APP_TUNE_RETRY_FRAMES;i++) { publish_tune_frame(); tick(0); }
    publish_tune_frame(); tick(REQ_HALT); /* Back wins on the frame that could restart. */
    assert(output.mode==RUN_STATIC_DEBUG && output.reason==STOP_LOCAL && duty[0]==0 && duty[1]==0);
    for (unsigned i=0;i<APP_TUNE_RETRY_FRAMES;i++) { publish_tune_frame(); tick(0); }
    assert(output.mode==RUN_STATIC_DEBUG && duty[0]==0 && duty[1]==0);
    tick(REQ_TRACK_TUNE);
    for (unsigned i=0;i<=APP_TUNE_WAIT_MS/5;i++) tick(0);
    assert(output.mode==RUN_TUNE_WAIT && output.reason==STOP_VISION);
    feedback.valid=false; tick(0);
    assert(output.mode==RUN_FAULT && output.reason==STOP_ENCODER && duty[0]==0 && duty[1]==0);
    feedback.valid=true;
    tick(REQ_RESET); tick(REQ_TRACK_TUNE); publish_tune_frame(); tick(0);
    g_app_param.speed_limit_mps=.25f;
    for (unsigned i=0;i<200;i++) {
        publish_tune_frame(); tick(0);
        assert(output.mode==RUN_TUNE_RUNNING);
        assert(output.targets.left_mps<=.251f && output.targets.right_mps<=.251f);
    }
    assert(output.targets.left_mps>.24f);
    tick(REQ_HALT);
    assert(output.mode==RUN_STATIC_DEBUG && duty[0]==0 && duty[1]==0);
}
static void check_gyro(void) {
    app_gyro_init();
    g_app_param.flags |= APP_FLAG_GYRO_LOOP;
    tick(REQ_TRACK_TUNE);
    assert(output.mode==RUN_FAULT && output.reason==STOP_GYRO && duty[0]==0 && duty[1]==0);
    tick(REQ_RESET);
    for (unsigned i=1;i<=APP_GYRO_CAL_SAMPLES;i++) {
        app_gyro_poll(&output);
        app_gyro_poll(&output);
        tick(0);
        assert(output.gyro.calibration_samples==i && gyro_reads==i);
        assert(output.gyro.status==(i==APP_GYRO_CAL_SAMPLES ? GYRO_READY : GYRO_CALIBRATING));
    }
    gyro_raw=.02f+1.5707963268f;
    app_gyro_poll(&output); tick(0);
    app_gyro_zero_angle();
    for (unsigned i=0;i<200;i++) { app_gyro_poll(&output); tick(0); }
    assert(fabsf(output.gyro.rate_radps-1.5707963268f)<1e-5f);
    assert(fabsf(output.gyro.angle_rad-1.5707963268f)<1e-4f);
    g_app_param.flags |= APP_FLAG_GYRO_INVERT;
    app_gyro_poll(&output); tick(0);
    assert(output.gyro.rate_radps<0 && output.gyro.angle_rad<0);
    g_app_param.flags &= ~APP_FLAG_GYRO_INVERT;

    gyro_raw=.02f;
    app_gyro_poll(&output); tick(REQ_TRACK_TUNE);
    publish_tune_frame(); tick(0);
    assert(output.mode==RUN_TUNE_RUNNING && output.omega_target_radps==-.4f);
    assert(output.omega_command_radps<output.omega_target_radps);
    unsigned reads_before=gyro_reads;
    for (unsigned i=0;i<=APP_GYRO_TIMEOUT_MS/5;i++) {
        publish_tune_frame(); tick(0);
    }
    assert(gyro_reads==reads_before); /* TIM6 never touches the bus. */
    assert(output.mode==RUN_FAULT && output.reason==STOP_GYRO && output.gyro.status==GYRO_STALE);
    assert(duty[0]==0 && duty[1]==0);

    tick(REQ_RESET); app_gyro_poll(&output); tick(REQ_TRACK_TUNE);
    publish_tune_frame(); tick(0);
    assert(output.mode==RUN_TUNE_RUNNING);
    gyro_result=-1;
    app_gyro_poll(&output); publish_tune_frame(); tick(0);
    assert(output.mode==RUN_FAULT && output.reason==STOP_GYRO && output.gyro.status==GYRO_ERROR);
    assert(duty[0]==0 && duty[1]==0);
    tick(REQ_RESET);
    g_app_param.flags &= ~APP_FLAG_GYRO_LOOP;
    tick(REQ_TRACK_TUNE); publish_tune_frame(); tick(0);
    assert(output.mode==RUN_TUNE_RUNNING && output.omega_command_radps==output.omega_target_radps);
    tick(REQ_HALT);

    float integral=0, command;
    for (unsigned i=0;i<1000;i++)
        assert(yaw_control_step(&integral,&g_app_param,1,0,.005f,false,&command));
    assert(integral==.5f);
    assert(yaw_control_step(&integral,&g_app_param,1,-20,.005f,false,&command) && command==3);
    assert(yaw_control_step(&integral,&g_app_param,0,1,.005f,true,&command) && integral<.5f);
    assert(!yaw_control_step(&integral,&g_app_param,0,NAN,.005f,false,&command));
    GyroEstimator estimator={0};
    assert(gyro_estimator_step(&estimator,.02f,.02f,1,true,0) && estimator.count==1);
    assert(gyro_estimator_step(&estimator,1,1,1,true,5) && estimator.count==0);
    assert(!gyro_estimator_step(&estimator,NAN,0,1,true,10));
}
int main(void) {
    param_defaults(&g_app_param);
    app_control_init();
    tick(REQ_RUN30); assert(output.mode==RUN_DUTY_DEBUG && fabsf(duty[0]-.3f)<1e-6f && duty[0]==duty[1]);
    tick(0); assert(output.mode==RUN_DUTY_DEBUG); /* no camera required */
    tick(REQ_HALT|REQ_RUN30); assert(output.mode==RUN_STATIC_DEBUG && duty[0]==0); /* stop wins */
    tick(REQ_RUN5); assert(output.mode==RUN_DUTY_DEBUG);
    assert(fabsf(duty[0]-.05f)<1e-6f && duty[0]==duty[1]);
    tick(0); assert(fabsf(duty[0]-.05f)<1e-6f);
    tick(REQ_HALT|REQ_RUN5); assert(output.mode==RUN_STATIC_DEBUG && duty[0]==0);
    tick(REQ_SPEED_DEBUG); assert(output.mode==RUN_STATIC_DEBUG); /* no feedback/calibration */
    g_app_param.wheel_left_m=g_app_param.wheel_right_m=.06f;
    g_app_param.track_m=.15f; g_app_param.axle_m=.1f;
    g_app_param.counts_left=g_app_param.counts_right=100;
    g_app_param.calibration_mask=7; g_app_param.debug_speed_mps=.2f;
    feedback.valid=true;
    tick(REQ_SPEED_DEBUG); assert(output.mode==RUN_SPEED_DEBUG && output.targets.left_mps>0);
    for(unsigned i=0;i<200;i++)tick(0);
    assert(fabsf(output.targets.left_mps-.2f)<1e-5f && output.targets.left_mps==output.targets.right_mps);
    assert(duty[0]>0 && duty[0]<=g_app_param.duty_limit);
    feedback.valid=false; tick(0); assert(output.mode==RUN_FAULT && duty[0]==0);
    tick(REQ_RESET); assert(output.mode==RUN_STATIC_DEBUG);
    tick(REQ_RUN30); clock_us+=25000; tick(0); assert(output.mode==RUN_FAULT && duty[0]==0);
    tick(REQ_RESET); tick(REQ_RUN30); tick(REQ_HALT); assert(duty[0]==0 && output.targets.left_mps==0);
    check_continuous_start();
    check_tune();
    check_gyro();
    puts("PASS: continuous start, bounded line search, vision recovery and halt");
    puts("PASS: gyro calibration, rate and angle signs, zero, PI limits, timeout, error, bypass and foreground I2C");
    puts("PASS: track tune fresh start, steering, speed cap, timeout, stale/lost frame, feedback and halt; real control ISR open-loop30, speed PI target/ramp, guards, halt priority, timing fault");
    return 0;
}
