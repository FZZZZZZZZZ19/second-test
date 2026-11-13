
#include "app_runtime.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
AppParam g_app_param;
static uint32_t clock_us, requests;
static ControlSnapshot output;
static SideFeedback feedback;
static float duty[2];
BoardIrqToken board_irq_save(void) { return 0; }
void board_irq_restore(BoardIrqToken t) { (void)t; }
void board_clock_tick(void) {}
uint32_t board_time_us(void) { return clock_us; }
uint32_t board_time_ms(void) { return clock_us/1000; }
void app_exchange_publish_control(const ControlSnapshot *s) { output=*s; }
void app_exchange_read_motion(MotionRequest *r) { memset(r,0,sizeof(*r)); }
uint32_t app_exchange_take_requests(void) { uint32_t r=requests; requests=0; return r; }
bool side_encoder_sample(float dt, uint32_t now, SideFeedback *s) { (void)dt;(void)now;*s=feedback;return s->valid; }
void motor_brake_zero(void) { duty[0]=duty[1]=0; }
void motor_apply_forward(float l,float r) { duty[0]=l;duty[1]=r; }
void board_motor_write_en(SideId side,uint32_t d) { duty[side]=d/10000.f; }
static void tick(uint32_t req) { clock_us+=5000; requests=req; app_control_tick_isr(); }
int main(void) {
    param_defaults(&g_app_param);
    app_control_init();
    tick(REQ_RUN30); assert(output.mode==RUN_DUTY_DEBUG && fabsf(duty[0]-.3f)<1e-6f && duty[0]==duty[1]);
    tick(0); assert(output.mode==RUN_DUTY_DEBUG); /* no camera required */
    tick(REQ_HALT|REQ_RUN30); assert(output.mode==RUN_STATIC_DEBUG && duty[0]==0); /* stop wins */
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
    puts("PASS: real control ISR open-loop30, speed PI target/ramp, guards, halt priority, timing fault");
    return 0;
}
