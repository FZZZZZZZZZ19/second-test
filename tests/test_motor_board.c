#include "app_runtime.h"
#include "zf_common_headfile.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
AppParam g_app_param;
static uint32_t physical_duty[2];
static uint8_t physical_phase[2];
static uint16_t raw[2];
/* Bench observation: PE9 drives physical right, PE14 drives physical left. */
void pwm_set_duty(int channel,uint32_t duty) {
    assert(channel==TIM1_PWM_MAP3_CH1_E9 || channel==TIM1_PWM_MAP3_CH4_E14);
    physical_duty[channel==TIM1_PWM_MAP3_CH1_E9 ? SIDE_RIGHT : SIDE_LEFT]=duty;
}
void pwm_init(int channel,uint32_t hz,uint32_t duty) { assert(hz==20000); pwm_set_duty(channel,duty); }
void gpio_set_level(int pin,uint8_t level) {
    assert(pin==E11 || pin==E13);
    physical_phase[pin==E11 ? SIDE_RIGHT : SIDE_LEFT]=level;
}
void gpio_init(int pin,int dir,uint8_t level,int mode) {
    (void)dir; (void)mode;
    if(pin==E9 || pin==E14) assert(level==0);
    else gpio_set_level(pin,level);
}
void board_encoder_init(void) {}
uint16_t board_encoder_raw16(SideId side) { return raw[side]; }
int main(void) {
    param_defaults(&g_app_param);
    g_app_param.wheel_left_m=g_app_param.wheel_right_m=.0337f;
    g_app_param.track_m=.1666f; g_app_param.axle_m=.0355f;
    g_app_param.counts_left=g_app_param.counts_right=2048;
    motor_init_zero();
    assert(physical_duty[0]==0 && physical_duty[1]==0);
    motor_apply_forward(.25f,0);
    assert(physical_duty[SIDE_LEFT]==2500 && physical_duty[SIDE_RIGHT]==0);
    motor_apply_forward(0,.1f);
    assert(physical_duty[SIDE_LEFT]==0 && physical_duty[SIDE_RIGHT]==1000);
    board_motor_write_ph(SIDE_LEFT,true);
    assert(physical_phase[SIDE_LEFT]==1 && physical_phase[SIDE_RIGHT]==0);
    board_motor_write_ph(SIDE_LEFT,false); board_motor_write_ph(SIDE_RIGHT,true);
    assert(physical_phase[SIDE_LEFT]==0 && physical_phase[SIDE_RIGHT]==1);
    motor_brake_zero(); assert(physical_duty[0]==0 && physical_duty[1]==0);
    assert(side_encoder_init()); raw[0]=(uint16_t)-20; raw[1]=40;
    SideFeedback feedback; assert(side_encoder_sample(.005f,5,&feedback));
    assert(feedback.delta[0]==20 && feedback.delta[1]==40);
    assert(feedback.speed_mps[0]>0 && feedback.speed_mps[1]>feedback.speed_mps[0]);
    float instant = feedback.rolling_m[0] / .005f;
    assert(fabsf(feedback.speed_mps[0]-instant/6)<.0001f);
    g_app_param.flags |= APP_FLAG_SPEED_LPF_OFF;
    raw[0]=(uint16_t)-40; raw[1]=80;
    assert(side_encoder_sample(.005f,10,&feedback));
    assert(fabsf(feedback.speed_mps[0]-instant)<.0001f);
    assert(fabsf(feedback.speed_mps[1]-2*instant)<.0001f);
    assert(feedback.count[0]==40 && feedback.count[1]==80);
    raw[0]=(uint16_t)-2040;
    assert(!side_encoder_sample(.005f,15,&feedback)); /* Raw overspeed protection. */
    puts("PASS: selectable speed LPF and real motor output port maps physical sides, PH, zero output, and encoder signs");
    return 0;
}
