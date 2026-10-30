#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H
#include "app_core.h"
#include "board_port.h"
extern AppParam g_app_param;
bool app_boot_init(void);
void app_main_poll(void);
void app_control_init(void);
void app_control_tick_isr(void);
void app_control_camera_ready(bool ok);
void app_exchange_publish_motion(const MotionRequest *r);
void app_exchange_read_motion(MotionRequest *r);
void app_exchange_publish_control(const ControlSnapshot *s);
void app_exchange_read_control(ControlSnapshot *s);
void app_exchange_request(uint32_t request);
uint32_t app_exchange_take_requests(void);
bool run_manager_step(RunMode *mode, StopReason *reason, uint32_t requests, bool armable,
                      bool fresh, uint32_t *session);
bool side_encoder_init(void);
bool side_encoder_sample(float dt, uint32_t now, SideFeedback *f);
void motor_init_zero(void);
void motor_apply_forward(float left, float right);
void motor_brake_zero(void);
bool camera_frame_init(void);
void camera_frame_on_done_isr(void);
bool camera_frame_acquire(FrameView *f);
void camera_frame_release(uint32_t frame_id);
void debug_view_init(void);
void debug_view_poll(const ControlSnapshot *s);
void debug_view_frame(const FrameView *f, const ControlSnapshot *s);
#endif
