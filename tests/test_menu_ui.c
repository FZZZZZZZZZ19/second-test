#include "app_runtime.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
AppParam g_app_param;
uint8_t mt9v03x_image[120][188];
static uint32_t now, requests;
static uint8_t key=7, record[APP_PARAM_RECORD_BYTES];
static ControlSnapshot s;
static unsigned resumes, images, saves, packets;
static bool capture;
static unsigned char screen[128][160];
static unsigned rgb_tiles;
BoardIrqToken board_irq_save(void) { return 0; }
void board_irq_restore(BoardIrqToken t) { (void)t; }
void board_memory_barrier(void) {}
uint32_t board_time_ms(void) { return now; }
void board_clock_init(void) { now=0; }
uint8_t board_key_code(void) { return key; }
bool board_camera_hold_after_frame_isr(void) { capture=false; return true; }
bool board_camera_restart_full_frame(void) { capture=true; resumes++; return true; }
int mt9v03x_init(void) { return 0; }
void app_exchange_request(uint32_t r) { requests |= r; }
void app_exchange_read_control(ControlSnapshot *out) { *out=s; }
void app_control_init(void) {}
bool side_encoder_init(void) { return true; }
void motor_brake_zero(void) {}
bool param_store_save(const AppParam *p) { saves++; return param_encode(p,saves,record); }
void tft180_init(void) {}
void tft180_set_dir(int d) { (void)d; }
void tft180_set_font(int f) { assert(f==1); }
void tft180_clear(void) { memset(screen,255,sizeof screen); }
void tft180_set_color(uint16_t pen,uint16_t bg) { assert(pen==0 && bg==0xffff); }
void tft180_show_rgb565_image(uint16_t x,uint16_t y,const uint16_t *p,uint16_t w,uint16_t h,uint16_t dw,uint16_t dh,uint8_t color_mode) {
    assert(x+dw<=160 && y+dh<=128 && w==8 && h==16 && color_mode==0);
    rgb_tiles++;
    for(unsigned j=0;j<dh;j++)for(unsigned i=0;i<dw;i++) screen[y+j][x+i]=p[(j*h/dh)*w+i*w/dw]==0?0:255;
}
static void snapshot(const char *path) {
    FILE *f=fopen(path,"wb"); assert(f); fprintf(f,"P5\n160 128\n255\n");
    assert(fwrite(screen,1,sizeof screen,f)==sizeof screen); fclose(f);
}
void tft180_show_string(uint16_t x,uint16_t y,const char *text) { assert(x<160 && y<128 && strlen(text)<=26); }
void tft180_show_gray_image(uint16_t x,uint16_t y,const uint8_t *p,uint16_t w,uint16_t h,uint16_t dw,uint16_t dh,uint8_t threshold) {
    assert(x+dw<=160 && y+dh<=128 && p && w==188 && h==120 && threshold==0); images++;
}
void uart_write_buffer(int uart,const uint8_t *p,unsigned len) { assert(uart==3 && len==20 && p[18]==0x80 && p[19]==0x7f); packets++; }
void pit_disable(int timer) { assert(timer==6); }
void pit_ms_init(int timer,unsigned period) { assert(timer==6 && period==5); }
static void poll(unsigned dt) { now+=dt; debug_view_poll(&s); }
static void click(void) { key=3; poll(1); poll(40); key=7; poll(50); poll(40); }
static void hold(void) { key=3; poll(1); poll(40); poll(800); key=7; poll(1); poll(40); }
int main(void) {
    param_defaults(&g_app_param); s.mode=RUN_STATIC_DEBUG; s.config_ok=true;
    assert(camera_frame_init()); assert(camera_frame_set_running(false)); debug_view_init(); poll(100);
    snapshot("build/host/menu-main.pgm");
    assert(screen[16][0]==0 && screen[48][0]==255); /* selected row is inverted */
    unsigned tiles_before=rgb_tiles; poll(100); assert(rgb_tiles==tiles_before); /* static UI does not repaint */
    hold(); assert(capture && (requests&REQ_READY));
    requests=0; s.mode=RUN_READY; poll(5); assert(requests&REQ_START);
    s.mode=RUN_RUNNING; poll(5);
    camera_frame_on_done_isr(); camera_frame_on_done_isr();
    FrameView f; VisionFeatures v={0}; assert(camera_frame_acquire(&f));
    debug_view_frame(&f,&s,&v); assert(images==1);
    camera_frame_release(f.frame_id); assert(capture);
    hold(); assert(!capture && (requests&REQ_HALT));
    unsigned count=resumes; camera_frame_on_done_isr(); assert(!camera_frame_acquire(&f) && count==resumes);
    s.mode=RUN_STATIC_DEBUG; requests=0;
    click(); hold(); snapshot("build/host/menu-para.pgm"); /* para */
    hold(); hold(); snapshot("build/host/menu-keypad.pgm"); /* vision -> omega Kp */
    for(unsigned i=0;i<13;i++) { click(); }
    hold(); /* keypad back */
    click(); click(); click(); hold(); /* vision back -> para */
    click(); click(); hold(); assert(saves==1); /* save without inventing geometry */
    AppParam loaded; uint32_t sequence;
    assert(param_decode(record,&loaded,&sequence) && loaded.wheel_left_m==0 && !param_can_arm(&loaded,false));
    click(); hold(); /* para back -> main */
    click(); click(); hold(); /* control */
    click(); click(); hold(); /* vofa blocked without target/calibration */
    assert(!(requests&REQ_SPEED_DEBUG));
    hold(); /* live stop -> control */
    g_app_param.wheel_left_m=g_app_param.wheel_right_m=.06f;
    g_app_param.track_m=.15f; g_app_param.axle_m=.1f; g_app_param.counts_left=g_app_param.counts_right=100;
    g_app_param.calibration_mask=7; g_app_param.debug_speed_mps=.1f; s.feedback.valid=true;
    click(); click(); hold(); assert(requests&REQ_SPEED_DEBUG);
    poll(20); assert(packets>0); snapshot("build/host/menu-live.pgm");
    hold(); count=packets; poll(100); assert(packets==count && !capture);
    puts("PASS: real UI start handshake, frame pause/resume, save/reload, VOFA guard/start/stop, render bounds");
    return 0;
}
