#include "app_runtime.h"
#include "maintenance.h"
#include "menu_display.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
AppParam g_app_param;
uint8_t mt9v03x_image[120][188];
volatile uint8_t mt9v03x_finish_flag;
static uint32_t now, requests;
static uint8_t key=7, record[APP_PARAM_RECORD_BYTES];
static ControlSnapshot s;
static unsigned resumes, images, saves, packets;
static bool capture, save_ok=true;
static int camera_error;
static unsigned char screen[160][128];
static unsigned rgb_tiles, grid_cells;
BoardIrqToken board_irq_save(void) { return 0; }
void board_irq_restore(BoardIrqToken t) { (void)t; }
void board_memory_barrier(void) {}
uint32_t board_time_ms(void) { return now; }
void board_clock_init(void) { now=0; }
uint8_t board_key_code(void) { return key; }
bool board_camera_stop(void) { capture=false; return true; }
bool board_camera_start(void) { capture=true; resumes++; return true; }
int mt9v03x_init(void) { return camera_error; }
void app_exchange_request(uint32_t r) { requests |= r; }
void app_exchange_read_control(ControlSnapshot *out) { *out=s; }
void app_control_init(void) {}
void app_control_camera_ready(bool ok) { (void)ok; }
bool side_encoder_init(void) { return true; }
void motor_brake_zero(void) {}
bool param_store_save(const AppParam *p) { assert(maintenance_save_check(&s,p)==MAINT_OK); saves++; return save_ok && param_encode(p,saves,record); }
void menu_display_init(void) {}
void tft180_set_dir(int d) { (void)d; }
void tft180_set_font(int f) { assert(f==1); }
void menu_display_clear(void) { memset(screen,255,sizeof screen); }
void tft180_set_color(uint16_t pen,uint16_t bg) { assert(pen==0 && bg==0xffff); }
void menu_display_rgb(uint16_t x,uint16_t y,const uint16_t *p,uint16_t w,uint16_t h,uint16_t dw,uint16_t dh,uint8_t color_mode) {
    assert(x+dw<=128 && y+dh<=160 && w==8 && h==16 && color_mode==0);
    if(dw==30 && dh==18) {
        assert(x>=4 && x+dw<=124 && y>=52 && y+dh<=124);
        grid_cells++;
    }
    rgb_tiles++;
    for(unsigned j=0;j<dh;j++)for(unsigned i=0;i<dw;i++) screen[y+j][x+i]=p[(j*h/dh)*w+i*w/dw]==0?0:255;
}
void menu_display_line(uint16_t x,uint16_t y,uint16_t x2,uint16_t y2,uint16_t color) {
    assert(x<128 && x2<128 && y<160 && y2<160 && (x==x2 || y==y2));
    for(unsigned j=y;j<=y2;j++)for(unsigned i=x;i<=x2;i++) screen[j][i]=color?255:0;
}
static void snapshot(const char *path) {
    FILE *f=fopen(path,"wb"); assert(f); fprintf(f,"P5\n128 160\n255\n");
    assert(fwrite(screen,1,sizeof screen,f)==sizeof screen); fclose(f);
}
void tft180_show_string(uint16_t x,uint16_t y,const char *text) { assert(x<160 && y<128 && strlen(text)<=26); }
void menu_display_camera(const uint8_t *p,uint16_t w,uint16_t h,uint16_t stride) {
    assert(p && w==APP_WIDTH && h==APP_HEIGHT && stride==APP_WIDTH); images++;
}
void uart_write_buffer(int uart,const uint8_t *p,unsigned len) {
    assert(uart==3 && len==20 && p[18]==0x80 && p[19]==0x7f);
    if(s.mode==RUN_DUTY_DEBUG) for(unsigned i=0;i<4;i++) {
        float v; memcpy(&v,p+i*4,4);
        assert(i%2?(s.feedback.valid?v==s.feedback.speed_mps[i/2]:isnan(v)):v==g_app_param.debug_speed_mps);
    }
    packets++;
}
void pit_disable(int timer) { assert(timer==6); }
void pit_ms_init(int timer,unsigned period) { assert(timer==6 && period==5); }
static void poll(unsigned dt) { now+=dt; debug_view_poll(&s); }
static void way(uint8_t code) { key=code; poll(1); poll(40); key=7; poll(1); poll(40); }
static void click(void) { way(4); }
static void hold(void) { way(1); }
static void back(void) { way(3); }
int main(void) {
    FrameView no_frame;
    camera_error=1; assert(!camera_frame_init() && camera_frame_init_status()==1);
    assert(!camera_frame_acquire(&no_frame));
    camera_error=0; assert(camera_frame_set_running(true) && camera_frame_init_status()==0);
    param_defaults(&g_app_param); s.mode=RUN_STATIC_DEBUG; s.config_ok=true; s.feedback.delta[1]=-6;
    assert(camera_frame_init()); assert(camera_frame_set_running(false)); debug_view_init(); poll(100);
    snapshot("build/host/menu-main.pgm");
    assert(screen[16][0]==0 && screen[48][0]==255); /* selected row is inverted */
    unsigned tiles_before=rgb_tiles; poll(100); assert(rgb_tiles==tiles_before); /* static UI does not repaint */
    hold(); assert(capture && (requests&REQ_READY));
    requests=0; s.mode=RUN_READY; poll(5); assert(requests&REQ_START);
    s.mode=RUN_RUNNING; poll(5);
    mt9v03x_image[0][0]=11; mt9v03x_image[119][187]=59;
    mt9v03x_image[0][29]=255; mt9v03x_image[0][30]=17;
    mt9v03x_image[0][157]=23; mt9v03x_image[0][158]=255;
    mt9v03x_image[119][30]=31; mt9v03x_image[119][157]=47;
    mt9v03x_finish_flag=1; camera_frame_on_done_isr();
    FrameView f; VisionFeatures v={0}; assert(camera_frame_acquire(&f));
    assert(f.width==188 && f.height==120 && f.stride==188);
    assert(f.pixels[0]==11 && f.pixels[30]==17);
    assert(f.pixels[157]==23 && f.pixels[158]==255 && f.pixels[119*188+30]==31 && f.pixels[120*188-1]==59);
    mt9v03x_image[0][30]=55; mt9v03x_finish_flag=1; camera_frame_on_done_isr();
    assert(f.pixels[0]==11 && f.pixels[30]==17); /* owned frame stays stable */
    debug_view_frame(&f,&s,&v); assert(images==1);
    FrameView next_frame=f; next_frame.frame_id++;
    poll(33); debug_view_frame(&next_frame,&s,&v);
    assert(images==2); /* New frames must not be throttled to 5 Hz. */
    unsigned restarts=resumes; camera_frame_release(f.frame_id);
    assert(capture && resumes==restarts); /* no per-frame DMA reset */
    back(); assert(!capture && (requests&REQ_HALT));
    unsigned count=resumes; camera_frame_on_done_isr(); assert(!camera_frame_acquire(&f) && count==resumes);
    s.mode=RUN_STATIC_DEBUG; requests=0;
    click(); hold(); snapshot("build/host/menu-para.pgm"); /* para */
    hold(); hold(); assert(grid_cells>=16); snapshot("build/host/menu-keypad.pgm"); /* vision -> omega Kp */
    unsigned tile_count=rgb_tiles; poll(100); assert(tile_count==rgb_tiles);
    way(6); poll(100); assert(rgb_tiles>tile_count); /* grid selection repaint */
    way(2); way(1); /* input 1 */
    way(6); way(6); way(6); /* DEL */
    way(4); way(4); way(4); way(2); /* SAVE */
    way(1); /* save -> vision */
    assert(g_app_param.lateral_kp==1);
    hold(); /* reopen omega Kp, input 2 and SAVE */
    way(6); way(1); way(4); way(4); way(4); way(6);
    save_ok=false; unsigned attempts=saves;
    way(1); assert(saves==attempts+1 && g_app_param.lateral_kp==1); /* failure leaves RAM unchanged */
    snapshot("build/host/menu-save-error.pgm");
    save_ok=true; poll(600); attempts=saves;
    key=1; poll(1); poll(40);
    assert(saves==attempts+1 && g_app_param.lateral_kp==2);
    poll(1500); assert(saves==attempts+1); /* held center cannot save or enter again after clock reset */
    key=7; poll(1); poll(40);
    AppParam from_save; uint32_t saved_seq;
    assert(param_decode(record,&from_save,&saved_seq) && from_save.lateral_kp==2);
    hold(); way(1); back(); /* type but KEY1 cancels */
    assert(g_app_param.lateral_kp==2 && saves==attempts+1);
    click(); click(); click(); hold(); /* camera calibration */
    snapshot("build/host/menu-camera-cal.pgm");
    for(unsigned j=0;j<7;j++) { click(); }
    hold(); /* ground_h7 */
    way(6); way(6); way(6); way(4); way(4); way(1); /* negative sign */
    way(4); way(2); way(2); way(2); way(1); /* decimal point */
    way(6); for(unsigned j=0;j<5;j++) { way(1); } /* zeros */
    way(5); way(5); way(5); way(2); way(1); /* 1 */
    way(4); way(4); way(4); way(6); way(6); way(1); /* SAVE */
    assert(fabsf(g_app_param.ground_h7+.000001f)<1e-12f);
    hold(); snapshot("build/host/menu-camera-value.pgm"); back(); back(); back(); /* preserve all parent positions */
    attempts=saves; click(); click(); click(); hold(); assert(saves==attempts+1); /* save without inventing geometry */
    AppParam loaded; uint32_t sequence;
    assert(param_decode(record,&loaded,&sequence) && loaded.wheel_left_m==0 && !param_can_arm(&loaded,false));
    way(5); hold(); /* para hardware */
    snapshot("build/host/menu-hardware.pgm");
    hold(); /* body length keyboard: enter 300 mm */
    way(6); way(6); way(1); way(2); way(2); way(4); way(4); way(4); way(6);
    way(1); way(1); way(6); way(1); /* SAVE */
    assert(fabsf(g_app_param.body_length_m-.3f)<1e-6f);
    click(); click(); click(); hold(); back(); /* wheel track edit cancellation preserves cursor */
    snapshot("build/host/menu-hardware-back.pgm");
    back(); back(); /* hardware -> para(hardware), para -> main(para) */
    click(); hold(); /* main control */
    click(); click(); hold(); /* Invalid configuration cannot start VOFA or run30. */
    assert(!(requests&(REQ_RUN30|REQ_SPEED_DEBUG)));
    poll(100);
    snapshot("build/host/menu-vofa-blocked.pgm");
    back(); /* KEY1 live back stops -> control */
    s.mode=RUN_STATIC_DEBUG; s.duty[0]=s.duty[1]=0; requests=0;
    g_app_param.wheel_left_m=g_app_param.wheel_right_m=.06f;
    g_app_param.track_m=.15f; g_app_param.axle_m=.1f; g_app_param.counts_left=g_app_param.counts_right=100;
    g_app_param.calibration_mask=0; g_app_param.debug_speed_mps=.1f; s.feedback.valid=true;
    hold(); assert(requests&REQ_SPEED_DEBUG);
    poll(20); assert(packets>0); snapshot("build/host/menu-live.pgm");
    back(); count=packets; poll(100); assert(packets==count && !capture);
    s.mode=RUN_STATIC_DEBUG; s.feedback.delta[0]=1; s.feedback.speed_mps[0]=0;
    click(); hold(); /* control -> VOFA target keyboard */
    way(4); way(4); way(4); way(1); /* decimal point */
    way(5); way(5); way(5); way(6); way(1); /* 2 */
    way(4); way(4); way(4); way(6); way(1); /* SAVE .2 despite idle count jitter */
    assert(fabsf(g_app_param.debug_speed_mps-.2f)<1e-6f);
    assert(param_decode(record,&loaded,&sequence) && loaded.debug_speed_mps==g_app_param.debug_speed_mps);
    requests=0; click(); click(); hold(); /* target -> stop -> run 5 */
    assert((requests&REQ_RUN5) && !(requests&(REQ_RUN30|REQ_SPEED_DEBUG)));
    back(); assert(requests&REQ_HALT);
    puts("PASS: real UI start handshake, frame pause/resume, save/reload, VOFA guard/start/stop, render bounds");
    return 0;
}
