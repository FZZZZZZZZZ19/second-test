
#include "menu_model.h"
#include "core_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static void put32(uint8_t *p, uint32_t x) { for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(x>>(i*8)); }
int main(void) {
    MenuModel m; menu_init(&m);
    assert(menu_count(&m)==3 && !strcmp(menu_label(&m,0),"start"));
    assert(menu_key(&m,3,10)==MENU_NONE);
    menu_key(&m,7,25); menu_key(&m,7,70); assert(m.selected==0); /* bounce */
    menu_key(&m,3,100); menu_key(&m,3,140);
    menu_key(&m,7,200); menu_key(&m,7,240); assert(m.selected==1);
    menu_key(&m,3,300); menu_key(&m,3,340);
    menu_key(&m,3,1140); assert(m.page==MENU_PARA);

    menu_key(&m,3,2500); assert(m.page==MENU_PARA); /* hold cannot repeat */
    menu_key(&m,7,2510); menu_key(&m,7,2550); assert(m.selected==0); /* no click after hold */
    menu_key(&m,0,2600); menu_key(&m,0,2700); assert(m.selected==0);
    menu_confirm(&m); assert(m.page==MENU_VISION);
    menu_confirm(&m); assert(m.page==MENU_KEYPAD);
    AppParam p={0}; p.speed_limit_mps=.4f;
    assert(!menu_apply_value(&m,&p));
    m.selected=10; menu_confirm(&m); m.selected=2; menu_confirm(&m);
    assert(!strcmp(m.input,".2")); assert(menu_apply_value(&m,&p)); assert(fabsf(p.lateral_kp-.2f)<1e-6f);
    m.selected=10; menu_confirm(&m); assert(m.input_error && !strcmp(m.input,".2"));
    m.selected=11; menu_confirm(&m); assert(!strcmp(m.input,"."));
    assert(!menu_apply_value(&m,&p));
    strcpy(m.input,"nan"); assert(!menu_apply_value(&m,&p));
    strcpy(m.input,"10001"); assert(!menu_apply_value(&m,&p));
    m.selected=13; menu_confirm(&m); assert(m.page==MENU_VISION);
    m.selected=3; menu_confirm(&m); assert(m.page==MENU_PARA);
    m.selected=1; menu_confirm(&m); assert(m.page==MENU_PARAMS);
    menu_confirm(&m); strcpy(m.input,".5"); assert(menu_apply_value(&m,&p));
    assert(p.kp_left==.5f && p.kp_right==.5f);
    m.field=9; strcpy(m.input,".5"); assert(!menu_apply_value(&m,&p));
    strcpy(m.input,".25"); assert(menu_apply_value(&m,&p));
    m.page=MENU_RACE_IMAGE; assert(menu_confirm(&m)==MENU_STOP && m.page==MENU_MAIN);
    m.selected=2; menu_confirm(&m); assert(m.page==MENU_CONTROL);
    m.selected=2; assert(menu_confirm(&m)==MENU_VOFA && m.page==MENU_LIVE);
    m.selected=1; assert(menu_confirm(&m)==MENU_STOP && m.page==MENU_CONTROL);
    m.selected=1; assert(menu_confirm(&m)==MENU_CAMERA && m.page==MENU_IMAGE);
    assert(menu_confirm(&m)==MENU_STOP && m.page==MENU_CONTROL);
    ControlSnapshot s={0}; s.targets.left_mps=1; s.targets.right_mps=-2;
    s.feedback.speed_mps[0]=.5f; s.feedback.speed_mps[1]=0;
    uint8_t packet[20]; vofa_pack(&s,packet);
    const uint8_t expected[20]={0,0,0x80,0x3f,0,0,0,0x3f,0,0,0,0xc0,0,0,0,0,0,0,0x80,0x7f};
    assert(!memcmp(packet,expected,20));
    param_defaults(&p);
    assert(!param_valid(&p) && param_storable(&p));
    uint8_t blank[APP_PARAM_RECORD_BYTES]; AppParam defaults; uint32_t blank_seq;
    assert(param_encode(&p,1,blank) && param_decode(blank,&defaults,&blank_seq));
    assert(defaults.wheel_left_m==0 && !param_can_arm(&defaults,false));
    p.wheel_left_m=p.wheel_right_m=.06f;
    p.track_m=.15f; p.axle_m=.1f; p.counts_left=p.counts_right=100;
    assert(param_valid(&p)); p.debug_speed_mps=.2f;
    uint8_t record[APP_PARAM_RECORD_BYTES]; AppParam q; uint32_t seq;
    assert(param_encode(&p,7,record) && param_decode(record,&q,&seq));
    assert(q.debug_speed_mps==.2f && seq==7);
    put32(record+4,2); put32(record+12,49); memset(record+32+49*4,255,4);
    put32(record+248,app_crc32(record,248));
    assert(param_decode(record,&q,&seq) && q.debug_speed_mps==0 && q.kp_left==p.kp_left);
    put32(record+4,3); put32(record+248,app_crc32(record,248));
    assert(!param_decode(record,&q,&seq));
    puts("PASS: KEY1 debounce/hold, menu/back, keypad validation, paired gains, VOFA bytes, Flash migration");
    return 0;
}
