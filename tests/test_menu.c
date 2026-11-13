
#include "menu_model.h"
#include "core_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static void put32(uint8_t *p, uint32_t x) { for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(x>>(i*8)); }
static void pick(MenuModel *m,const char *label) {
    for(unsigned i=0;i<menu_count(m);i++) if(!strcmp(menu_label(m,i),label)) { m->selected=i;return; }
    assert(!"missing keypad label");
}
static uint32_t key_ms=5000;
static MenuAction way(MenuModel *m,uint8_t key) {
    menu_key(m,key,++key_ms); key_ms+=40;
    MenuAction a=menu_key(m,key,key_ms);
    key_ms+=1000; assert(menu_key(m,key,key_ms)==MENU_NONE); /* no held repeat */
    menu_key(m,7,++key_ms); key_ms+=40; menu_key(m,7,key_ms);
    return a;
}
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
    pick(&m,"."); menu_confirm(&m); pick(&m,"2"); menu_confirm(&m);
    assert(!strcmp(m.input,".2")); assert(menu_apply_value(&m,&p)); assert(fabsf(p.lateral_kp-.2f)<1e-6f);
    pick(&m,"."); menu_confirm(&m); assert(m.input_error && !strcmp(m.input,".2"));
    pick(&m,"DEL"); menu_confirm(&m); assert(!strcmp(m.input,"."));
    assert(!menu_apply_value(&m,&p));
    strcpy(m.input,"nan"); assert(!menu_apply_value(&m,&p));
    strcpy(m.input,"10001"); assert(!menu_apply_value(&m,&p));
    pick(&m,"BACK"); menu_confirm(&m); assert(m.page==MENU_VISION);
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
    MenuModel k; menu_init(&k); k.page=MENU_KEYPAD; k.parent=MENU_VISION;
    assert(menu_count(&k)==16);
    way(&k,4); assert(k.selected==0); /* upper edge clamps */
    way(&k,2); assert(k.selected==1); way(&k,5); assert(k.selected==5);
    way(&k,6); assert(k.selected==4); way(&k,4); assert(k.selected==0);
    way(&k,1); assert(!strcmp(k.input,"1")); /* held center only once */
    way(&k,3); assert(!strcmp(k.input,"1") && k.selected==0); /* KEY1 ignored here */
    k.selected=13; way(&k,2); assert(k.selected==15); /* skip blank */
    way(&k,6); assert(k.selected==13);
    k.selected=10; way(&k,5); assert(k.selected==10); /* blank at bottom, stay */
    pick(&k,"CLR"); way(&k,1); assert(!k.input[0]);
    pick(&k,"OK"); assert(way(&k,1)==MENU_APPLY);
    pick(&k,"BACK"); way(&k,1); assert(k.page==MENU_VISION && k.selected==0);
    way(&k,2); assert(k.page==MENU_VISION && k.selected==0); /* five-way scoped to keypad */
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
