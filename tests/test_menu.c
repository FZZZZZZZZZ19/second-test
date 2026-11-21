
#include "menu_model.h"
#include "vehicle_calibration.h"
#include "core_math.h"
#include "maintenance.h"
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
    assert(menu_key(&m,4,10)==MENU_NONE);
    menu_key(&m,7,25); menu_key(&m,7,70); assert(m.selected==0); /* bounce */
    way(&m,3); assert(m.page==MENU_MAIN && m.selected==0); /* KEY1 never selects */
    way(&m,5); assert(m.selected==2); way(&m,4); assert(m.selected==0);
    way(&m,4); assert(m.selected==1); way(&m,6); assert(m.selected==1);
    way(&m,1); assert(m.page==MENU_PARA && m.selected==0);
    way(&m,1); assert(m.page==MENU_VISION);
    way(&m,1); assert(m.page==MENU_KEYPAD);
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
    m.selected=4; menu_confirm(&m); assert(m.page==MENU_PARA);
    m.selected=1; menu_confirm(&m); assert(m.page==MENU_PARAMS);
    menu_confirm(&m); strcpy(m.input,".5"); assert(menu_apply_value(&m,&p));
    assert(p.kp_left==.5f && p.kp_right==.5f);
    m.field=9; strcpy(m.input,".5"); assert(menu_apply_value(&m,&p));
    assert(p.debug_speed_mps==.5f && p.speed_limit_mps==.4f);
    strcpy(m.input,".25"); assert(menu_apply_value(&m,&p));
    m.page=MENU_RACE_IMAGE; assert(menu_confirm(&m)==MENU_STOP && m.page==MENU_MAIN);
    m.selected=2; menu_confirm(&m); assert(m.page==MENU_CONTROL);
    m.selected=2; assert(menu_confirm(&m)==MENU_VOFA && m.page==MENU_LIVE);
    m.selected=1; assert(menu_confirm(&m)==MENU_STOP && m.page==MENU_CONTROL);
    m.selected=1; assert(menu_confirm(&m)==MENU_CAMERA && m.page==MENU_IMAGE);
    assert(menu_confirm(&m)==MENU_STOP && m.page==MENU_CONTROL);
    MenuModel k; menu_init(&k); k.page=MENU_KEYPAD; k.parent=MENU_VISION;
    assert(menu_count(&k)==16);
    way(&k,5); assert(k.selected==0); /* upper edge clamps */
    way(&k,6); assert(k.selected==1); way(&k,4); assert(k.selected==5);
    way(&k,2); assert(k.selected==4); way(&k,5); assert(k.selected==0);
    way(&k,1); assert(!strcmp(k.input,"1")); /* held center only once */
    way(&k,3); assert(k.page==MENU_VISION); /* KEY1 cancels keypad */
    way(&k,1); assert(k.page==MENU_KEYPAD && !k.input[0]);
    k.selected=13; way(&k,6); assert(k.selected==14); /* SAVE occupies old blank */
    assert(way(&k,1)==MENU_APPLY_SAVE);
    way(&k,2); assert(k.selected==13);
    k.selected=10; way(&k,4); assert(k.selected==14);
    pick(&k,"CLR"); way(&k,1); assert(!k.input[0]);
    pick(&k,"-"); way(&k,1); assert(!strcmp(k.input,"-"));
    way(&k,1); assert(!k.input[0]);
    pick(&k,"BACK"); way(&k,1); assert(k.page==MENU_VISION && k.selected==0);
    way(&k,6); assert(k.page==MENU_VISION && k.selected==0); /* five-way scoped to keypad */
    const MenuPage pages[]={MENU_MAIN,MENU_PARA,MENU_VISION,MENU_PARAMS,MENU_CONTROL,MENU_KEYPAD,MENU_RACE_IMAGE,MENU_IMAGE,MENU_LIVE,MENU_HARDWARE,MENU_CAMERA_CAL};
    const MenuPage parents[]={MENU_MAIN,MENU_MAIN,MENU_PARA,MENU_PARA,MENU_MAIN,MENU_PARAMS,MENU_MAIN,MENU_CONTROL,MENU_CONTROL,MENU_PARA,MENU_VISION};
    for(unsigned j=0;j<sizeof pages/sizeof pages[0];j++) {
        menu_init(&k); k.page=pages[j]; k.parent=MENU_PARAMS;
        MenuAction a=way(&k,3);
        assert(k.page==parents[j] && k.selected==0);
        assert(a==((pages[j]==MENU_CONTROL || pages[j]==MENU_RACE_IMAGE || pages[j]==MENU_IMAGE || pages[j]==MENU_LIVE)?MENU_STOP:MENU_NONE));
    }
    MenuModel h; menu_init(&h); h.selected=1; menu_confirm(&h);
    h.selected=2; menu_confirm(&h); assert(h.page==MENU_HARDWARE && menu_count(&h)==11);
    AppParam hardware; param_defaults(&hardware);
    h.selected=3; menu_confirm(&h); strcpy(h.input,"150");
    assert(menu_apply_value(&h,&hardware) && fabsf(hardware.track_m-.15f)<1e-6f);
    assert(fabsf(menu_param_value(&h,&hardware)-150)<.001f);
    menu_back(&h); assert(h.page==MENU_HARDWARE && h.selected==3);
    h.selected=0; menu_confirm(&h); strcpy(h.input,"300");
    assert(menu_apply_value(&h,&hardware) && fabsf(hardware.body_length_m-.3f)<1e-6f);
    strcpy(h.input,"3001"); assert(!menu_apply_value(&h,&hardware)); menu_back(&h);
    h.selected=7; menu_confirm(&h); strcpy(h.input,"1024.5"); assert(!menu_apply_value(&h,&hardware));
    strcpy(h.input,"1024"); assert(menu_apply_value(&h,&hardware) && hardware.counts_left==1024);
    menu_back(&h); assert(h.selected==7);
    h.selected=10; menu_confirm(&h); assert(h.page==MENU_PARA && h.selected==2);
    menu_back(&h); assert(h.page==MENU_MAIN && h.selected==1);
    h.selected=2; menu_confirm(&h); h.selected=2; assert(menu_confirm(&h)==MENU_VOFA);
    menu_back(&h); assert(h.page==MENU_CONTROL && h.selected==2);
    h.selected=3; menu_confirm(&h); menu_back(&h); assert(h.selected==3);
    h.selected=5; assert(menu_confirm(&h)==MENU_RUN5);
    assert(menu_back(&h)==MENU_STOP && h.selected==5);
    h.selected=6; assert(menu_confirm(&h)==MENU_TRACK_TUNE && h.page==MENU_TUNE_IMAGE);
    assert(menu_count(&h)==1);
    assert(menu_back(&h)==MENU_STOP && h.selected==6);
    h.selected=7; assert(menu_confirm(&h)==MENU_SPEED_LPF && h.page==MENU_CONTROL);
    h.selected=8; menu_confirm(&h); assert(h.page==MENU_MAIN && h.selected==2);
    menu_init(&h); h.selected=1; menu_confirm(&h); menu_confirm(&h);
    h.selected=3; menu_confirm(&h); assert(h.page==MENU_CAMERA_CAL && menu_count(&h)==9);
    AppParam calibration; param_defaults(&calibration);
    for(unsigned j=0;j<8;j++) {
        h.selected=j; menu_confirm(&h);
        assert(h.page==MENU_KEYPAD && h.field==25+j);
        strcpy(h.input,".000001"); pick(&h,"-"); menu_confirm(&h);
        assert(!strcmp(h.input,"-.000001") && menu_apply_value(&h,&calibration));
        assert(fabsf(menu_param_value(&h,&calibration)+.000001f)<1e-12f);
        char value_text[16]; menu_value_text(&h,&calibration,value_text,sizeof value_text);
        assert(!strcmp(value_text,"-1.0000e-06"));
        pick(&h,"-"); menu_confirm(&h); assert(!strcmp(h.input,".000001"));
        strcpy(h.input,"-"); assert(!menu_apply_value(&h,&calibration));
        strcpy(h.input,"nan"); assert(!menu_apply_value(&h,&calibration));
        menu_back(&h); assert(h.page==MENU_CAMERA_CAL && h.selected==j);
    }
    menu_back(&h); assert(h.page==MENU_VISION && h.selected==3);
    menu_back(&h); assert(h.page==MENU_PARA && h.selected==0);
    MenuModel positive; menu_init(&positive); positive.page=MENU_KEYPAD;
    strcpy(positive.input,"1"); pick(&positive,"-"); menu_confirm(&positive);
    assert(!menu_apply_value(&positive,&calibration)); /* gains remain nonnegative */
    ControlSnapshot s={0}; s.mode=RUN_SPEED_DEBUG; s.feedback.valid=true; s.targets.left_mps=1; s.targets.right_mps=-2;
    s.feedback.speed_mps[0]=.5f; s.feedback.speed_mps[1]=0;
    uint8_t packet[20]; vofa_pack(&s,packet);
    const uint8_t expected[20]={0,0,0x80,0x3f,0,0,0,0x3f,0,0,0,0xc0,0,0,0,0,0,0,0x80,0x7f};
    assert(!memcmp(packet,expected,20));
    s.mode=RUN_SPEED_DEBUG; s.feedback.valid=false; vofa_pack(&s,packet);
    float target,measured; memcpy(&target,packet,4); memcpy(&measured,packet+4,4);
    assert(target==1 && isnan(measured));
    assert(packet[16]==0 && packet[17]==0 && packet[18]==0x80 && packet[19]==0x7f);
    ControlSnapshot stopped={0}; stopped.mode=RUN_STATIC_DEBUG; stopped.feedback.delta[1]=-6;
    param_defaults(&p); p.debug_speed_mps=.2f;
    assert(maintenance_save_check(&stopped,&p)==MAINT_OK); /* real-board regression: uncalibrated raw activity */
    stopped.feedback.valid=true; stopped.feedback.delta[1]=1;
    assert(maintenance_save_check(&stopped,&p)==MAINT_OK);
    stopped.feedback.speed_mps[1]=.49f;
    assert(maintenance_save_check(&stopped,&p)==MAINT_OK);
    stopped.feedback.speed_mps[1]=.5f; assert(maintenance_save_check(&stopped,&p)==MAINT_MOTION);
    stopped.feedback.delta[1]=0; stopped.feedback.speed_mps[1]=NAN;
    assert(maintenance_save_check(&stopped,&p)==MAINT_MOTION);
    stopped.feedback.valid=false; stopped.duty[0]=.3f;
    assert(maintenance_save_check(&stopped,&p)==MAINT_OUTPUT);
    stopped.duty[0]=0; stopped.mode=RUN_DUTY_DEBUG;
    assert(maintenance_save_check(&stopped,&p)==MAINT_MODE);
    stopped.mode=RUN_STATIC_DEBUG; p.debug_speed_mps=NAN;
    assert(maintenance_save_check(&stopped,&p)==MAINT_PARAM);
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
    p.body_length_m=.3f; p.body_width_m=.2f;
    assert(param_encode(&p,8,record) && param_decode(record,&q,&seq));
    assert(q.body_length_m==.3f && q.body_width_m==.2f);
    uint8_t schema5[APP_PARAM_RECORD_BYTES]; memcpy(schema5,record,sizeof schema5);
    float schema5_center=93.5f; memcpy(schema5+32+23*4,&schema5_center,4);
    put32(schema5+4,5); put32(schema5+12,52); put32(schema5+248,app_crc32(schema5,248));
    assert(param_decode(schema5,&q,&seq) && q.center_x_px==93.5f);
    assert(q.body_length_m==.3f && q.body_width_m==.2f && q.debug_speed_mps==.2f);
    assert(q.ground_h0==CAL_GROUND_H0 && q.ground_h2==CAL_GROUND_H2);
    uint8_t cropped_record[APP_PARAM_RECORD_BYTES];
    p.center_x_px=62.5f; p.ground_h2=-.6f;
    assert(param_encode(&p,8,cropped_record));
    put32(cropped_record+4,6); put32(cropped_record+12,52); put32(cropped_record+248,app_crc32(cropped_record,248));
    assert(param_decode(cropped_record,&q,&seq) && q.center_x_px==92.5f);
    assert(q.ground_h2==CAL_GROUND_H2 && q.debug_speed_mps==.2f && q.track_m==p.track_m);
    uint8_t old_signs[APP_PARAM_RECORD_BYTES];
    memcpy(old_signs,record,sizeof old_signs);
    put32(old_signs+4,7); put32(old_signs+12,52); put32(old_signs+28,0);
    put32(old_signs+248,app_crc32(old_signs,248));
    assert(param_decode(old_signs,&q,&seq));
    assert(q.encoder_sign[0]==-1 && q.encoder_sign[1]==1);
    assert(q.track_m==p.track_m && q.debug_speed_mps==p.debug_speed_mps);
    put32(old_signs+4,8); put32(old_signs+12,52); put32(old_signs+28,1); /* v1.50.17..19: left -, right + */
    put32(old_signs+248,app_crc32(old_signs,248));
    assert(param_decode(old_signs,&q,&seq));
    assert(q.encoder_sign[0]==-1 && q.encoder_sign[1]==1);
    assert(q.debug_speed_mps==p.debug_speed_mps && q.track_m==p.track_m);
    put32(old_signs+4,9); put32(old_signs+12,52); put32(old_signs+28,3); /* v1.50.20: both - */
    put32(old_signs+248,app_crc32(old_signs,248));
    assert(param_decode(old_signs,&q,&seq));
    assert(q.encoder_sign[0]==-1 && q.encoder_sign[1]==1);
    assert(q.debug_speed_mps==p.debug_speed_mps && q.track_m==p.track_m);
    q.encoder_sign[0]=1; q.encoder_sign[1]=-1;
    assert(param_encode(&q,8,old_signs) && param_decode(old_signs,&q,&seq));
    assert(q.encoder_sign[0]==1 && q.encoder_sign[1]==-1);
    AppParam speed_range; param_defaults(&speed_range);
    assert(speed_range.speed_limit_mps==5);
    MenuModel speed_edit; menu_init(&speed_edit); speed_edit.field=9;
    strcpy(speed_edit.input,"5"); assert(menu_apply_value(&speed_edit,&speed_range));
    strcpy(speed_edit.input,"5.01"); assert(!menu_apply_value(&speed_edit,&speed_range));
    speed_range.debug_speed_mps=.2f; speed_range.speed_limit_mps=.4f;
    assert(param_encode(&speed_range,20,old_signs));
    put32(old_signs+4,10); put32(old_signs+12,52); put32(old_signs+248,app_crc32(old_signs,248));
    assert(param_decode(old_signs,&q,&seq));
    assert(q.speed_limit_mps==5 && q.debug_speed_mps==.2f);
    q.speed_limit_mps=2;
    assert(param_encode(&q,21,old_signs) && param_decode(old_signs,&q,&seq));
    assert(q.speed_limit_mps==2); /* later saved limits remain configurable */
    put32(old_signs+4,11); put32(old_signs+12,52);
    put32(old_signs+248,app_crc32(old_signs,248));
    assert(param_decode(old_signs,&q,&seq) && q.tune_speed_mps==APP_TUNE_SPEED_DEFAULT_MPS);
    assert(q.speed_limit_mps==2);
    MenuModel tune_menu; menu_init(&tune_menu); tune_menu.page=MENU_PARAMS;
    tune_menu.selected=12;
    assert(!strcmp(menu_label(&tune_menu,12),"tune speed m/s"));
    menu_confirm(&tune_menu); strcpy(tune_menu.input,"0.65");
    assert(menu_apply_value(&tune_menu,&q) && param_encode(&q,22,old_signs));
    assert(param_decode(old_signs,&q,&seq) && fabsf(q.tune_speed_mps-.65f)<.0001f);
    MenuModel accel_menu; menu_init(&accel_menu); accel_menu.page=MENU_PARAMS;
    accel_menu.selected=14;
    assert(!strcmp(menu_label(&accel_menu,14),"accel m/s2"));
    menu_confirm(&accel_menu);
    assert(fabsf(menu_param_value(&accel_menu,&q)-.3f)<.0001f);
    strcpy(accel_menu.input,"0"); assert(!menu_apply_value(&accel_menu,&q));
    strcpy(accel_menu.input,"5.01"); assert(!menu_apply_value(&accel_menu,&q));
    strcpy(accel_menu.input,"0.8"); assert(menu_apply_value(&accel_menu,&q));
    assert(param_encode(&q,23,old_signs) && param_decode(old_signs,&q,&seq));
    assert(fabsf(q.accel_mps2-.8f)<.0001f && fabsf(q.decel_mps2-.6f)<.0001f);
    put32(old_signs+4,12); put32(old_signs+12,53);
    put32(old_signs+248,app_crc32(old_signs,248));
    assert(param_decode(old_signs,&q,&seq) && q.tune_duration_s==10);
    MenuModel time_menu; menu_init(&time_menu); time_menu.page=MENU_PARAMS;
    time_menu.selected=13;
    assert(!strcmp(menu_label(&time_menu,13),"tune time s"));
    menu_confirm(&time_menu);
    strcpy(time_menu.input,"30.1"); assert(!menu_apply_value(&time_menu,&q));
    strcpy(time_menu.input,"4.5"); assert(menu_apply_value(&time_menu,&q));
    assert(param_encode(&q,24,old_signs) && param_decode(old_signs,&q,&seq));
    assert(q.tune_duration_s==4.5f && fabsf(q.tune_speed_mps-.65f)<.0001f);
    float old_center=93.5f; memcpy(record+32+23*4,&old_center,4);
    put32(record+4,4); put32(record+12,50); memset(record+232,255,8);
    put32(record+248,app_crc32(record,248));
    assert(param_decode(record,&q,&seq) && q.body_length_m==0 && q.body_width_m==0 && q.debug_speed_mps==.2f);
    assert(q.kp_left==p.kp_left && q.ki_right==p.ki_right && q.track_m==p.track_m);
    assert(fabsf(q.center_x_px-93.5f)<1e-6f);
    assert(q.ground_h0==CAL_GROUND_H0 && q.ground_h7==CAL_GROUND_H7);
    uint8_t migrated[APP_PARAM_RECORD_BYTES];
    q.center_x_px=61; q.ground_h2=-.42f;
    assert(param_encode(&q,9,migrated) && migrated[4]==13);
    AppParam again; uint32_t migrated_seq;
    assert(param_decode(migrated,&again,&migrated_seq));
    assert(again.center_x_px==61 && again.ground_h2==-.42f && migrated_seq==9);
    put32(record+4,2); put32(record+12,49); memset(record+32+49*4,255,4);
    put32(record+248,app_crc32(record,248));
    assert(param_decode(record,&q,&seq) && q.debug_speed_mps==0 && q.kp_left==p.kp_left);
    put32(record+4,3); put32(record+248,app_crc32(record,248));
    assert(!param_decode(record,&q,&seq));
    puts("PASS: five-way menu selection, KEY1 back, keypad SAVE, keypad validation, paired gains, VOFA bytes, Flash migration");
    return 0;
}
