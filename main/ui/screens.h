#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *main_page;
    lv_obj_t *ch_voltage;
    lv_obj_t *ch_on_off_page;
    lv_obj_t *setup_wifi;
    lv_obj_t *setup_bb3;
    lv_obj_t *kbd_setup_bb3;
    lv_obj_t *kbd_setup_voltage;
    lv_obj_t *kbd_setup_wifi;
    lv_obj_t *lbl_ch1_a;
    lv_obj_t *lbl_ch1_status;
    lv_obj_t *lbl_ch1_v;
    lv_obj_t *lbl_ch2_a;
    lv_obj_t *lbl_ch2_status;
    lv_obj_t *lbl_ch2_v;
    lv_obj_t *lbl_conn_status;
    lv_obj_t *lbl_conn_status__1_;
    lv_obj_t *obj0;
    lv_obj_t *obj1;
    lv_obj_t *obj10;
    lv_obj_t *obj11;
    lv_obj_t *obj12;
    lv_obj_t *obj13;
    lv_obj_t *obj14;
    lv_obj_t *obj15;
    lv_obj_t *obj2;
    lv_obj_t *obj3;
    lv_obj_t *obj4;
    lv_obj_t *obj5;
    lv_obj_t *obj6;
    lv_obj_t *obj7;
    lv_obj_t *obj8;
    lv_obj_t *obj9;
    lv_obj_t *txt_bb3_ip;
    lv_obj_t *txt_ch1_i;
    lv_obj_t *txt_ch1_v;
    lv_obj_t *txt_ch2_i;
    lv_obj_t *txt_ch2_v;
    lv_obj_t *txt_pass;
    lv_obj_t *txt_ssid;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_MAIN_PAGE = 1,
    SCREEN_ID_CH_VOLTAGE = 2,
    SCREEN_ID_CH_ON_OFF_PAGE = 3,
    SCREEN_ID_SETUP_WIFI = 4,
    SCREEN_ID_SETUP_BB3 = 5,
};

void create_screen_main_page();
void tick_screen_main_page();

void create_screen_ch_voltage();
void tick_screen_ch_voltage();

void create_screen_ch_on_off_page();
void tick_screen_ch_on_off_page();

void create_screen_setup_wifi();
void tick_screen_setup_wifi();

void create_screen_setup_bb3();
void tick_screen_setup_bb3();

void create_screens();
void tick_screen(int screen_index);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/