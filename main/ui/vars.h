#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations



// Flow global variables

enum FlowGlobalVariables {
    FLOW_GLOBAL_VARIABLE_NONE
};

// Native global variables

extern const char *get_var_wifi_ssid();
extern void set_var_wifi_ssid(const char *value);
extern const char *get_var_wifi_pass();
extern void set_var_wifi_pass(const char *value);
extern const char *get_var_wifi_ip();
extern void set_var_wifi_ip(const char *value);
extern bool get_var_wifi_connected();
extern void set_var_wifi_connected(bool value);
extern const char *get_var_bb3_ip();
extern void set_var_bb3_ip(const char *value);
extern bool get_var_bb3_connected();
extern void set_var_bb3_connected(bool value);
extern bool get_var_scpi_updated();
extern void set_var_scpi_updated(bool value);
extern bool get_var_ch1_update();
extern void set_var_ch1_update(bool value);
extern bool get_var_ch1_power_on();
extern void set_var_ch1_power_on(bool value);
extern bool get_var_ch2_update();
extern void set_var_ch2_update(bool value);
extern bool get_var_ch2_power_on();
extern void set_var_ch2_power_on(bool value);
extern const char *get_var_bb3_response();
extern void set_var_bb3_response(const char *value);
extern const char *get_var_ch1_v();
extern void set_var_ch1_v(const char *value);
extern const char *get_var_ch1_a();
extern void set_var_ch1_a(const char *value);
extern const char *get_var_ch2_v();
extern void set_var_ch2_v(const char *value);
extern const char *get_var_ch2_a();
extern void set_var_ch2_a(const char *value);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/