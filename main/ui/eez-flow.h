/*
 * eez-framework
 *
 * MIT License
 * Copyright 2024 Envox d.o.o.
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#pragma once

#define EEZ_FOR_LVGL 1
#define EEZ_FOR_LVGL_LZ4_OPTION 0
#define EEZ_FOR_LVGL_SHA256_OPTION 0
#define EEZ_FLOW_QUEUE_SIZE 1000
#define EEZ_FLOW_EVAL_STACK_SIZE 20

// -----------------------------------------------------------------------------
// conf-internal.h
// -----------------------------------------------------------------------------
#ifdef __cplusplus
    #ifdef __has_include
        #if __has_include("eez-framework-conf.h")
            #include "eez-framework-conf.h"
        #endif
    #endif
#endif
#if ARDUINO
    #define EEZ_FOR_LVGL 1
    #ifndef LV_LVGL_H_INCLUDE_SIMPLE
        #define LV_LVGL_H_INCLUDE_SIMPLE
    #endif
    #include <Arduino.h>
#endif
#if EEZ_FOR_LVGL
    #define EEZ_OPTION_GUI 0
#endif
#ifndef EEZ_OPTION_GUI
    #define EEZ_OPTION_GUI 1
#endif
#ifndef OPTION_KEYBOARD
#define OPTION_KEYBOARD 0
#endif
#ifndef OPTION_MOUSE
#define OPTION_MOUSE 0
#endif
#ifndef OPTION_KEYPAD
#define OPTION_KEYPAD 0
#endif
#ifndef CUSTOM_VALUE_TYPES
#define CUSTOM_VALUE_TYPES
#endif
#ifdef __cplusplus
    #if EEZ_OPTION_GUI
        #ifdef __has_include
            #if __has_include("eez-framework-gui-conf.h")
                #include <eez-framework-gui-conf.h>
            #endif
        #endif
        #ifndef EEZ_OPTION_GUI_ANIMATIONS
            #define EEZ_OPTION_GUI_ANIMATIONS 1
        #endif
    #endif
#endif
#ifndef EEZ_FOR_LVGL_LZ4_OPTION
#define EEZ_FOR_LVGL_LZ4_OPTION 1
#endif
#ifndef EEZ_FOR_LVGL_SHA256_OPTION
#define EEZ_FOR_LVGL_SHA256_OPTION 1
#endif
#ifdef __cplusplus

// -----------------------------------------------------------------------------
// core/unit.h
// -----------------------------------------------------------------------------
#define INFINITY_SYMBOL "\x91"
#define DEGREE_SYMBOL "\x8a"
namespace eez {
enum Unit {
    UNIT_UNKNOWN = 255,
    UNIT_NONE = 0,
    UNIT_VOLT,
    UNIT_MILLI_VOLT,
    UNIT_AMPER,
    UNIT_MILLI_AMPER,
    UNIT_MICRO_AMPER,
    UNIT_WATT,
    UNIT_MILLI_WATT,
    UNIT_SECOND,
    UNIT_MILLI_SECOND,
    UNIT_CELSIUS,
    UNIT_RPM,
    UNIT_OHM,
    UNIT_KOHM,
    UNIT_MOHM,
    UNIT_PERCENT,
    UNIT_HERTZ,
    UNIT_MILLI_HERTZ,
    UNIT_KHERTZ,
    UNIT_MHERTZ,
    UNIT_JOULE,
    UNIT_FARAD,
    UNIT_MILLI_FARAD,
    UNIT_MICRO_FARAD,
    UNIT_NANO_FARAD,
    UNIT_PICO_FARAD,
    UNIT_MINUTE,
    UNIT_VOLT_AMPERE,
    UNIT_VOLT_AMPERE_REACTIVE,
	UNIT_DEGREE,
	UNIT_VOLT_PP,
	UNIT_MILLI_VOLT_PP,
	UNIT_AMPER_PP,
	UNIT_MILLI_AMPER_PP,
	UNIT_MICRO_AMPER_PP,
};
extern const char *g_unitNames[];
inline const char *getUnitName(Unit unit) {
	if (unit == UNIT_UNKNOWN) {
		return "";
	}
    return g_unitNames[unit];
}
Unit getUnitFromName(const char *unitName);
#if OPTION_SCPI
int getScpiUnit(Unit unit);
#endif
Unit getBaseUnit(Unit unit);
float getUnitFactor(Unit unit);
Unit findDerivedUnit(float value, Unit unit);
Unit getSmallerUnit(Unit unit, float min, float precision);
Unit getBiggestUnit(Unit unit, float max);
Unit getSmallestUnit(Unit unit, float min, float precision);
}
// -----------------------------------------------------------------------------
// core/value_types.h
// -----------------------------------------------------------------------------
#define VALUE_TYPES \
    VALUE_TYPE(UNDEFINED)                           \
    VALUE_TYPE(NULL)                                \
    VALUE_TYPE(BOOLEAN)                             \
    VALUE_TYPE(INT8)                                \
    VALUE_TYPE(UINT8)                               \
    VALUE_TYPE(INT16)                               \
    VALUE_TYPE(UINT16)                              \
    VALUE_TYPE(INT32)                               \
    VALUE_TYPE(UINT32)                              \
    VALUE_TYPE(INT64)                               \
    VALUE_TYPE(UINT64)                              \
    VALUE_TYPE(FLOAT)                               \
	VALUE_TYPE(DOUBLE)                              \
    VALUE_TYPE(STRING)                              \
    VALUE_TYPE(STRING_ASSET)                        \
    VALUE_TYPE(ARRAY)                               \
    VALUE_TYPE(ARRAY_ASSET)                         \
	VALUE_TYPE(STRING_REF)                          \
    VALUE_TYPE(ARRAY_REF)                           \
    VALUE_TYPE(BLOB_REF)                            \
    VALUE_TYPE(STREAM)                              \
    VALUE_TYPE(DATE)                                \
	VALUE_TYPE(VERSIONED_STRING)                    \
	VALUE_TYPE(VALUE_PTR)                           \
    VALUE_TYPE(ARRAY_ELEMENT_VALUE)                 \
    VALUE_TYPE(FLOW_OUTPUT)                         \
    VALUE_TYPE(NATIVE_VARIABLE)                     \
    VALUE_TYPE(ERROR)                               \
    VALUE_TYPE(RANGE)                               \
    VALUE_TYPE(POINTER)                             \
    VALUE_TYPE(ENUM)                                \
    VALUE_TYPE(IP_ADDRESS)                          \
    VALUE_TYPE(TIME_ZONE)                           \
    VALUE_TYPE(YT_DATA_GET_VALUE_FUNCTION_POINTER)  \
    VALUE_TYPE(WIDGET)                              \
    VALUE_TYPE(JSON)                                \
    VALUE_TYPE(JSON_MEMBER_VALUE)                   \
    CUSTOM_VALUE_TYPES
namespace eez {
#define VALUE_TYPE(NAME) VALUE_TYPE_##NAME,
enum ValueType {
	VALUE_TYPES
};
#undef VALUE_TYPE
}
// -----------------------------------------------------------------------------
// core/alloc.h
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <math.h>
#include <new>
#if OPTION_SCPI
#include <scpi/scpi.h>
#endif
namespace eez {
void initAllocHeap(uint8_t *heap, size_t heapSize);
void *alloc(size_t size, uint32_t id);
void free(void *ptr);
template<class T> struct ObjectAllocator {
	static T *allocate(uint32_t id) {
		auto ptr = alloc(sizeof(T), id);
		return new (ptr) T;
	}
	static void deallocate(T* ptr) {
		ptr->~T();
		free(ptr);
	}
};
#if OPTION_SCPI
void dumpAlloc(scpi_t *context);
#endif
void getAllocInfo(uint32_t &free, uint32_t &alloc);
}
// -----------------------------------------------------------------------------
// flow/flow_defs_v3.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
namespace defs_v3 {
enum ComponentTypes {
    COMPONENT_TYPE_NONE = 0,
    COMPONENT_TYPE_CONTAINER_WIDGET = 1,
    COMPONENT_TYPE_LIST_WIDGET = 2,
    COMPONENT_TYPE_GRID_WIDGET = 3,
    COMPONENT_TYPE_SELECT_WIDGET = 4,
    COMPONENT_TYPE_DISPLAY_DATA_WIDGET = 5,
    COMPONENT_TYPE_TEXT_WIDGET = 6,
    COMPONENT_TYPE_MULTILINE_TEXT_WIDGET = 7,
    COMPONENT_TYPE_RECTANGLE_WIDGET = 8,
    COMPONENT_TYPE_BITMAP_WIDGET = 9,
    COMPONENT_TYPE_BUTTON_WIDGET = 10,
    COMPONENT_TYPE_TOGGLE_BUTTON_WIDGET = 11,
    COMPONENT_TYPE_BUTTON_GROUP_WIDGET = 12,
    COMPONENT_TYPE_BAR_GRAPH_WIDGET = 14,
    COMPONENT_TYPE_USER_WIDGET_WIDGET = 15,
    COMPONENT_TYPE_YT_GRAPH_WIDGET = 16,
    COMPONENT_TYPE_UP_DOWN_WIDGET = 17,
    COMPONENT_TYPE_LIST_GRAPH_WIDGET = 18,
    COMPONENT_TYPE_APP_VIEW_WIDGET = 19,
    COMPONENT_TYPE_SCROLL_BAR_WIDGET = 20,
    COMPONENT_TYPE_PROGRESS_WIDGET = 21,
    COMPONENT_TYPE_CANVAS_WIDGET = 22,
    COMPONENT_TYPE_GAUGE_EMBEDDED_WIDGET = 23,
    COMPONENT_TYPE_INPUT_EMBEDDED_WIDGET = 24,
    COMPONENT_TYPE_ROLLER_WIDGET = 25,
    COMPONENT_TYPE_SWITCH_WIDGET = 26,
    COMPONENT_TYPE_SLIDER_WIDGET = 27,
    COMPONENT_TYPE_DROP_DOWN_LIST_WIDGET = 28,
    COMPONENT_TYPE_LINE_CHART_EMBEDDED_WIDGET = 29,
    COMPONENT_TYPE_QR_CODE_WIDGET = 30,
    COMPONENT_TYPE_START_ACTION = 1001,
    COMPONENT_TYPE_END_ACTION = 1002,
    COMPONENT_TYPE_INPUT_ACTION = 1003,
    COMPONENT_TYPE_OUTPUT_ACTION = 1004,
    COMPONENT_TYPE_WATCH_VARIABLE_ACTION = 1005,
    COMPONENT_TYPE_EVAL_EXPR_ACTION = 1006,
    COMPONENT_TYPE_SET_VARIABLE_ACTION = 1007,
    COMPONENT_TYPE_SWITCH_ACTION = 1008,
    COMPONENT_TYPE_COMPARE_ACTION = 1009,
    COMPONENT_TYPE_IS_TRUE_ACTION = 1010,
    COMPONENT_TYPE_CONSTANT_ACTION = 1011,
    COMPONENT_TYPE_LOG_ACTION = 1012,
    COMPONENT_TYPE_CALL_ACTION_ACTION = 1013,
    COMPONENT_TYPE_DELAY_ACTION = 1014,
    COMPONENT_TYPE_ERROR_ACTION = 1015,
    COMPONENT_TYPE_CATCH_ERROR_ACTION = 1016,
    COMPONENT_TYPE_COUNTER_ACTION = 1017,
    COMPONENT_TYPE_LOOP_ACTION = 1018,
    COMPONENT_TYPE_SHOW_PAGE_ACTION = 1019,
    COMPONENT_TYPE_SCPI_ACTION = 1020,
    COMPONENT_TYPE_SHOW_MESSAGE_BOX_ACTION = 1021,
    COMPONENT_TYPE_SHOW_KEYBOARD_ACTION = 1022,
    COMPONENT_TYPE_SHOW_KEYPAD_ACTION = 1023,
    COMPONENT_TYPE_NOOP_ACTION = 1024,
    COMPONENT_TYPE_COMMENT_ACTION = 1025,
    COMPONENT_TYPE_SELECT_LANGUAGE_ACTION = 1026,
    COMPONENT_TYPE_SET_PAGE_DIRECTION_ACTION = 1027,
    COMPONENT_TYPE_ANIMATE_ACTION = 1028,
    COMPONENT_TYPE_ON_EVENT_ACTION = 1029,
    COMPONENT_TYPE_LVGL_ACTION = 1030,
    COMPONENT_TYPE_OVERRIDE_STYLE_ACTION = 1031,
    COMPONENT_TYPE_SORT_ARRAY_ACTION = 1032,
    COMPONENT_TYPE_LVGL_USER_WIDGET_WIDGET = 1033,
    COMPONENT_TYPE_TEST_AND_SET_ACTION = 1034,
    COMPONENT_TYPE_MQTT_INIT_ACTION = 1035,
    COMPONENT_TYPE_MQTT_CONNECT_ACTION = 1036,
    COMPONENT_TYPE_MQTT_DISCONNECT_ACTION = 1037,
    COMPONENT_TYPE_MQTT_EVENT_ACTION = 1038,
    COMPONENT_TYPE_MQTT_SUBSCRIBE_ACTION = 1039,
    COMPONENT_TYPE_MQTT_UNSUBSCRIBE_ACTION = 1040,
    COMPONENT_TYPE_MQTT_PUBLISH_ACTION = 1041,
    COMPONENT_TYPE_LABEL_IN_ACTION = 1042,
    COMPONENT_TYPE_LABEL_OUT_ACTION = 1043,
    FIRST_DASHBOARD_ACTION_COMPONENT_TYPE = 10000,
    FIRST_DASHBOARD_WIDGET_COMPONENT_TYPE = 20000
};
enum Component_CONTAINER_WIDGET_Properties {
    CONTAINER_WIDGET_PROPERTY_DATA = 0,
    CONTAINER_WIDGET_PROPERTY_VISIBLE = 1,
    CONTAINER_WIDGET_PROPERTY_TAB_TITLE = 2,
    CONTAINER_WIDGET_PROPERTY_OVERLAY = 3
};
enum Component_LIST_WIDGET_Properties {
    LIST_WIDGET_PROPERTY_DATA = 0,
    LIST_WIDGET_PROPERTY_VISIBLE = 1,
    LIST_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_GRID_WIDGET_Properties {
    GRID_WIDGET_PROPERTY_DATA = 0,
    GRID_WIDGET_PROPERTY_VISIBLE = 1,
    GRID_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_SELECT_WIDGET_Properties {
    SELECT_WIDGET_PROPERTY_DATA = 0,
    SELECT_WIDGET_PROPERTY_VISIBLE = 1,
    SELECT_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_DISPLAY_DATA_WIDGET_Properties {
    DISPLAY_DATA_WIDGET_PROPERTY_DATA = 0,
    DISPLAY_DATA_WIDGET_PROPERTY_VISIBLE = 1,
    DISPLAY_DATA_WIDGET_PROPERTY_TAB_TITLE = 2,
    DISPLAY_DATA_WIDGET_PROPERTY_REFRESH_RATE = 3
};
enum Component_TEXT_WIDGET_Properties {
    TEXT_WIDGET_PROPERTY_DATA = 0,
    TEXT_WIDGET_PROPERTY_VISIBLE = 1,
    TEXT_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_MULTILINE_TEXT_WIDGET_Properties {
    MULTILINE_TEXT_WIDGET_PROPERTY_DATA = 0,
    MULTILINE_TEXT_WIDGET_PROPERTY_VISIBLE = 1,
    MULTILINE_TEXT_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_RECTANGLE_WIDGET_Properties {
    RECTANGLE_WIDGET_PROPERTY_DATA = 0,
    RECTANGLE_WIDGET_PROPERTY_VISIBLE = 1,
    RECTANGLE_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_BITMAP_WIDGET_Properties {
    BITMAP_WIDGET_PROPERTY_DATA = 0,
    BITMAP_WIDGET_PROPERTY_VISIBLE = 1,
    BITMAP_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_BUTTON_WIDGET_Properties {
    BUTTON_WIDGET_PROPERTY_DATA = 0,
    BUTTON_WIDGET_PROPERTY_VISIBLE = 1,
    BUTTON_WIDGET_PROPERTY_TAB_TITLE = 2,
    BUTTON_WIDGET_PROPERTY_ENABLED = 3
};
enum Component_TOGGLE_BUTTON_WIDGET_Properties {
    TOGGLE_BUTTON_WIDGET_PROPERTY_DATA = 0,
    TOGGLE_BUTTON_WIDGET_PROPERTY_VISIBLE = 1,
    TOGGLE_BUTTON_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_BUTTON_GROUP_WIDGET_Properties {
    BUTTON_GROUP_WIDGET_PROPERTY_DATA = 0,
    BUTTON_GROUP_WIDGET_PROPERTY_VISIBLE = 1,
    BUTTON_GROUP_WIDGET_PROPERTY_TAB_TITLE = 2,
    BUTTON_GROUP_WIDGET_PROPERTY_SELECTED_BUTTON = 3
};
enum Component_BAR_GRAPH_WIDGET_Properties {
    BAR_GRAPH_WIDGET_PROPERTY_DATA = 0,
    BAR_GRAPH_WIDGET_PROPERTY_VISIBLE = 1,
    BAR_GRAPH_WIDGET_PROPERTY_TAB_TITLE = 2,
    BAR_GRAPH_WIDGET_PROPERTY_LINE1_DATA = 3,
    BAR_GRAPH_WIDGET_PROPERTY_LINE2_DATA = 4,
    BAR_GRAPH_WIDGET_PROPERTY_MIN = 5,
    BAR_GRAPH_WIDGET_PROPERTY_MAX = 6,
    BAR_GRAPH_WIDGET_PROPERTY_REFRESH_RATE = 7
};
enum Component_USER_WIDGET_WIDGET_Properties {
    USER_WIDGET_WIDGET_PROPERTY_DATA = 0,
    USER_WIDGET_WIDGET_PROPERTY_VISIBLE = 1,
    USER_WIDGET_WIDGET_PROPERTY_TAB_TITLE = 2,
    USER_WIDGET_WIDGET_PROPERTY_CONTEXT = 3
};
enum Component_YT_GRAPH_WIDGET_Properties {
    YT_GRAPH_WIDGET_PROPERTY_DATA = 0,
    YT_GRAPH_WIDGET_PROPERTY_VISIBLE = 1,
    YT_GRAPH_WIDGET_PROPERTY_TAB_TITLE = 2,
    YT_GRAPH_WIDGET_PROPERTY_Y2_DATA = 3
};
enum Component_UP_DOWN_WIDGET_Properties {
    UP_DOWN_WIDGET_PROPERTY_DATA = 0,
    UP_DOWN_WIDGET_PROPERTY_VISIBLE = 1,
    UP_DOWN_WIDGET_PROPERTY_TAB_TITLE = 2,
    UP_DOWN_WIDGET_PROPERTY_MIN = 3,
    UP_DOWN_WIDGET_PROPERTY_MAX = 4
};
enum Component_LIST_GRAPH_WIDGET_Properties {
    LIST_GRAPH_WIDGET_PROPERTY_DATA = 0,
    LIST_GRAPH_WIDGET_PROPERTY_VISIBLE = 1,
    LIST_GRAPH_WIDGET_PROPERTY_TAB_TITLE = 2,
    LIST_GRAPH_WIDGET_PROPERTY_DWELL_DATA = 3,
    LIST_GRAPH_WIDGET_PROPERTY_Y1_DATA = 4,
    LIST_GRAPH_WIDGET_PROPERTY_Y2_DATA = 5,
    LIST_GRAPH_WIDGET_PROPERTY_CURSOR_DATA = 6
};
enum Component_APP_VIEW_WIDGET_Properties {
    APP_VIEW_WIDGET_PROPERTY_DATA = 0,
    APP_VIEW_WIDGET_PROPERTY_VISIBLE = 1,
    APP_VIEW_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_SCROLL_BAR_WIDGET_Properties {
    SCROLL_BAR_WIDGET_PROPERTY_DATA = 0,
    SCROLL_BAR_WIDGET_PROPERTY_VISIBLE = 1,
    SCROLL_BAR_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_PROGRESS_WIDGET_Properties {
    PROGRESS_WIDGET_PROPERTY_DATA = 0,
    PROGRESS_WIDGET_PROPERTY_VISIBLE = 1,
    PROGRESS_WIDGET_PROPERTY_TAB_TITLE = 2,
    PROGRESS_WIDGET_PROPERTY_MIN = 3,
    PROGRESS_WIDGET_PROPERTY_MAX = 4
};
enum Component_CANVAS_WIDGET_Properties {
    CANVAS_WIDGET_PROPERTY_DATA = 0,
    CANVAS_WIDGET_PROPERTY_VISIBLE = 1,
    CANVAS_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_GAUGE_EMBEDDED_WIDGET_Properties {
    GAUGE_EMBEDDED_WIDGET_PROPERTY_DATA = 0,
    GAUGE_EMBEDDED_WIDGET_PROPERTY_VISIBLE = 1,
    GAUGE_EMBEDDED_WIDGET_PROPERTY_TAB_TITLE = 2,
    GAUGE_EMBEDDED_WIDGET_PROPERTY_MIN = 3,
    GAUGE_EMBEDDED_WIDGET_PROPERTY_MAX = 4,
    GAUGE_EMBEDDED_WIDGET_PROPERTY_THRESHOLD = 5,
    GAUGE_EMBEDDED_WIDGET_PROPERTY_UNIT = 6
};
enum Component_INPUT_EMBEDDED_WIDGET_Properties {
    INPUT_EMBEDDED_WIDGET_PROPERTY_DATA = 0,
    INPUT_EMBEDDED_WIDGET_PROPERTY_VISIBLE = 1,
    INPUT_EMBEDDED_WIDGET_PROPERTY_TAB_TITLE = 2,
    INPUT_EMBEDDED_WIDGET_PROPERTY_MIN = 3,
    INPUT_EMBEDDED_WIDGET_PROPERTY_MAX = 4,
    INPUT_EMBEDDED_WIDGET_PROPERTY_PRECISION = 5,
    INPUT_EMBEDDED_WIDGET_PROPERTY_UNIT = 6
};
enum Component_ROLLER_WIDGET_Properties {
    ROLLER_WIDGET_PROPERTY_DATA = 0,
    ROLLER_WIDGET_PROPERTY_VISIBLE = 1,
    ROLLER_WIDGET_PROPERTY_TAB_TITLE = 2,
    ROLLER_WIDGET_PROPERTY_MIN = 3,
    ROLLER_WIDGET_PROPERTY_MAX = 4,
    ROLLER_WIDGET_PROPERTY_TEXT = 5
};
enum Component_SWITCH_WIDGET_Properties {
    SWITCH_WIDGET_PROPERTY_DATA = 0,
    SWITCH_WIDGET_PROPERTY_VISIBLE = 1,
    SWITCH_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_SLIDER_WIDGET_Properties {
    SLIDER_WIDGET_PROPERTY_DATA = 0,
    SLIDER_WIDGET_PROPERTY_VISIBLE = 1,
    SLIDER_WIDGET_PROPERTY_TAB_TITLE = 2,
    SLIDER_WIDGET_PROPERTY_MIN = 3,
    SLIDER_WIDGET_PROPERTY_MAX = 4
};
enum Component_DROP_DOWN_LIST_WIDGET_Properties {
    DROP_DOWN_LIST_WIDGET_PROPERTY_DATA = 0,
    DROP_DOWN_LIST_WIDGET_PROPERTY_VISIBLE = 1,
    DROP_DOWN_LIST_WIDGET_PROPERTY_TAB_TITLE = 2,
    DROP_DOWN_LIST_WIDGET_PROPERTY_OPTIONS = 3
};
enum Component_LINE_CHART_EMBEDDED_WIDGET_Properties {
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_DATA = 0,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_VISIBLE = 1,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_TAB_TITLE = 2,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_X_VALUE = 3,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_SHOW_TITLE = 4,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_SHOW_LEGEND = 5,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_SHOW_XAXIS = 6,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_SHOW_YAXIS = 7,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_SHOW_GRID = 8,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_TITLE = 9,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_Y_AXIS_RANGE_FROM = 10,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_Y_AXIS_RANGE_TO = 11,
    LINE_CHART_EMBEDDED_WIDGET_PROPERTY_MARKER = 12
};
enum Component_QR_CODE_WIDGET_Properties {
    QR_CODE_WIDGET_PROPERTY_DATA = 0,
    QR_CODE_WIDGET_PROPERTY_VISIBLE = 1,
    QR_CODE_WIDGET_PROPERTY_TAB_TITLE = 2
};
enum Component_WATCH_VARIABLE_ACTION_COMPONENT_Properties {
    WATCH_VARIABLE_ACTION_COMPONENT_PROPERTY_VARIABLE = 0
};
enum Component_EVAL_EXPR_ACTION_COMPONENT_Properties {
    EVAL_EXPR_ACTION_COMPONENT_PROPERTY_EXPRESSION = 0
};
enum Component_COMPARE_ACTION_COMPONENT_Properties {
    COMPARE_ACTION_COMPONENT_PROPERTY_A = 0,
    COMPARE_ACTION_COMPONENT_PROPERTY_B = 1,
    COMPARE_ACTION_COMPONENT_PROPERTY_C = 2
};
enum Component_IS_TRUE_ACTION_COMPONENT_Properties {
    IS_TRUE_ACTION_COMPONENT_PROPERTY_VALUE = 0
};
enum Component_CONSTANT_ACTION_COMPONENT_Properties {
    CONSTANT_ACTION_COMPONENT_PROPERTY_VALUE = 0
};
enum Component_LOG_ACTION_COMPONENT_Properties {
    LOG_ACTION_COMPONENT_PROPERTY_VALUE = 0
};
enum Component_DELAY_ACTION_COMPONENT_Properties {
    DELAY_ACTION_COMPONENT_PROPERTY_MILLISECONDS = 0
};
enum Component_ERROR_ACTION_COMPONENT_Properties {
    ERROR_ACTION_COMPONENT_PROPERTY_MESSAGE = 0
};
enum Component_COUNTER_ACTION_COMPONENT_Properties {
    COUNTER_ACTION_COMPONENT_PROPERTY_COUNT_VALUE = 0
};
enum Component_LOOP_ACTION_COMPONENT_Properties {
    LOOP_ACTION_COMPONENT_PROPERTY_VARIABLE = 0,
    LOOP_ACTION_COMPONENT_PROPERTY_FROM = 1,
    LOOP_ACTION_COMPONENT_PROPERTY_TO = 2,
    LOOP_ACTION_COMPONENT_PROPERTY_STEP = 3
};
enum Component_SCPI_ACTION_COMPONENT_Properties {
    SCPI_ACTION_COMPONENT_PROPERTY_INSTRUMENT = 0,
    SCPI_ACTION_COMPONENT_PROPERTY_TIMEOUT = 1,
    SCPI_ACTION_COMPONENT_PROPERTY_DELAY = 2
};
enum Component_SHOW_MESSAGE_BOX_ACTION_COMPONENT_Properties {
    SHOW_MESSAGE_BOX_ACTION_COMPONENT_PROPERTY_MESSAGE = 0,
    SHOW_MESSAGE_BOX_ACTION_COMPONENT_PROPERTY_BUTTONS = 1
};
enum Component_SHOW_KEYBOARD_ACTION_COMPONENT_Properties {
    SHOW_KEYBOARD_ACTION_COMPONENT_PROPERTY_LABEL = 0,
    SHOW_KEYBOARD_ACTION_COMPONENT_PROPERTY_INITAL_TEXT = 1,
    SHOW_KEYBOARD_ACTION_COMPONENT_PROPERTY_MIN_CHARS = 2,
    SHOW_KEYBOARD_ACTION_COMPONENT_PROPERTY_MAX_CHARS = 3
};
enum Component_SHOW_KEYPAD_ACTION_COMPONENT_Properties {
    SHOW_KEYPAD_ACTION_COMPONENT_PROPERTY_LABEL = 0,
    SHOW_KEYPAD_ACTION_COMPONENT_PROPERTY_INITAL_VALUE = 1,
    SHOW_KEYPAD_ACTION_COMPONENT_PROPERTY_MIN = 2,
    SHOW_KEYPAD_ACTION_COMPONENT_PROPERTY_MAX = 3,
    SHOW_KEYPAD_ACTION_COMPONENT_PROPERTY_PRECISION = 4,
    SHOW_KEYPAD_ACTION_COMPONENT_PROPERTY_UNIT = 5
};
enum Component_SELECT_LANGUAGE_ACTION_COMPONENT_Properties {
    SELECT_LANGUAGE_ACTION_COMPONENT_PROPERTY_LANGUAGE = 0
};
enum Component_ANIMATE_ACTION_COMPONENT_Properties {
    ANIMATE_ACTION_COMPONENT_PROPERTY_FROM = 0,
    ANIMATE_ACTION_COMPONENT_PROPERTY_TO = 1,
    ANIMATE_ACTION_COMPONENT_PROPERTY_SPEED = 2
};
enum Component_SORT_ARRAY_ACTION_COMPONENT_Properties {
    SORT_ARRAY_ACTION_COMPONENT_PROPERTY_ARRAY = 0
};
enum Component_LVGL_USER_WIDGET_WIDGET_Properties {
    LVGL_USER_WIDGET_WIDGET_PROPERTY_DATA = 0,
    LVGL_USER_WIDGET_WIDGET_PROPERTY_VISIBLE = 1,
    LVGL_USER_WIDGET_WIDGET_PROPERTY_TAB_TITLE = 2,
    LVGL_USER_WIDGET_WIDGET_PROPERTY_HIDDEN_FLAG = 3,
    LVGL_USER_WIDGET_WIDGET_PROPERTY_CLICKABLE_FLAG = 4,
    LVGL_USER_WIDGET_WIDGET_PROPERTY_CHECKED_STATE = 5,
    LVGL_USER_WIDGET_WIDGET_PROPERTY_DISABLED_STATE = 6
};
enum Component_TEST_AND_SET_ACTION_COMPONENT_Properties {
    TEST_AND_SET_ACTION_COMPONENT_PROPERTY_VARIABLE = 0
};
enum Component_MQTT_INIT_ACTION_COMPONENT_Properties {
    MQTT_INIT_ACTION_COMPONENT_PROPERTY_CONNECTION = 0,
    MQTT_INIT_ACTION_COMPONENT_PROPERTY_PROTOCOL = 1,
    MQTT_INIT_ACTION_COMPONENT_PROPERTY_HOST = 2,
    MQTT_INIT_ACTION_COMPONENT_PROPERTY_PORT = 3,
    MQTT_INIT_ACTION_COMPONENT_PROPERTY_USER_NAME = 4,
    MQTT_INIT_ACTION_COMPONENT_PROPERTY_PASSWORD = 5
};
enum Component_MQTT_CONNECT_ACTION_COMPONENT_Properties {
    MQTT_CONNECT_ACTION_COMPONENT_PROPERTY_CONNECTION = 0
};
enum Component_MQTT_DISCONNECT_ACTION_COMPONENT_Properties {
    MQTT_DISCONNECT_ACTION_COMPONENT_PROPERTY_CONNECTION = 0
};
enum Component_MQTT_EVENT_ACTION_COMPONENT_Properties {
    MQTT_EVENT_ACTION_COMPONENT_PROPERTY_CONNECTION = 0
};
enum Component_MQTT_SUBSCRIBE_ACTION_COMPONENT_Properties {
    MQTT_SUBSCRIBE_ACTION_COMPONENT_PROPERTY_CONNECTION = 0,
    MQTT_SUBSCRIBE_ACTION_COMPONENT_PROPERTY_TOPIC = 1
};
enum Component_MQTT_UNSUBSCRIBE_ACTION_COMPONENT_Properties {
    MQTT_UNSUBSCRIBE_ACTION_COMPONENT_PROPERTY_CONNECTION = 0,
    MQTT_UNSUBSCRIBE_ACTION_COMPONENT_PROPERTY_TOPIC = 1
};
enum Component_MQTT_PUBLISH_ACTION_COMPONENT_Properties {
    MQTT_PUBLISH_ACTION_COMPONENT_PROPERTY_CONNECTION = 0,
    MQTT_PUBLISH_ACTION_COMPONENT_PROPERTY_TOPIC = 1,
    MQTT_PUBLISH_ACTION_COMPONENT_PROPERTY_PAYLOAD = 2
};
enum OperationTypes {
    OPERATION_TYPE_ADD = 0,
    OPERATION_TYPE_SUB = 1,
    OPERATION_TYPE_MUL = 2,
    OPERATION_TYPE_DIV = 3,
    OPERATION_TYPE_MOD = 4,
    OPERATION_TYPE_LEFT_SHIFT = 5,
    OPERATION_TYPE_RIGHT_SHIFT = 6,
    OPERATION_TYPE_BINARY_AND = 7,
    OPERATION_TYPE_BINARY_OR = 8,
    OPERATION_TYPE_BINARY_XOR = 9,
    OPERATION_TYPE_EQUAL = 10,
    OPERATION_TYPE_NOT_EQUAL = 11,
    OPERATION_TYPE_LESS = 12,
    OPERATION_TYPE_GREATER = 13,
    OPERATION_TYPE_LESS_OR_EQUAL = 14,
    OPERATION_TYPE_GREATER_OR_EQUAL = 15,
    OPERATION_TYPE_LOGICAL_AND = 16,
    OPERATION_TYPE_LOGICAL_OR = 17,
    OPERATION_TYPE_UNARY_PLUS = 18,
    OPERATION_TYPE_UNARY_MINUS = 19,
    OPERATION_TYPE_BINARY_ONE_COMPLEMENT = 20,
    OPERATION_TYPE_NOT = 21,
    OPERATION_TYPE_CONDITIONAL = 22,
    OPERATION_TYPE_SYSTEM_GET_TICK = 23,
    OPERATION_TYPE_FLOW_INDEX = 24,
    OPERATION_TYPE_FLOW_IS_PAGE_ACTIVE = 25,
    OPERATION_TYPE_FLOW_PAGE_TIMELINE_POSITION = 26,
    OPERATION_TYPE_FLOW_MAKE_VALUE = 27,
    OPERATION_TYPE_FLOW_MAKE_ARRAY_VALUE = 28,
    OPERATION_TYPE_FLOW_LANGUAGES = 29,
    OPERATION_TYPE_FLOW_TRANSLATE = 30,
    OPERATION_TYPE_FLOW_PARSE_INTEGER = 31,
    OPERATION_TYPE_FLOW_PARSE_FLOAT = 32,
    OPERATION_TYPE_FLOW_PARSE_DOUBLE = 33,
    OPERATION_TYPE_FLOW_TO_INTEGER = 71,
    OPERATION_TYPE_FLOW_GET_BITMAP_INDEX = 70,
    OPERATION_TYPE_FLOW_GET_BITMAP_AS_DATA_URL = 78,
    OPERATION_TYPE_CRYPTO_SHA256 = 74,
    OPERATION_TYPE_DATE_NOW = 34,
    OPERATION_TYPE_DATE_TO_STRING = 35,
    OPERATION_TYPE_DATE_TO_LOCALE_STRING = 59,
    OPERATION_TYPE_DATE_FROM_STRING = 36,
    OPERATION_TYPE_DATE_GET_YEAR = 60,
    OPERATION_TYPE_DATE_GET_MONTH = 61,
    OPERATION_TYPE_DATE_GET_DAY = 62,
    OPERATION_TYPE_DATE_GET_HOURS = 63,
    OPERATION_TYPE_DATE_GET_MINUTES = 64,
    OPERATION_TYPE_DATE_GET_SECONDS = 65,
    OPERATION_TYPE_DATE_GET_MILLISECONDS = 66,
    OPERATION_TYPE_DATE_MAKE = 67,
    OPERATION_TYPE_MATH_SIN = 37,
    OPERATION_TYPE_MATH_COS = 38,
    OPERATION_TYPE_MATH_POW = 68,
    OPERATION_TYPE_MATH_LOG = 39,
    OPERATION_TYPE_MATH_LOG10 = 40,
    OPERATION_TYPE_MATH_ABS = 41,
    OPERATION_TYPE_MATH_FLOOR = 42,
    OPERATION_TYPE_MATH_CEIL = 43,
    OPERATION_TYPE_MATH_ROUND = 44,
    OPERATION_TYPE_MATH_MIN = 45,
    OPERATION_TYPE_MATH_MAX = 46,
    OPERATION_TYPE_STRING_LENGTH = 47,
    OPERATION_TYPE_STRING_SUBSTRING = 48,
    OPERATION_TYPE_STRING_FIND = 49,
    OPERATION_TYPE_STRING_PAD_START = 50,
    OPERATION_TYPE_STRING_SPLIT = 51,
    OPERATION_TYPE_STRING_FROM_CODE_POINT = 72,
    OPERATION_TYPE_STRING_CODE_POINT_AT = 73,
    OPERATION_TYPE_ARRAY_LENGTH = 52,
    OPERATION_TYPE_ARRAY_SLICE = 53,
    OPERATION_TYPE_ARRAY_ALLOCATE = 54,
    OPERATION_TYPE_ARRAY_APPEND = 55,
    OPERATION_TYPE_ARRAY_INSERT = 56,
    OPERATION_TYPE_ARRAY_REMOVE = 57,
    OPERATION_TYPE_ARRAY_CLONE = 58,
    OPERATION_TYPE_BLOB_ALLOCATE = 75,
    OPERATION_TYPE_JSON_GET = 76,
    OPERATION_TYPE_JSON_CLONE = 77,
    OPERATION_TYPE_LVGL_METER_TICK_INDEX = 69
};
enum SystemStructures {
    SYSTEM_STRUCTURE_CLICK_EVENT = 8192,
    SYSTEM_STRUCTURE_CHECKBOX_CHANGE_EVENT = 8193,
    SYSTEM_STRUCTURE_TEXT_INPUT_CHANGE_EVENT = 8194,
    SYSTEM_STRUCTURE_DROP_DOWN_LIST_CHANGE_EVENT = 8195,
    SYSTEM_STRUCTURE_SLIDER_CHANGE_EVENT = 8196,
    SYSTEM_STRUCTURE_SWITCH_CHANGE_EVENT = 8197,
    SYSTEM_STRUCTURE_SCROLLBAR_STATE = 8198,
    SYSTEM_STRUCTURE_OBJECT_VARIABLE_STATUS = 8199,
    SYSTEM_STRUCTURE_RADIO_CHANGE_EVENT = 8200,
    SYSTEM_STRUCTURE_REG_EXP_RESULT = 8201,
    SYSTEM_STRUCTURE_SERIAL_PORT = 8202,
    SYSTEM_STRUCTURE_MQTT_MESSAGE = 8203,
    SYSTEM_STRUCTURE_TERMINAL_WIDGET_ON_DATA_PARAMS = 8204
};
enum ClickEventSystemStructureFields {
    SYSTEM_STRUCTURE_CLICK_EVENT_FIELD_INDEX = 0,
    SYSTEM_STRUCTURE_CLICK_EVENT_FIELD_INDEXES = 1,
    SYSTEM_STRUCTURE_CLICK_EVENT_NUM_FIELDS
};
enum CheckboxChangeEventSystemStructureFields {
    SYSTEM_STRUCTURE_CHECKBOX_CHANGE_EVENT_FIELD_INDEX = 0,
    SYSTEM_STRUCTURE_CHECKBOX_CHANGE_EVENT_FIELD_INDEXES = 1,
    SYSTEM_STRUCTURE_CHECKBOX_CHANGE_EVENT_FIELD_VALUE = 2,
    SYSTEM_STRUCTURE_CHECKBOX_CHANGE_EVENT_NUM_FIELDS
};
enum TextInputChangeEventSystemStructureFields {
    SYSTEM_STRUCTURE_TEXT_INPUT_CHANGE_EVENT_FIELD_INDEX = 0,
    SYSTEM_STRUCTURE_TEXT_INPUT_CHANGE_EVENT_FIELD_INDEXES = 1,
    SYSTEM_STRUCTURE_TEXT_INPUT_CHANGE_EVENT_FIELD_VALUE = 2,
    SYSTEM_STRUCTURE_TEXT_INPUT_CHANGE_EVENT_NUM_FIELDS
};
enum DropDownListChangeEventSystemStructureFields {
    SYSTEM_STRUCTURE_DROP_DOWN_LIST_CHANGE_EVENT_FIELD_INDEX = 0,
    SYSTEM_STRUCTURE_DROP_DOWN_LIST_CHANGE_EVENT_FIELD_INDEXES = 1,
    SYSTEM_STRUCTURE_DROP_DOWN_LIST_CHANGE_EVENT_FIELD_SELECTED_INDEX = 2,
    SYSTEM_STRUCTURE_DROP_DOWN_LIST_CHANGE_EVENT_NUM_FIELDS
};
enum SliderChangeEventSystemStructureFields {
    SYSTEM_STRUCTURE_SLIDER_CHANGE_EVENT_FIELD_INDEX = 0,
    SYSTEM_STRUCTURE_SLIDER_CHANGE_EVENT_FIELD_INDEXES = 1,
    SYSTEM_STRUCTURE_SLIDER_CHANGE_EVENT_FIELD_VALUE = 2,
    SYSTEM_STRUCTURE_SLIDER_CHANGE_EVENT_NUM_FIELDS
};
enum SwitchChangeEventSystemStructureFields {
    SYSTEM_STRUCTURE_SWITCH_CHANGE_EVENT_FIELD_INDEX = 0,
    SYSTEM_STRUCTURE_SWITCH_CHANGE_EVENT_FIELD_INDEXES = 1,
    SYSTEM_STRUCTURE_SWITCH_CHANGE_EVENT_FIELD_VALUE = 2,
    SYSTEM_STRUCTURE_SWITCH_CHANGE_EVENT_NUM_FIELDS
};
enum ScrollbarStateSystemStructureFields {
    SYSTEM_STRUCTURE_SCROLLBAR_STATE_FIELD_NUM_ITEMS = 0,
    SYSTEM_STRUCTURE_SCROLLBAR_STATE_FIELD_ITEMS_PER_PAGE = 1,
    SYSTEM_STRUCTURE_SCROLLBAR_STATE_FIELD_POSITION_INCREMENT = 2,
    SYSTEM_STRUCTURE_SCROLLBAR_STATE_FIELD_POSITION = 3,
    SYSTEM_STRUCTURE_SCROLLBAR_STATE_NUM_FIELDS
};
enum ObjectVariableStatusSystemStructureFields {
    SYSTEM_STRUCTURE_OBJECT_VARIABLE_STATUS_FIELD_LABEL = 0,
    SYSTEM_STRUCTURE_OBJECT_VARIABLE_STATUS_FIELD_IMAGE = 1,
    SYSTEM_STRUCTURE_OBJECT_VARIABLE_STATUS_FIELD_COLOR = 2,
    SYSTEM_STRUCTURE_OBJECT_VARIABLE_STATUS_FIELD_ERROR = 3,
    SYSTEM_STRUCTURE_OBJECT_VARIABLE_STATUS_NUM_FIELDS
};
enum RadioChangeEventSystemStructureFields {
    SYSTEM_STRUCTURE_RADIO_CHANGE_EVENT_FIELD_INDEX = 0,
    SYSTEM_STRUCTURE_RADIO_CHANGE_EVENT_FIELD_INDEXES = 1,
    SYSTEM_STRUCTURE_RADIO_CHANGE_EVENT_FIELD_CHECKED = 2,
    SYSTEM_STRUCTURE_RADIO_CHANGE_EVENT_NUM_FIELDS
};
enum RegExpResultSystemStructureFields {
    SYSTEM_STRUCTURE_REG_EXP_RESULT_FIELD_INDEX = 0,
    SYSTEM_STRUCTURE_REG_EXP_RESULT_FIELD_TEXTS = 1,
    SYSTEM_STRUCTURE_REG_EXP_RESULT_FIELD_INDICES = 2,
    SYSTEM_STRUCTURE_REG_EXP_RESULT_NUM_FIELDS
};
enum SerialPortSystemStructureFields {
    SYSTEM_STRUCTURE_SERIAL_PORT_FIELD_MANUFACTURER = 0,
    SYSTEM_STRUCTURE_SERIAL_PORT_FIELD_SERIAL_NUMBER = 1,
    SYSTEM_STRUCTURE_SERIAL_PORT_FIELD_PATH = 2,
    SYSTEM_STRUCTURE_SERIAL_PORT_NUM_FIELDS
};
enum MQTTMessageSystemStructureFields {
    SYSTEM_STRUCTURE_MQTT_MESSAGE_FIELD_TOPIC = 0,
    SYSTEM_STRUCTURE_MQTT_MESSAGE_FIELD_PAYLOAD = 1,
    SYSTEM_STRUCTURE_MQTT_MESSAGE_NUM_FIELDS
};
enum TerminalWidgetOnDataParamsSystemStructureFields {
    SYSTEM_STRUCTURE_TERMINAL_WIDGET_ON_DATA_PARAMS_FIELD_INDEX = 0,
    SYSTEM_STRUCTURE_TERMINAL_WIDGET_ON_DATA_PARAMS_FIELD_INDEXES = 1,
    SYSTEM_STRUCTURE_TERMINAL_WIDGET_ON_DATA_PARAMS_FIELD_DATA = 2,
    SYSTEM_STRUCTURE_TERMINAL_WIDGET_ON_DATA_PARAMS_NUM_FIELDS
};
enum ObjectTypes {
    OBJECT_TYPE_INSTRUMENT = 40960,
    OBJECT_TYPE_SERIAL_CONNECTION = 40961,
    OBJECT_TYPE_MQTT_CONNECTION = 40962,
    OBJECT_TYPE_TCP_CONNECTION = 40963
};
enum InstrumentObjectTypeFields {
    OBJECT_TYPE_INSTRUMENT_FIELD_ID = 0,
    OBJECT_TYPE_INSTRUMENT_FIELD_IS_CONNECTED = 1,
    OBJECT_TYPE_INSTRUMENT_FIELD_STATUS = 2,
    OBJECT_TYPE_INSTRUMENT_NUM_FIELDS
};
enum SerialConnectionObjectTypeFields {
    OBJECT_TYPE_SERIAL_CONNECTION_FIELD_PORT = 0,
    OBJECT_TYPE_SERIAL_CONNECTION_FIELD_BAUD_RATE = 1,
    OBJECT_TYPE_SERIAL_CONNECTION_FIELD_DATA_BITS = 2,
    OBJECT_TYPE_SERIAL_CONNECTION_FIELD_STOP_BITS = 3,
    OBJECT_TYPE_SERIAL_CONNECTION_FIELD_PARITY = 4,
    OBJECT_TYPE_SERIAL_CONNECTION_FIELD_IS_CONNECTED = 5,
    OBJECT_TYPE_SERIAL_CONNECTION_FIELD_ID = 6,
    OBJECT_TYPE_SERIAL_CONNECTION_FIELD_STATUS = 7,
    OBJECT_TYPE_SERIAL_CONNECTION_NUM_FIELDS
};
enum MQTTConnectionObjectTypeFields {
    OBJECT_TYPE_MQTT_CONNECTION_FIELD_PROTOCOL = 0,
    OBJECT_TYPE_MQTT_CONNECTION_FIELD_HOST = 1,
    OBJECT_TYPE_MQTT_CONNECTION_FIELD_PORT = 2,
    OBJECT_TYPE_MQTT_CONNECTION_FIELD_USER_NAME = 3,
    OBJECT_TYPE_MQTT_CONNECTION_FIELD_PASSWORD = 4,
    OBJECT_TYPE_MQTT_CONNECTION_FIELD_IS_CONNECTED = 5,
    OBJECT_TYPE_MQTT_CONNECTION_FIELD_ID = 6,
    OBJECT_TYPE_MQTT_CONNECTION_FIELD_STATUS = 7,
    OBJECT_TYPE_MQTT_CONNECTION_NUM_FIELDS
};
enum TCPConnectionObjectTypeFields {
    OBJECT_TYPE_TCP_CONNECTION_FIELD_IP_ADDRESS = 0,
    OBJECT_TYPE_TCP_CONNECTION_FIELD_PORT = 1,
    OBJECT_TYPE_TCP_CONNECTION_FIELD_IS_CONNECTED = 2,
    OBJECT_TYPE_TCP_CONNECTION_FIELD_ID = 3,
    OBJECT_TYPE_TCP_CONNECTION_FIELD_STATUS = 4,
    OBJECT_TYPE_TCP_CONNECTION_NUM_FIELDS
};
enum ArrayTypes {
    ARRAY_TYPE_INTEGER = 65543,
    ARRAY_TYPE_FLOAT = 65547,
    ARRAY_TYPE_DOUBLE = 65548,
    ARRAY_TYPE_BOOLEAN = 65538,
    ARRAY_TYPE_STRING = 65549,
    ARRAY_TYPE_DATE = 65557,
    ARRAY_TYPE_BLOB = 65555,
    ARRAY_TYPE_STREAM = 65556,
    ARRAY_TYPE_WIDGET = 65570,
    ARRAY_TYPE_JSON = 65571,
    ARRAY_TYPE_ANY = 15
};
}
}
}
// -----------------------------------------------------------------------------
// core/value.h
// -----------------------------------------------------------------------------
#include <string.h>
namespace eez {
static const size_t MAX_ITERATORS = 4;
#if EEZ_OPTION_GUI
namespace gui {
    class AppContext;
}
using gui::AppContext;
#endif
namespace gui {
    struct Style;
}
struct EnumValue {
    uint16_t enumValue;
    uint16_t enumDefinition;
};
#define VALUE_OPTIONS_REF (1 << 0)
#define STRING_OPTIONS_FILE_ELLIPSIS (1 << 1)
#define FLOAT_OPTIONS_LESS_THEN (1 << 1)
#define FLOAT_OPTIONS_FIXED_DECIMALS (1 << 2)
#define FLOAT_OPTIONS_GET_NUM_FIXED_DECIMALS(options) (((options) >> 3) & 0b111)
#define FLOAT_OPTIONS_SET_NUM_FIXED_DECIMALS(n) (FLOAT_OPTIONS_FIXED_DECIMALS | ((n & 0b111) << 3))
struct Value;
typedef bool (*CompareValueFunction)(const Value &a, const Value &b);
typedef void (*ValueToTextFunction)(const Value &value, char *text, int count);
typedef const char * (*ValueTypeNameFunction)(const Value &value);
typedef void (*CopyValueFunction)(Value &a, const Value &b);
extern CompareValueFunction g_valueTypeCompareFunctions[];
extern ValueToTextFunction g_valueTypeToTextFunctions[];
extern ValueTypeNameFunction g_valueTypeNames[];
struct PairOfUint8Value {
    uint8_t first;
    uint8_t second;
};
struct PairOfUint16Value {
    uint16_t first;
    uint16_t second;
};
struct PairOfInt16Value {
    int16_t first;
    int16_t second;
};
struct Ref {
	uint32_t refCounter;
    virtual ~Ref() {}
};
struct ArrayValue;
struct ArrayElementValue;
struct BlobRef;
#if defined(EEZ_DASHBOARD_API)
namespace flow {
    extern void dashboardObjectValueIncRef(int json);
    extern void dashboardObjectValueDecRef(int json);
}
#endif
struct Value {
  public:
    Value()
        : type(VALUE_TYPE_UNDEFINED), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), uint64Value(0)
    {
    }
	Value(int value)
        : type(VALUE_TYPE_INT32), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), int32Value(value)
    {
    }
	Value(const char *str)
        : type(VALUE_TYPE_STRING), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), strValue(str)
    {
    }
	Value(uint8_t version, const char *str)
        : type(VALUE_TYPE_VERSIONED_STRING), unit(version), options(0), dstValueType(VALUE_TYPE_UNDEFINED), strValue(str)
    {
    }
	Value(Value *pValue)
		: type(VALUE_TYPE_VALUE_PTR), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), pValueValue(pValue)
    {
	}
    Value(const char *str, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), strValue(str)
    {
    }
    Value(int value, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), int32Value(value)
    {
    }
    Value(int value, ValueType type, uint16_t options)
        : type(type), unit(UNIT_UNKNOWN), options(options), dstValueType(VALUE_TYPE_UNDEFINED), int32Value(value)
    {
    }
    Value(int8_t value, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), int8Value(value)
    {
    }
    Value(uint8_t value, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), uint8Value(value)
    {
    }
    Value(int16_t value, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), int16Value(value)
    {
    }
    Value(uint16_t value, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), uint16Value(value)
    {
    }
    Value(uint32_t value, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), uint32Value(value)
    {
    }
    Value(int64_t value, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), int64Value(value)
    {
    }
    Value(uint64_t value, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), uint64Value(value)
    {
    }
    Value(float value, Unit unit)
        : type(VALUE_TYPE_FLOAT), unit(unit), options(0), dstValueType(VALUE_TYPE_UNDEFINED), floatValue(value)
    {
    }
    Value(float value, Unit unit, uint16_t options)
        : type(VALUE_TYPE_FLOAT), unit(unit), options(options), dstValueType(VALUE_TYPE_UNDEFINED), floatValue(value)
    {
    }
    Value(float value, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), floatValue(value)
    {
    }
	Value(double value, ValueType type)
		: type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), doubleValue(value) {
	}
	Value(const char *value, ValueType type, uint16_t options)
        : type(type), unit(UNIT_UNKNOWN), options(options), dstValueType(VALUE_TYPE_UNDEFINED), strValue(value)
    {
    }
    Value(void *value, ValueType type)
        : type(type), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), pVoidValue(value)
    {
    }
    typedef float (*YtDataGetValueFunctionPointer)(uint32_t rowIndex, uint8_t columnIndex, float *max);
    Value(YtDataGetValueFunctionPointer ytDataGetValueFunctionPointer)
        : type(VALUE_TYPE_YT_DATA_GET_VALUE_FUNCTION_POINTER), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), pVoidValue((void *)ytDataGetValueFunctionPointer)
    {
    }
	Value(const Value& value)
		: type(VALUE_TYPE_UNDEFINED), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), uint64Value(0)
	{
		*this = value;
	}
#if EEZ_OPTION_GUI
    Value(AppContext *appContext)
        : type(VALUE_TYPE_POINTER), unit(UNIT_UNKNOWN), options(0), dstValueType(VALUE_TYPE_UNDEFINED), pVoidValue(appContext)
    {
    }
#endif
	~Value() {
        freeRef();
	}
    void freeRef() {
		if (options & VALUE_OPTIONS_REF) {
			if (--refValue->refCounter == 0) {
                ObjectAllocator<Ref>::deallocate(refValue);
			}
		}
#if defined(EEZ_DASHBOARD_API)
        if (type == VALUE_TYPE_JSON || type == VALUE_TYPE_STREAM) {
            flow::dashboardObjectValueDecRef(int32Value);;
        }
#endif
    }
    Value& operator = (const Value &value) {
        freeRef();
        if (value.type == VALUE_TYPE_STRING_ASSET) {
            type = VALUE_TYPE_STRING;
            unit = 0;
            options = 0;
            strValue = (const char *)((uint8_t *)&value.int32Value + value.int32Value);
        } else if (value.type == VALUE_TYPE_ARRAY_ASSET) {
            type = VALUE_TYPE_ARRAY;
            unit = 0;
            options = 0;
            arrayValue = (ArrayValue *)((uint8_t *)&value.int32Value + value.int32Value);
        } else {
            type = value.type;
            unit = value.unit;
            options = value.options;
            dstValueType = value.dstValueType;
            memcpy((void *)&int64Value, (const void *)&value.int64Value, sizeof(int64_t));
            if (options & VALUE_OPTIONS_REF) {
                refValue->refCounter++;
            }
#if defined(EEZ_DASHBOARD_API)
            if (type == VALUE_TYPE_JSON || type == VALUE_TYPE_STREAM) {
                flow::dashboardObjectValueIncRef(value.int32Value);;
            }
#endif
        }
        return *this;
    }
    bool operator==(const Value &other) const {
		return type == other.type && g_valueTypeCompareFunctions[type](*this, other);
	}
    bool operator!=(const Value &other) const {
        return !(*this == other);
    }
    ValueType getType() const {
        return (ValueType)type;
    }
    bool isIndirectValueType() const {
        return type == VALUE_TYPE_VALUE_PTR || type == VALUE_TYPE_NATIVE_VARIABLE || type == VALUE_TYPE_ARRAY_ELEMENT_VALUE || type == VALUE_TYPE_JSON_MEMBER_VALUE;
    }
    Value getValue() const;
    bool isUndefinedOrNull() {
        return type == VALUE_TYPE_UNDEFINED || type == VALUE_TYPE_NULL;
    }
	bool isInt32OrLess() const {
		return (type >= VALUE_TYPE_INT8 && type <= VALUE_TYPE_UINT32) || type == VALUE_TYPE_BOOLEAN;
	}
	bool isInt64() const {
		return type == VALUE_TYPE_INT64 || type == VALUE_TYPE_UINT64;
	}
	bool isInt32() const {
		return type == VALUE_TYPE_INT32 || type == VALUE_TYPE_UINT32;
	}
	bool isInt16() const {
		return type == VALUE_TYPE_INT16 || type == VALUE_TYPE_UINT16;
	}
	bool isInt8() const {
		return type == VALUE_TYPE_INT8 || type == VALUE_TYPE_UINT8;
	}
	bool isFloat() const {
        return type == VALUE_TYPE_FLOAT;
    }
	bool isDouble() const {
		return type == VALUE_TYPE_DOUBLE || type == VALUE_TYPE_DATE;
	}
	bool isBoolean() const {
		return type == VALUE_TYPE_BOOLEAN;
	}
	bool isString() const {
        return type == VALUE_TYPE_STRING || type == VALUE_TYPE_STRING_ASSET || type == VALUE_TYPE_STRING_REF;
    }
    bool isArray() const {
        return type == VALUE_TYPE_ARRAY || type == VALUE_TYPE_ARRAY_ASSET || type == VALUE_TYPE_ARRAY_REF;
    }
	bool isBlob() const {
        return type == VALUE_TYPE_BLOB_REF;
    }
	bool isJson() const {
        return type == VALUE_TYPE_JSON;
    }
    bool isError() const {
        return type == VALUE_TYPE_ERROR;
    }
    Unit getUnit() const {
        return (Unit)unit;
    }
	bool getBoolean() const {
		return int32Value;
	}
	int8_t getInt8() const {
		return int8Value;
	}
	uint8_t getUInt8() const {
        return uint8Value;
    }
	int16_t getInt16() const {
		return int16Value;
	}
	uint16_t getUInt16() const {
        return uint16Value;
    }
	int32_t getInt32() const {
		return int32Value;
	}
	uint32_t getUInt32() const {
        return uint32Value;
    }
	int64_t getInt64() const {
		return int64Value;
	}
	uint64_t getUInt64() const {
        return uint64Value;
    }
	float getFloat() const {
		return floatValue;
	}
	double getDouble() const {
		return doubleValue;
	}
	const char *getString() const;
    const ArrayValue *getArray() const;
    ArrayValue *getArray();
	int getInt() const {
		if (type == VALUE_TYPE_ENUM) {
			return enumValue.enumValue;
		}
		return int32Value;
	}
    const EnumValue &getEnum() const {
        return enumValue;
    }
    int16_t getScpiError() const {
        return int16Value;
    }
    uint8_t *getPUint8() const {
        return puint8Value;
    }
    float *getFloatList() const {
        return pFloatValue;
    }
    void *getVoidPointer() const {
        return pVoidValue;
    }
    YtDataGetValueFunctionPointer getYtDataGetValueFunctionPointer() const {
        return (YtDataGetValueFunctionPointer)pVoidValue;
    }
    uint8_t getFirstUInt8() const {
        return pairOfUint8Value.first;
    }
    uint8_t getSecondUInt8() const {
        return pairOfUint8Value.second;
    }
    uint16_t getFirstUInt16() const {
        return pairOfUint16Value.first;
    }
    uint16_t getSecondUInt16() const {
        return pairOfUint16Value.second;
    }
    int16_t getFirstInt16() const {
        return pairOfInt16Value.first;
    }
    int16_t getSecondInt16() const {
        return pairOfInt16Value.second;
    }
    BlobRef *getBlob() const {
        return (BlobRef *)refValue;
    }
    void toText(char *text, int count) const {
		*text = 0;
		g_valueTypeToTextFunctions[type](*this, text, count);
	}
	uint16_t getOptions() const {
        return options;
    }
    uint16_t getRangeFrom() {
        return pairOfUint16Value.first;
    }
    uint16_t getRangeTo() {
        return pairOfUint16Value.second;
    }
	double toDouble(int *err = nullptr) const;
	float toFloat(int *err = nullptr) const;
	int32_t toInt32(int *err = nullptr) const;
	int64_t toInt64(int *err = nullptr) const;
    bool toBool(int *err = nullptr) const;
	Value toString(uint32_t id) const;
	static Value makeStringRef(const char *str, int len, uint32_t id);
	static Value concatenateString(const Value &str1, const Value &str2);
    static Value makeArrayRef(int arraySize, int arrayType, uint32_t id);
    static Value makeArrayElementRef(Value arrayValue, int elementIndex, uint32_t id);
    static Value makeJsonMemberRef(Value jsonValue, Value propertyName, uint32_t id);
    static Value makeBlobRef(const uint8_t *blob, uint32_t len, uint32_t id);
    static Value makeBlobRef(const uint8_t *blob1, uint32_t len1, const uint8_t *blob2, uint32_t len2, uint32_t id);
    static Value makeError() { return Value(0, VALUE_TYPE_ERROR); }
    Value clone();
  public:
	uint8_t type;
	uint8_t unit;
	uint16_t options;
    uint32_t dstValueType;
    union {
		int8_t int8Value;
		uint8_t uint8Value;
		int16_t int16Value;
		uint16_t uint16Value;
		int32_t int32Value;
		uint32_t uint32Value;
		int64_t int64Value;
		uint64_t uint64Value;
		float floatValue;
		double doubleValue;
		const char *strValue;
		ArrayValue *arrayValue;
		Ref *refValue;
		uint8_t *puint8Value;
		float *pFloatValue;
		void *pVoidValue;
		Value *pValueValue;
		EnumValue enumValue;
		PairOfUint8Value pairOfUint8Value;
		PairOfUint16Value pairOfUint16Value;
		PairOfInt16Value pairOfInt16Value;
	};
};
struct StringRef : public Ref {
    ~StringRef() {
        if (str) {
            eez::free(str);
        }
    }
	char *str;
};
struct ArrayValue {
	uint32_t arraySize;
    uint32_t arrayType;
	Value values[1];
};
struct ArrayValueRef : public Ref {
    ~ArrayValueRef();
	ArrayValue arrayValue;
};
struct BlobRef : public Ref {
    ~BlobRef() {
        if (blob) {
            eez::free(blob);
        }
    }
	uint8_t *blob;
    uint32_t len;
};
struct ArrayElementValue : public Ref {
	Value arrayValue;
    int elementIndex;
};
struct JsonMemberValue : public Ref {
	Value jsonValue;
    Value propertyName;
};
#if EEZ_OPTION_GUI
namespace gui {
    struct WidgetCursor;
    extern gui::WidgetCursor g_widgetCursor;
    extern Value get(const gui::WidgetCursor &widgetCursor, int16_t id);
}
#else
Value getVar(int16_t id);
void setVar(int16_t id, const Value& value);
#endif
#if defined(EEZ_DASHBOARD_API)
namespace flow {
    extern Value operationJsonGet(int json, const char *property);
}
#endif
inline Value Value::getValue() const {
    if (type == VALUE_TYPE_VALUE_PTR) {
        return *pValueValue;
    }
    if (type == VALUE_TYPE_NATIVE_VARIABLE) {
#if EEZ_OPTION_GUI
        using namespace gui;
        return get(g_widgetCursor, int32Value);
#else
        return getVar(int32Value);
#endif
    }
    if (type == VALUE_TYPE_ARRAY_ELEMENT_VALUE) {
        auto arrayElementValue = (ArrayElementValue *)refValue;
        if (arrayElementValue->arrayValue.isBlob()) {
            auto blobRef = arrayElementValue->arrayValue.getBlob();
            if (arrayElementValue->elementIndex < 0 || arrayElementValue->elementIndex >= (int)blobRef->len) {
                return Value();
            }
            return Value((uint32_t)blobRef->blob[arrayElementValue->elementIndex], VALUE_TYPE_UINT32);
        } else {
            auto array = arrayElementValue->arrayValue.getArray();
            if (arrayElementValue->elementIndex < 0 || arrayElementValue->elementIndex >= (int)array->arraySize) {
                return Value();
            }
            return array->values[arrayElementValue->elementIndex];
        }
    }
#if defined(EEZ_DASHBOARD_API)
    else if (type == VALUE_TYPE_JSON_MEMBER_VALUE) {
        auto jsonMemberValue = (JsonMemberValue *)refValue;
        return flow::operationJsonGet(jsonMemberValue->jsonValue.getInt(), jsonMemberValue->propertyName.getString());
    }
#endif
    return *this;
}
bool assignValue(Value &dstValue, const Value &srcValue, uint32_t dstValueType = VALUE_TYPE_UNDEFINED);
uint16_t getPageIndexFromValue(const Value &value);
uint16_t getNumPagesFromValue(const Value &value);
Value MakeRangeValue(uint16_t from, uint16_t to);
Value MakeEnumDefinitionValue(uint8_t enumValue, uint8_t enumDefinition);
inline Value IntegerValue(int32_t value) { return Value((int)value, VALUE_TYPE_INT32); }
inline Value FloatValue(float value) { return Value(value, VALUE_TYPE_FLOAT); }
inline Value DoubleValue(double value) { return Value(value, VALUE_TYPE_DOUBLE); }
inline Value BooleanValue(bool value) { return Value(value, VALUE_TYPE_BOOLEAN); }
inline Value StringValue(const char *value) { return Value::makeStringRef(value, -1, 0); }
template<class T, uint32_t ARRAY_TYPE>
struct ArrayOf {
    Value value;
    ArrayOf(size_t size) {
        value = Value::makeArrayRef((uint32_t)size, ARRAY_TYPE, 0);
    }
    ArrayOf(Value value) : value(value) {}
    operator Value() const { return value; }
    operator bool() const { return value.isArray(); }
    size_t size() {
        return (size_t)value.getArray()->arraySize;
    }
    T at(int position) {
        return value.getArray()->values[position];
    }
    void at(int position, const T &point) {
        value.getArray()->values[position] = point.value;
    }
};
struct ArrayOfInteger {
    Value value;
    ArrayOfInteger(size_t size) {
        value = Value::makeArrayRef((uint32_t)size, flow::defs_v3::ARRAY_TYPE_INTEGER, 0);
    }
    ArrayOfInteger(Value value) : value(value) {}
    operator Value() const { return value; }
    operator bool() const { return value.isArray(); }
    size_t size() {
        return (size_t)value.getArray()->arraySize;
    }
    int at(int position) {
        return value.getArray()->values[position].getInt();
    }
    void at(int position, int intValue) {
        value.getArray()->values[position] = Value(intValue, VALUE_TYPE_INT32);
    }
};
struct ArrayOfFloat {
    Value value;
    ArrayOfFloat(size_t size) {
        value = Value::makeArrayRef((uint32_t)size, flow::defs_v3::ARRAY_TYPE_INTEGER, 0);
    }
    ArrayOfFloat(Value value) : value(value) {}
    operator Value() const { return value; }
    operator bool() const { return value.isArray(); }
    size_t size() {
        return (size_t)value.getArray()->arraySize;
    }
    float at(int position) {
        return value.getArray()->values[position].getFloat();
    }
    void at(int position, float floatValue) {
        value.getArray()->values[position] = Value(floatValue, VALUE_TYPE_FLOAT);
    }
};
struct ArrayOfDouble {
    Value value;
    ArrayOfDouble(size_t size) {
        value = Value::makeArrayRef((uint32_t)size, flow::defs_v3::ARRAY_TYPE_INTEGER, 0);
    }
    ArrayOfDouble(Value value) : value(value) {}
    operator Value() const { return value; }
    operator bool() const { return value.isArray(); }
    size_t size() {
        return (size_t)value.getArray()->arraySize;
    }
    double at(int position) {
        return value.getArray()->values[position].getDouble();
    }
    void at(int position, double doubleValue) {
        value.getArray()->values[position] = Value(doubleValue, VALUE_TYPE_DOUBLE);
    }
};
struct ArrayOfBoolean {
    Value value;
    ArrayOfBoolean(size_t size) {
        value = Value::makeArrayRef((uint32_t)size, flow::defs_v3::ARRAY_TYPE_INTEGER, 0);
    }
    ArrayOfBoolean(Value value) : value(value) {}
    operator Value() const { return value; }
    operator bool() const { return value.isArray(); }
    size_t size() {
        return (size_t)value.getArray()->arraySize;
    }
    bool at(int position) {
        return value.getArray()->values[position].getDouble();
    }
    void at(int position, bool boolValue) {
        value.getArray()->values[position] = Value(boolValue, VALUE_TYPE_BOOLEAN);
    }
};
struct ArrayOfString {
    Value value;
    ArrayOfString(size_t size) {
        value = Value::makeArrayRef((uint32_t)size, flow::defs_v3::ARRAY_TYPE_INTEGER, 0);
    }
    ArrayOfString(Value value) : value(value) {}
    operator Value() const { return value; }
    operator bool() const { return value.isArray(); }
    size_t size() {
        return (size_t)value.getArray()->arraySize;
    }
    const char *at(int position) {
        return value.getArray()->values[position].getString();
    }
    void at(int position, const char *stringValue) {
        value.getArray()->values[position] = Value(stringValue, VALUE_TYPE_STRING);
    }
};
}
// -----------------------------------------------------------------------------
// core/action.h
// -----------------------------------------------------------------------------
#if defined(EEZ_FOR_LVGL)
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#endif
namespace eez {
#if EEZ_OPTION_GUI
namespace gui {
#endif
#if !defined(EEZ_FOR_LVGL)
typedef void (*ActionExecFunc)();
extern ActionExecFunc g_actionExecFunctions[];
#endif
void executeActionFunction(int actionId);
#if EEZ_OPTION_GUI
}
#endif
}
// -----------------------------------------------------------------------------
// core/assets.h
// -----------------------------------------------------------------------------
#include <stdint.h>
namespace eez {
static const uint32_t HEADER_TAG = 0x5A45457E;
static const uint32_t HEADER_TAG_COMPRESSED = 0x7A65657E;
static const uint8_t PROJECT_VERSION_V2 = 2;
static const uint8_t PROJECT_VERSION_V3 = 3;
static const uint8_t ASSETS_TYPE_FIRMWARE = 1;
static const uint8_t ASSETS_TYPE_FIRMWARE_MODULE = 2;
static const uint8_t ASSETS_TYPE_RESOURCE = 3;
static const uint8_t ASSETS_TYPE_APPLET = 4;
static const uint8_t ASSETS_TYPE_DASHBOARD = 5;
struct Header {
	uint32_t tag;
	uint8_t projectMajorVersion;
	uint8_t projectMinorVersion;
	uint8_t assetsType;
    uint8_t reserved;
	uint32_t decompressedSize;
};
extern bool g_isMainAssetsLoaded;
struct Assets;
extern Assets *g_mainAssets;
extern bool g_mainAssetsUncompressed;
extern Assets *g_externalAssets;
template<typename T>
struct AssetsPtr {
    AssetsPtr() : offset(0) {}
    AssetsPtr(const AssetsPtr<T> &rhs) = delete;
	void operator=(T* ptr) {
		if (ptr != nullptr) {
            offset = (uint8_t *)ptr - (uint8_t *)&offset;
		} else {
			offset = 0;
		}
    }
    operator T*() { return ptr(); }
    operator const T*() const { return ptr(); }
          T* operator->()       { return ptr(); }
    const T* operator->() const { return ptr(); }
private:
    int32_t offset;
    T* ptr() {
        return offset ? (T *)((uint8_t *)&offset + offset) : nullptr;
    }
	const T* ptr() const {
		return offset ? (const T *)((uint8_t *)&offset + offset) : nullptr;
	}
};
template<typename T>
struct ListOfAssetsPtr {
	uint32_t count = 0;
    T*       operator[](uint32_t i)       { return item(i); }
    const T* operator[](uint32_t i) const { return item(i); }
private:
    AssetsPtr<AssetsPtr<T>> items;
    T* item(int i) {
        return static_cast<T *>(static_cast<AssetsPtr<T> *>(items)[i]);
    }
    const T* item(int i) const {
        return static_cast<const T *>(static_cast<const AssetsPtr<T> *>(items)[i]);
    }
};
template<typename T>
struct ListOfFundamentalType {
	uint32_t count;
    AssetsPtr<T> items;
    T&       operator[](uint32_t i)       { return ptr()[i]; }
    const T& operator[](uint32_t i) const { return ptr()[i]; }
private:
    T *ptr() {
        return static_cast<T *>(items);
    }
};
struct Settings {
    uint16_t displayWidth;
    uint16_t displayHeight;
};
#if EEZ_OPTION_GUI
#define WIDGET_FLAG_PIN_TO_LEFT (1 << 0)
#define WIDGET_FLAG_PIN_TO_RIGHT (1 << 1)
#define WIDGET_FLAG_PIN_TO_TOP (1 << 2)
#define WIDGET_FLAG_PIN_TO_BOTTOM (1 << 3)
#define WIDGET_FLAG_FIX_WIDTH (1 << 4)
#define WIDGET_FLAG_FIX_HEIGHT (1 << 5)
#define WIDGET_TIMELINE_PROPERTY_X (1 << 0)
#define WIDGET_TIMELINE_PROPERTY_Y (1 << 1)
#define WIDGET_TIMELINE_PROPERTY_WIDTH (1 << 2)
#define WIDGET_TIMELINE_PROPERTY_HEIGHT (1 << 3)
#define WIDGET_TIMELINE_PROPERTY_OPACITY (1 << 4)
#define WIDGET_TIMELINE_PROPERTY_CP1 (1 << 5)
#define WIDGET_TIMELINE_PROPERTY_CP2 (1 << 6)
#define EASING_FUNC_LINEAR 0
#define EASING_FUNC_IN_QUAD 1
#define EASING_FUNC_OUT_QUAD 2
#define EASING_FUNC_IN_OUT_QUAD 3
#define EASING_FUNC_IN_CUBIC 4
#define EASING_FUNC_OUT_CUBIC 5
#define EASING_FUNC_IN_OUT_CUBIC 6
#define EASING_FUNC_IN__QUART 7
#define EASING_FUNC_OUT_QUART 8
#define EASING_FUNC_IN_OUT_QUART 9
#define EASING_FUNC_IN_QUINT 10
#define EASING_FUNC_OUT_QUINT 11
#define EASING_FUNC_IN_OUT_QUINT 12
#define EASING_FUNC_IN_SINE 13
#define EASING_FUNC_OUT_SINE 14
#define EASING_FUNC_IN_OUT_SINE 15
#define EASING_FUNC_IN_EXPO 16
#define EASING_FUNC_OUT_EXPO 17
#define EASING_FUNC_IN_OUT_EXPO 18
#define EASING_FUNC_IN_CIRC 19
#define EASING_FUNC_OUT_CIRC 20
#define EASING_FUNC_IN_OUT_CIRC 21
#define EASING_FUNC_IN_BACK 22
#define EASING_FUNC_OUT_BACK 23
#define EASING_FUNC_IN_OUT_BACK 24
#define EASING_FUNC_IN_ELASTIC 25
#define EASING_FUNC_OUT_ELASTIC 26
#define EASING_FUNC_IN_OUT_ELASTIC 27
#define EASING_FUNC_IN_BOUNCE 28
#define EASING_FUNC_OUT_BOUNCE 29
#define EASING_FUNC_IN_OUT_BOUNCE 30
namespace gui {
struct TimelineKeyframe {
    float start;
    float end;
    uint32_t enabledProperties;
	int16_t x;
	int16_t y;
	int16_t width;
	int16_t height;
    float opacity;
    uint8_t xEasingFunc;
    uint8_t yEasingFunc;
    uint8_t widthEasingFunc;
    uint8_t heightEasingFunc;
    uint8_t opacityEasingFunc;
    uint8_t reserved1;
    uint16_t reserved2;
	int16_t cp1x;
	int16_t cp1y;
    int16_t cp2x;
    int16_t cp2y;
};
struct Widget {
	uint16_t type;
	int16_t data;
    int16_t visible;
	int16_t action;
	int16_t x;
	int16_t y;
	int16_t width;
	int16_t height;
	int16_t style;
    uint16_t flags;
    ListOfAssetsPtr<TimelineKeyframe> timeline;
};
#define SHADOW_FLAG (1 << 0)
#define CLOSE_PAGE_IF_TOUCHED_OUTSIDE_FLAG (1 << 1)
#define PAGE_IS_USED_AS_USER_WIDGET (1 << 2)
#define PAGE_CONTAINER (1 << 3)
#define PAGE_SCALE_TO_FIT (1 << 4)
struct PageAsset : public Widget {
	ListOfAssetsPtr<Widget> widgets;
	uint16_t flags;
	int16_t overlay;
};
#define STYLE_FLAGS_HORZ_ALIGN_MASK 0x7
#define STYLE_FLAGS_HORZ_ALIGN_LEFT 0
#define STYLE_FLAGS_HORZ_ALIGN_RIGHT 1
#define STYLE_FLAGS_HORZ_ALIGN_CENTER 2
#define STYLE_FLAGS_VERT_ALIGN_MASK (0x7 << 3)
#define STYLE_FLAGS_VERT_ALIGN_TOP (0 << 3)
#define STYLE_FLAGS_VERT_ALIGN_BOTTOM (1 << 3)
#define STYLE_FLAGS_VERT_ALIGN_CENTER (2 << 3)
#define STYLE_FLAGS_BLINK (1 << 6)
struct Style {
    uint16_t flags;
    uint16_t backgroundColor;
    uint16_t color;
    uint16_t activeBackgroundColor;
    uint16_t activeColor;
    uint16_t focusBackgroundColor;
    uint16_t focusColor;
    uint8_t borderSizeTop;
    uint8_t borderSizeRight;
    uint8_t borderSizeBottom;
    uint8_t borderSizeLeft;
    uint16_t borderColor;
    uint8_t borderRadiusTLX;
	uint8_t borderRadiusTLY;
    uint8_t borderRadiusTRX;
	uint8_t borderRadiusTRY;
    uint8_t borderRadiusBLX;
	uint8_t borderRadiusBLY;
    uint8_t borderRadiusBRX;
	uint8_t borderRadiusBRY;
    uint8_t font;
    uint8_t opacity;
    uint8_t paddingTop;
    uint8_t paddingRight;
    uint8_t paddingBottom;
    uint8_t paddingLeft;
	int16_t backgroundImage;
};
struct GlyphData {
	int8_t dx;
	uint8_t width;
	uint8_t height;
	int8_t x;
	int8_t y;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
	uint8_t pixels[1];
};
struct GlyphsGroup {
    uint32_t encoding;
    uint32_t glyphIndex;
    uint32_t length;
};
struct FontData {
	uint8_t ascent;
	uint8_t descent;
    uint8_t reserved1;
    uint8_t reserved2;
	uint32_t encodingStart;
	uint32_t encodingEnd;
    ListOfAssetsPtr<GlyphsGroup> groups;
    ListOfAssetsPtr<GlyphData> glyphs;
};
struct Bitmap {
    int16_t w;
    int16_t h;
    int16_t bpp;
    int16_t reserved;
    AssetsPtr<const char> name;
    const uint8_t pixels[1];
};
}
#endif
struct Theme {
	AssetsPtr<const char> name;
	ListOfFundamentalType<uint16_t> colors;
};
struct Colors {
	ListOfAssetsPtr<Theme> themes;
	ListOfFundamentalType<uint16_t> colors;
};
static const uint16_t EXPR_EVAL_INSTRUCTION_TYPE_MASK  = 0x0007 << 13;
static const uint16_t EXPR_EVAL_INSTRUCTION_PARAM_MASK = 0xFFFF >> 3;
static const uint16_t EXPR_EVAL_INSTRUCTION_TYPE_PUSH_CONSTANT           = (0 << 13);
static const uint16_t EXPR_EVAL_INSTRUCTION_TYPE_PUSH_INPUT              = (1 << 13);
static const uint16_t EXPR_EVAL_INSTRUCTION_TYPE_PUSH_LOCAL_VAR          = (2 << 13);
static const uint16_t EXPR_EVAL_INSTRUCTION_TYPE_PUSH_GLOBAL_VAR         = (3 << 13);
static const uint16_t EXPR_EVAL_INSTRUCTION_TYPE_PUSH_OUTPUT             = (4 << 13);
static const uint16_t EXPR_EVAL_INSTRUCTION_ARRAY_ELEMENT                = (5 << 13);
static const uint16_t EXPR_EVAL_INSTRUCTION_TYPE_OPERATION               = (6 << 13);
static const uint16_t EXPR_EVAL_INSTRUCTION_TYPE_END                     = (7 << 13);
static const uint16_t EXPR_EVAL_INSTRUCTION_TYPE_END_WITH_DST_VALUE_TYPE = (7 << 13) | (1 << 12);
struct Property {
	uint8_t evalInstructions[1];
};
struct Connection {
	uint16_t targetComponentIndex;
	uint16_t targetInputIndex;
};
struct ComponentOutput {
	ListOfAssetsPtr<Connection> connections;
	uint32_t isSeqOut;
};
static const uint16_t BREAKPOINT_ENABLED = 1;
static const uint16_t BREAKPOINT_DISABLED = 2;
struct Component {
    uint16_t type;
    uint16_t breakpoint;
	ListOfFundamentalType<uint16_t> inputs;
	ListOfAssetsPtr<Property> properties;
	ListOfAssetsPtr<ComponentOutput> outputs;
	int16_t errorCatchOutput;
	uint16_t reserved;
};
struct WidgetDataItem {
	int16_t componentIndex;
	int16_t propertyValueIndex;
};
struct WidgetActionItem {
	int16_t componentIndex;
	int16_t componentOutputIndex;
};
#define COMPONENT_INPUT_FLAG_IS_SEQ_INPUT   (1 << 0)
#define COMPONENT_INPUT_FLAG_IS_OPTIONAL (1 << 1)
typedef uint8_t ComponentInput;
struct Flow {
	ListOfAssetsPtr<Component> components;
	ListOfAssetsPtr<Value> localVariables;
	ListOfFundamentalType<ComponentInput> componentInputs;
	ListOfAssetsPtr<WidgetDataItem> widgetDataItems;
	ListOfAssetsPtr<WidgetActionItem> widgetActions;
};
struct FlowDefinition {
	ListOfAssetsPtr<Flow> flows;
	ListOfAssetsPtr<Value> constants;
	ListOfAssetsPtr<Value> globalVariables;
};
struct Language {
    AssetsPtr<const char> languageID;
    ListOfAssetsPtr<const char> translations;
};
struct Assets {
    uint8_t projectMajorVersion;
	uint8_t projectMinorVersion;
    uint8_t assetsType;
    uint8_t external;
    uint32_t reserved;
    AssetsPtr<Settings> settings;
#if EEZ_OPTION_GUI
	ListOfAssetsPtr<gui::PageAsset> pages;
	ListOfAssetsPtr<gui::Style> styles;
	ListOfAssetsPtr<gui::FontData> fonts;
	ListOfAssetsPtr<gui::Bitmap> bitmaps;
#endif
	AssetsPtr<Colors> colorsDefinition;
	ListOfAssetsPtr<const char> actionNames;
	ListOfAssetsPtr<const char> variableNames;
	AssetsPtr<FlowDefinition> flowDefinition;
    ListOfAssetsPtr<Language> languages;
};
bool decompressAssetsData(const uint8_t *assetsData, uint32_t assetsDataSize, Assets *decompressedAssets, uint32_t maxDecompressedAssetsSize, int *err);
void loadMainAssets(const uint8_t *assets, uint32_t assetsSize);
bool loadExternalAssets(const char *filePath, int *err);
void unloadExternalAssets();
#if EEZ_OPTION_GUI
const gui::PageAsset *getPageAsset(int pageId);
const gui::PageAsset* getPageAsset(int pageId, gui::WidgetCursor& widgetCursor);
const gui::Style *getStyle(int styleID);
const gui::FontData *getFontData(int fontID);
const gui::Bitmap *getBitmap(int bitmapID);
const int getBitmapIdByName(const char *bitmapName);
#endif
int getThemesCount();
const char *getThemeName(int i);
const uint32_t getThemeColorsCount(int themeIndex);
const uint16_t *getThemeColors(int themeIndex);
const uint16_t *getColors();
int getExternalAssetsMainPageId();
#if EEZ_OPTION_GUI
const char *getActionName(const gui::WidgetCursor &widgetCursor, int16_t actionId);
int16_t getDataIdFromName(const gui::WidgetCursor &widgetCursor, const char *name);
#endif
}
// -----------------------------------------------------------------------------
// core/debug.h
// -----------------------------------------------------------------------------
#ifdef DEBUG
#include <stdlib.h>
#include <stdint.h>
namespace eez {
namespace debug {
void pushDebugTraceHook(const char *message, size_t messageLength);
void pushInfoTraceHook(const char *message, size_t messageLength);
void pushErrorTraceHook(const char *message, size_t messageLength);
enum TraceType {
    TRACE_TYPE_DEBUG,
    TRACE_TYPE_INFO,
    TRACE_TYPE_ERROR
};
void Trace(TraceType traceType, const char *format, ...);
}
}
#define InfoTrace(...) ::eez::debug::Trace(::eez::debug::TRACE_TYPE_INFO, __VA_ARGS__)
#define ErrorTrace(...) ::eez::debug::Trace(::eez::debug::TRACE_TYPE_ERROR, __VA_ARGS__)
#define DebugTrace(...) ::eez::debug::Trace(::eez::debug::TRACE_TYPE_DEBUG, __VA_ARGS__)
#else
#define InfoTrace(...) (void)0
#define ErrorTrace(...) (void)0
#define DebugTrace(...) (void)0
#endif
// -----------------------------------------------------------------------------
// core/os.h
// -----------------------------------------------------------------------------
#include <stdint.h>
#if defined(EEZ_FOR_LVGL)
uint32_t osKernelGetTickCount(void);
#else
#include "cmsis_os2.h"
#if defined(EEZ_PLATFORM_STM32)
#include "FreeRTOS.h"
#include "task.h"
#endif
#define EEZ_THREAD_DECLARE(NAME, PRIORITY, STACK_SIZE) \
    osThreadId_t g_##NAME##TaskHandle; \
    const osThreadAttr_t g_##NAME##TaskAttributes = { \
        #NAME, \
		0, \
		0, \
		0, \
		0, \
        STACK_SIZE, \
        osPriority##PRIORITY, \
		0, \
		0, \
    }
#define EEZ_THREAD_CREATE(NAME, THREAD_FUNC) g_##NAME##TaskHandle = osThreadNew(THREAD_FUNC, nullptr, &g_##NAME##TaskAttributes);
#define EEZ_THREAD_TERMINATE(NAME) osThreadTerminate(g_##NAME##TaskHandle)
#define EEZ_MESSAGE_QUEUE_DECLARE(NAME, OBJECT_DEF) \
    struct NAME##MessageQueueObject OBJECT_DEF; \
    osMessageQueueId_t g_##NAME##MessageQueueId
#define EEZ_MESSAGE_QUEUE_CREATE(NAME, QUEUE_SIZE) g_##NAME##MessageQueueId = osMessageQueueNew(QUEUE_SIZE, sizeof(NAME##MessageQueueObject), nullptr)
#define EEZ_MESSAGE_QUEUE_GET(NAME, OBJ, TIMEOUT) (osMessageQueueGet(g_##NAME##MessageQueueId, &OBJ, nullptr, TIMEOUT) == osOK)
#define EEZ_MESSAGE_QUEUE_PUT(NAME, OBJ, TIMEOUT) osMessageQueuePut(g_##NAME##MessageQueueId, &OBJ, 0, TIMEOUT)
#define EEZ_MUTEX_DECLARE(NAME) \
    osMutexId_t g_##NAME##mutexId;\
    const osMutexAttr_t g_##NAME##mutexAttr = { \
        #NAME, \
        osMutexRecursive | osMutexPrioInherit, \
        NULL, \
        0U \
    }
#define EEZ_MUTEX_CREATE(NAME) g_##NAME##mutexId = osMutexNew(&g_##NAME##mutexAttr)
#define EEZ_MUTEX_WAIT(NAME, TIMEOUT) osMutexAcquire(g_##NAME##mutexId, TIMEOUT) == osOK
#define EEZ_MUTEX_RELEASE(NAME) osMutexRelease(g_##NAME##mutexId)
#endif
#if defined(__EMSCRIPTEN__)
#ifndef EM_PORT_API
#	if defined(__EMSCRIPTEN__)
#		include <emscripten.h>
#		if defined(__cplusplus)
#			define EM_PORT_API(rettype) extern "C" rettype EMSCRIPTEN_KEEPALIVE
#		else
#			define EM_PORT_API(rettype) rettype EMSCRIPTEN_KEEPALIVE
#		endif
#	else
#		if defined(__cplusplus)
#			define EM_PORT_API(rettype) extern "C" rettype
#		else
#			define EM_PORT_API(rettype) rettype
#		endif
#	endif
#endif
#else
#    define EM_PORT_API(rettype) rettype
#endif
namespace eez {
enum TestResult {
	TEST_NONE,
	TEST_FAILED,
	TEST_OK,
	TEST_CONNECTING,
	TEST_SKIPPED,
	TEST_WARNING
};
uint32_t millis();
extern bool g_shutdown;
void shutdown();
}
// -----------------------------------------------------------------------------
// core/memory.h
// -----------------------------------------------------------------------------
#include <stdint.h>
namespace eez {
#ifdef CONF_MEMORY_BEGIN
    static uint8_t * const MEMORY_BEGIN = (uint8_t *)CONF_MEMORY_BEGIN;
#else
    #if defined(EEZ_FOR_LVGL)
        static uint8_t * const MEMORY_BEGIN = 0;
    #else
        #if defined(EEZ_PLATFORM_STM32)
            static uint8_t * const MEMORY_BEGIN = (uint8_t *)0xc0000000u;
        #endif
        #if defined(EEZ_PLATFORM_SIMULATOR) || defined(__EMSCRIPTEN__)
            extern uint8_t g_memory[];
            static uint8_t * const MEMORY_BEGIN = g_memory;
        #endif
    #endif
#endif
#ifdef CONF_MEMORY_SIZE
    static const uint32_t MEMORY_SIZE = CONF_MEMORY_SIZE;
#else
    #if defined(EEZ_FOR_LVGL)
    #else
        #if defined(EEZ_PLATFORM_STM32)
            #if CONF_OPTION_FPGA
                static const uint32_t MEMORY_SIZE = 32 * 1024 * 1024;
            #elif defined(EEZ_PLATFORM_STM32F469I_DISCO)
                static const uint32_t MEMORY_SIZE = 16 * 1024 * 1024;
            #else
                static const uint32_t MEMORY_SIZE = 8 * 1024 * 1024;
            #endif
        #endif
        #if defined(EEZ_PLATFORM_SIMULATOR) || defined(__EMSCRIPTEN__)
            static const uint32_t MEMORY_SIZE = 64 * 1024 * 1024;
        #endif
    #endif
#endif
extern uint8_t *ALLOC_BUFFER;
extern uint32_t ALLOC_BUFFER_SIZE;
#if !defined(EEZ_FOR_LVGL)
    extern uint8_t *DECOMPRESSED_ASSETS_START_ADDRESS;
    #if defined(CONF_MAX_DECOMPRESSED_ASSETS_SIZE)
        static const uint32_t MAX_DECOMPRESSED_ASSETS_SIZE = CONF_MAX_DECOMPRESSED_ASSETS_SIZE;
    #else
        #if defined(EEZ_PLATFORM_STM32)
            static const uint32_t MAX_DECOMPRESSED_ASSETS_SIZE = 2 * 1024 * 1024;
        #endif
        #if defined(EEZ_PLATFORM_SIMULATOR) || defined(__EMSCRIPTEN__)
            static const uint32_t MAX_DECOMPRESSED_ASSETS_SIZE = 8 * 1024 * 1024;
        #endif
    #endif
#endif
#if !defined(EEZ_FOR_LVGL)
    extern uint8_t *FLOW_TO_DEBUGGER_MESSAGE_BUFFER;
    #if defined(EEZ_FOR_LVGL)
        static const uint32_t FLOW_TO_DEBUGGER_MESSAGE_BUFFER_SIZE = 32 * 1024;
    #else
        #if defined(EEZ_PLATFORM_STM32)
            static const uint32_t FLOW_TO_DEBUGGER_MESSAGE_BUFFER_SIZE = 32 * 1024;
        #endif
        #if defined(EEZ_PLATFORM_SIMULATOR) || defined(__EMSCRIPTEN__)
            static const uint32_t FLOW_TO_DEBUGGER_MESSAGE_BUFFER_SIZE = 1024 * 1024;
        #endif
    #endif
#endif
#if EEZ_OPTION_GUI
    extern uint8_t *VRAM_BUFFER1_START_ADDRESS;
    extern uint8_t *VRAM_BUFFER2_START_ADDRESS;
    #if EEZ_OPTION_GUI_ANIMATIONS
        extern uint8_t *VRAM_ANIMATION_BUFFER1_START_ADDRESS;
        extern uint8_t *VRAM_ANIMATION_BUFFER2_START_ADDRESS;
    #endif
    #ifndef NUM_AUX_BUFFERS
        #define NUM_AUX_BUFFERS 6
    #endif
    extern uint8_t *VRAM_AUX_BUFFER_START_ADDRESSES[NUM_AUX_BUFFERS];
    extern uint8_t* SCREENSHOOT_BUFFER_START_ADDRESS;
    extern uint8_t* GUI_STATE_BUFFER;
#endif
void initMemory();
void initAssetsMemory();
void initOtherMemory();
uint8_t *allocBuffer(uint32_t size);
}
// -----------------------------------------------------------------------------
// core/utf8.h
// -----------------------------------------------------------------------------
#ifndef UTF8_SUPPORT
#define UTF8_SUPPORT 1
#endif
#if UTF8_SUPPORT
#else
#include <string.h>
typedef char utf8_int8_t;
typedef int32_t utf8_int32_t;
inline const utf8_int8_t* utf8codepoint(const utf8_int8_t *str, utf8_int32_t *out_codepoint) {
    *out_codepoint = *((uint8_t *)str);
    return str + 1;
}
inline utf8_int8_t *utf8catcodepoint(utf8_int8_t *str, utf8_int32_t chr, size_t n) {
    if (n < 1) return nullptr;
    str[0] = (char)chr;
    return str + 1;
}
#define utf8len strlen
#define utf8cmp strcmp
#ifdef _MSC_VER
#define utf8casecmp _stricmp
#else
#define utf8casecmp strcasecmp
#endif
#endif
// -----------------------------------------------------------------------------
// core/util.h
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <stdlib.h>
#define clear_bit(reg, bitmask) *reg &= ~bitmask
#define set_bit(reg, bitmask) *reg |= bitmask
#define util_swap(type, i, j)                                                                      \
    {                                                                                              \
        type t = i;                                                                                \
        i = j;                                                                                     \
        j = t;                                                                                     \
    }
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#define PATH_SEPARATOR "/"
namespace eez {
float remap(float x, float x1, float y1, float x2, float y2);
float remapQuad(float x, float x1, float y1, float x2, float y2);
float remapOutQuad(float x, float x1, float y1, float x2, float y2);
float remapInOutQuad(float x, float x1, float y1, float x2, float y2);
float remapCubic(float x, float x1, float y1, float x2, float y2);
float remapOutCubic(float x, float x1, float y1, float x2, float y2);
float remapExp(float x, float x1, float y1, float x2, float y2);
float remapOutExp(float x, float x1, float y1, float x2, float y2);
float clamp(float x, float min, float max);
void stringCopy(char *dst, size_t maxStrLength, const char *src);
void stringCopyLength(char *dst, size_t maxStrLength, const char *src, size_t length);
void stringAppendString(char *str, size_t maxStrLength, const char *value);
void stringAppendStringLength(char *str, size_t maxStrLength, const char *value, size_t length);
void stringAppendInt(char *str, size_t maxStrLength, int value);
void stringAppendUInt32(char *str, size_t maxStrLength, uint32_t value);
void stringAppendInt64(char *str, size_t maxStrLength, int64_t value);
void stringAppendUInt64(char *str, size_t maxStrLength, uint64_t value);
void stringAppendFloat(char *str, size_t maxStrLength, float value);
void stringAppendFloat(char *str, size_t maxStrLength, float value, int numDecimalPlaces);
void stringAppendDouble(char *str, size_t maxStrLength, double value);
void stringAppendDouble(char *str, size_t maxStrLength, double value, int numDecimalPlaces);
void stringAppendVoltage(char *str, size_t maxStrLength, float value);
void stringAppendCurrent(char *str, size_t maxStrLength, float value);
void stringAppendPower(char *str, size_t maxStrLength, float value);
void stringAppendDuration(char *str, size_t maxStrLength, float value);
void stringAppendLoad(char *str, size_t maxStrLength, float value);
uint32_t crc32(const uint8_t *message, size_t size);
uint8_t toBCD(uint8_t bin);
uint8_t fromBCD(uint8_t bcd);
float roundPrec(float a, float prec);
float floorPrec(float a, float prec);
float ceilPrec(float a, float prec);
bool isNaN(float x);
bool isNaN(double x);
bool isDigit(char ch);
bool isHexDigit(char ch);
bool isUperCaseLetter(char ch);
char toHexDigit(int num);
int fromHexDigit(char ch);
bool pointInsideRect(int xPoint, int yPoint, int xRect, int yRect, int wRect, int hRect);
void getParentDir(const char *path, char *parentDirPath);
bool parseMacAddress(const char *macAddressStr, size_t macAddressStrLength, uint8_t *macAddress);
int getIpAddressPartA(uint32_t ipAddress);
void setIpAddressPartA(uint32_t *ipAddress, uint8_t value);
int getIpAddressPartB(uint32_t ipAddress);
void setIpAddressPartB(uint32_t *ipAddress, uint8_t value);
int getIpAddressPartC(uint32_t ipAddress);
void setIpAddressPartC(uint32_t *ipAddress, uint8_t value);
int getIpAddressPartD(uint32_t ipAddress);
void setIpAddressPartD(uint32_t *ipAddress, uint8_t value);
void ipAddressToArray(uint32_t ipAddress, uint8_t *ipAddressArray);
uint32_t arrayToIpAddress(uint8_t *ipAddressArray);
uint32_t getIpAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
bool parseIpAddress(const char *ipAddressStr, size_t ipAddressStrLength, uint32_t &ipAddress);
void ipAddressToString(uint32_t ipAddress, char *ipAddressStr, size_t maxIpAddressStrLength);
void macAddressToString(const uint8_t *macAddress, char *macAddressStr);
void formatTimeZone(int16_t timeZone, char *text, int count);
bool parseTimeZone(const char *timeZoneStr, size_t timeZoneLength, int16_t &timeZone);
void replaceCharacter(char *str, char ch, char repl);
int strcicmp(char const *a, char const *b);
int strncicmp(char const *a, char const *b, int n);
bool isStringEmpty(char const *a);
bool startsWith(const char *str, const char *prefix);
bool startsWithNoCase(const char *str, const char *prefix);
bool endsWith(const char *str, const char *suffix);
bool endsWithNoCase(const char *str, const char *suffix);
void formatBytes(uint64_t bytes, char *text, int count);
void getFileName(const char *path, char *fileName, unsigned fileNameSize);
void getBaseFileName(const char *path, char *baseName, unsigned baseNameSize);
typedef float (*EasingFuncType)(float x);
extern EasingFuncType g_easingFuncs[];
class Interval {
public:
	bool test(uint32_t interval) {
		auto time = millis();
		if (lastTime == 0) {
			lastTime = time == 0 ? 1 : time;
			return true;
		}
		if (time >= lastTime + interval) {
			lastTime += ((uint32_t)(time - lastTime) / interval) * interval;
			return true;
		}
		return false;
	}
private:
	uint32_t lastTime = 0;
};
template <typename T, typename Total, uint64_t N>
class MovingAverage {
public:
    void operator()(T sample) {
        if (m_numSamples < N) {
            m_samples[m_numSamples++] = sample;
            m_total += sample;
        } else {
            T& oldest = m_samples[m_numSamples++ % N];
			m_total += sample - oldest;
            oldest = sample;
        }
    }
    operator T() const {
		if (m_numSamples < N) {
			return m_total / m_numSamples;
		} else {
			return m_total / N;
		}
    }
    void reset() {
        m_numSamples = 0;
        m_total = 0;
    }
private:
    T m_samples[N];
    uint64_t m_numSamples{0};
    Total m_total{0};
};
}
#ifdef EEZ_PLATFORM_SIMULATOR_WIN32
char *strnstr(const char *s1, const char *s2, size_t n);
#endif
// -----------------------------------------------------------------------------
// flow/private.h
// -----------------------------------------------------------------------------
#if EEZ_OPTION_GUI
using namespace eez::gui;
#endif
namespace eez {
namespace flow {
struct GlobalVariables {
    uint32_t count;
    Value values[1];
};
extern struct GlobalVariables *g_globalVariables;
void initGlobalVariables(Assets *assets);
static const int UNDEFINED_VALUE_INDEX = 0;
static const int NULL_VALUE_INDEX = 1;
#define TRACK_REF_COUNTER_FOR_COMPONENT_STATE(component) \
    !( \
        component->type == defs_v3::COMPONENT_TYPE_INPUT_ACTION || \
        component->type == defs_v3::COMPONENT_TYPE_LOOP_ACTION || \
        component->type == defs_v3::COMPONENT_TYPE_COUNTER_ACTION || \
        component->type == defs_v3::COMPONENT_TYPE_WATCH_VARIABLE_ACTION \
    )
struct ComponenentExecutionState {
    uint32_t lastExecutedTime;
    ComponenentExecutionState() : lastExecutedTime(millis()) {}
	virtual ~ComponenentExecutionState() {}
};
struct CatchErrorComponenentExecutionState : public ComponenentExecutionState {
	Value message;
};
struct FlowState {
	uint32_t flowStateIndex;
	Assets *assets;
	FlowDefinition *flowDefinition;
	Flow *flow;
	uint16_t flowIndex;
	bool isAction;
	bool error;
    uint32_t refCounter;
    FlowState *parentFlowState;
	Component *parentComponent;
	int parentComponentIndex;
	Value *values;
	ComponenentExecutionState **componenentExecutionStates;
    bool *componenentAsyncStates;
    unsigned executingComponentIndex;
    float timelinePosition;
#if defined(EEZ_FOR_LVGL)
    int32_t lvglWidgetStartIndex;
#endif
    Value eventValue;
    FlowState *firstChild;
    FlowState *lastChild;
    FlowState *previousSibling;
    FlowState *nextSibling;
};
extern int g_selectedLanguage;
extern FlowState *g_firstFlowState;
extern FlowState *g_lastFlowState;
FlowState *initActionFlowState(int flowIndex, FlowState *parentFlowState, int parentComponentIndex);
FlowState *initPageFlowState(Assets *assets, int flowIndex, FlowState *parentFlowState, int parentComponentIndex);
void incRefCounterForFlowState(FlowState *flowState);
void decRefCounterForFlowState(FlowState *flowState);
bool canFreeFlowState(FlowState *flowState);
void freeFlowState(FlowState *flowState);
void freeAllChildrenFlowStates(FlowState *flowState);
void deallocateComponentExecutionState(FlowState *flowState, unsigned componentIndex);
extern void onComponentExecutionStateChanged(FlowState *flowState, int componentIndex);
template<class T>
T *allocateComponentExecutionState(FlowState *flowState, unsigned componentIndex) {
    if (flowState->componenentExecutionStates[componentIndex]) {
        deallocateComponentExecutionState(flowState, componentIndex);
    }
    auto executionState = ObjectAllocator<T>::allocate(0x72dc3bf4);
    flowState->componenentExecutionStates[componentIndex] = executionState;
    auto component = flowState->flow->components[componentIndex];
    if (TRACK_REF_COUNTER_FOR_COMPONENT_STATE(component)) {
        incRefCounterForFlowState(flowState);
    }
    onComponentExecutionStateChanged(flowState, componentIndex);
    return executionState;
}
void resetSequenceInputs(FlowState *flowState);
void propagateValue(FlowState *flowState, unsigned componentIndex, unsigned outputIndex, const Value &value);
void propagateValue(FlowState *flowState, unsigned componentIndex, unsigned outputIndex);
void propagateValueThroughSeqout(FlowState *flowState, unsigned componentIndex);
#if EEZ_OPTION_GUI
void getValue(uint16_t dataId, DataOperationEnum operation, const WidgetCursor &widgetCursor, Value &value);
void setValue(uint16_t dataId, const WidgetCursor &widgetCursor, const Value& value);
#endif
void assignValue(FlowState *flowState, int componentIndex, Value &dstValue, const Value &srcValue);
void clearInputValue(FlowState *flowState, int inputIndex);
void startAsyncExecution(FlowState *flowState, int componentIndex);
void endAsyncExecution(FlowState *flowState, int componentIndex);
void executeCallAction(FlowState *flowState, unsigned componentIndex, int flowIndex);
enum FlowEvent {
    FLOW_EVENT_OPEN_PAGE,
    FLOW_EVENT_CLOSE_PAGE,
    FLOW_EVENT_KEYDOWN
};
void onEvent(FlowState *flowState, FlowEvent flowEvent, Value eventValue);
void throwError(FlowState *flowState, int componentIndex, const char *errorMessage);
void throwError(FlowState *flowState, int componentIndex, const char *errorMessage, const char *errorMessageDescription);
void enableThrowError(bool enable);
}
}
// -----------------------------------------------------------------------------
// flow/components.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
using defs_v3::ComponentTypes;
typedef void (*ExecuteComponentFunctionType)(FlowState *flowState, unsigned componentIndex);
void registerComponent(ComponentTypes componentType, ExecuteComponentFunctionType executeComponentFunction);
void executeComponent(FlowState *flowState, unsigned componentIndex);
}
}
// -----------------------------------------------------------------------------
// flow/date.h
// -----------------------------------------------------------------------------
#include <stdint.h>
namespace eez {
namespace flow {
namespace date {
enum DstRule { DST_RULE_OFF, DST_RULE_EUROPE, DST_RULE_USA, DST_RULE_AUSTRALIA };
enum Format { FORMAT_DMY_24, FORMAT_MDY_24, FORMAT_DMY_12, FORMAT_MDY_12 };
typedef uint64_t Date;
extern Format g_localeFormat;
extern int g_timeZone;
extern DstRule g_dstRule;
Date now();
void toString(Date time, char *str, uint32_t strLen);
void toLocaleString(Date time, char *str, uint32_t strLen);
Date fromString(const char *str);
Date makeDate(int year, int month, int day, int hours, int minutes, int seconds, int milliseconds);
void breakDate(Date time, int &year, int &month, int &day, int &hours, int &minutes, int &seconds, int &milliseconds);
int getYear(Date time);
int getMonth(Date time);
int getDay(Date time);
int getHours(Date time);
int getMinutes(Date time);
int getSeconds(Date time);
int getMilliseconds(Date time);
Date utcToLocal(Date utc);
Date localToUtc(Date local);
}
}
}
// -----------------------------------------------------------------------------
// flow/debugger.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
extern bool g_debuggerIsConnected;
enum {
    DEBUGGER_MODE_RUN,
    DEBUGGER_MODE_DEBUG,
};
extern int g_debuggerMode;
bool canExecuteStep(FlowState *&flowState, unsigned &componentIndex);
void onStarted(Assets *assets);
void onStopped();
void onAddToQueue(FlowState *flowState, int sourceComponentIndex, int sourceOutputIndex, unsigned targetComponentIndex, int targetInputIndex);
void onRemoveFromQueue();
void onValueChanged(const Value *pValue);
void onFlowStateCreated(FlowState *flowState);
void onFlowStateDestroyed(FlowState *flowState);
void onFlowStateTimelineChanged(FlowState *flowState);
void onFlowError(FlowState *flowState, int componentIndex, const char *errorMessage);
void onComponentExecutionStateChanged(FlowState *flowState, int componentIndex);
void onComponentAsyncStateChanged(FlowState *flowState, int componentIndex);
void logInfo(FlowState *flowState, unsigned componentIndex, const char *message);
void logScpiCommand(FlowState *flowState, unsigned componentIndex, const char *cmd);
void logScpiQuery(FlowState *flowState, unsigned componentIndex, const char *query);
void logScpiQueryResult(FlowState *flowState, unsigned componentIndex, const char *resultText, size_t resultTextLen);
void onPageChanged(int previousPageId, int activePageId, bool activePageIsFromStack = false, bool previousPageIsStillOnStack = false);
void processDebuggerInput(char *buffer, uint32_t length);
}
}
// -----------------------------------------------------------------------------
// flow/expression.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
#if !defined(EEZ_FLOW_EVAL_STACK_SIZE)
#if defined(EEZ_DASHBOARD_API)
#define EEZ_FLOW_EVAL_STACK_SIZE 10000
#else
#define EEZ_FLOW_EVAL_STACK_SIZE 20
#endif
#endif
static const size_t STACK_SIZE = EEZ_FLOW_EVAL_STACK_SIZE;
struct EvalStack {
	FlowState *flowState;
	int componentIndex;
	const int32_t *iterators;
	Value stack[STACK_SIZE];
	size_t sp = 0;
    char errorMessage[512];
	bool push(const Value &value) {
		if (sp >= STACK_SIZE) {
			throwError(flowState, componentIndex, "Evaluation stack is full\n");
			return false;
		}
		stack[sp++] = value;
		return true;
	}
	bool push(Value *pValue) {
		if (sp >= STACK_SIZE) {
			return false;
		}
		stack[sp++] = Value(pValue, VALUE_TYPE_VALUE_PTR);
		return true;
	}
	Value pop() {
        if (sp == 0) {
            return Value::makeError();
        }
		return stack[--sp];
	}
    void setErrorMessage(const char *str) {
        stringCopy(errorMessage, sizeof(errorMessage), str);
    }
};
#if EEZ_OPTION_GUI
bool evalExpression(FlowState *flowState, int componentIndex, const uint8_t *instructions, Value &result, const char *errorMessage, int *numInstructionBytes = nullptr, const int32_t *iterators = nullptr, eez::gui::DataOperationEnum operation = eez::gui::DATA_OPERATION_GET);
#else
bool evalExpression(FlowState *flowState, int componentIndex, const uint8_t *instructions, Value &result, const char *errorMessage, int *numInstructionBytes = nullptr, const int32_t *iterators = nullptr);
#endif
bool evalAssignableExpression(FlowState *flowState, int componentIndex, const uint8_t *instructions, Value &result, const char *errorMessage, int *numInstructionBytes = nullptr, const int32_t *iterators = nullptr);
#if EEZ_OPTION_GUI
bool evalProperty(FlowState *flowState, int componentIndex, int propertyIndex, Value &result, const char *errorMessage, int *numInstructionBytes = nullptr, const int32_t *iterators = nullptr, eez::gui::DataOperationEnum operation = eez::gui::DATA_OPERATION_GET);
#else
bool evalProperty(FlowState *flowState, int componentIndex, int propertyIndex, Value &result, const char *errorMessage, int *numInstructionBytes = nullptr, const int32_t *iterators = nullptr);
#endif
bool evalAssignableProperty(FlowState *flowState, int componentIndex, int propertyIndex, Value &result, const char *errorMessage, int *numInstructionBytes = nullptr, const int32_t *iterators = nullptr);
}
}
// -----------------------------------------------------------------------------
// flow/flow.h
// -----------------------------------------------------------------------------
#if EEZ_OPTION_GUI
using namespace eez::gui;
#endif
namespace eez {
namespace flow {
#if defined(__EMSCRIPTEN__)
extern uint32_t g_wasmModuleId;
#endif
struct FlowState;
unsigned start(Assets *assets);
void tick();
void stop();
bool isFlowStopped();
#if EEZ_OPTION_GUI
FlowState *getPageFlowState(Assets *assets, int16_t pageIndex, const WidgetCursor &widgetCursor);
#else
FlowState *getPageFlowState(Assets *assets, int16_t pageIndex);
#endif
int getPageIndex(FlowState *flowState);
Value getGlobalVariable(uint32_t globalVariableIndex);
Value getGlobalVariable(Assets *assets, uint32_t globalVariableIndex);
void setGlobalVariable(uint32_t globalVariableIndex, const Value &value);
void setGlobalVariable(Assets *assets, uint32_t globalVariableIndex, const Value &value);
#if EEZ_OPTION_GUI
FlowState *getUserWidgetFlowState(FlowState *flowState, uint16_t userWidgetWidgetComponentIndex, int16_t pageId);
void executeFlowAction(const WidgetCursor &widgetCursor, int16_t actionId, void *param);
void dataOperation(int16_t dataId, DataOperationEnum operation, const WidgetCursor &widgetCursor, Value &value);
int16_t getNativeVariableId(const WidgetCursor &widgetCursor);
#endif
void setDebuggerMessageSubsciptionFilter(uint32_t filter);
void onDebuggerClientConnected();
void onDebuggerClientDisconnected();
void onArrayValueFree(ArrayValue *arrayValue);
void onFreeMQTTConnection(ArrayValue *mqttConnectionValue);
void executeScpi();
void flushToDebuggerMessage();
}
}
// -----------------------------------------------------------------------------
// flow/hooks.h
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <stdlib.h>
#if defined(EEZ_FOR_LVGL)
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#endif
namespace eez {
namespace flow {
extern void (*replacePageHook)(int16_t pageId, uint32_t animType, uint32_t speed, uint32_t delay);
extern void (*showKeyboardHook)(Value label, Value initialText, Value minChars, Value maxChars, bool isPassword, void(*onOk)(char *), void(*onCancel)());
extern void (*showKeypadHook)(Value label, Value initialValue, Value min, Value max, Unit unit, void(*onOk)(float), void(*onCancel)());
extern void (*stopScriptHook)();
extern void (*scpiComponentInitHook)();
extern void (*startToDebuggerMessageHook)();
extern void (*writeDebuggerBufferHook)(const char *buffer, uint32_t length);
extern void (*finishToDebuggerMessageHook)();
extern void (*onDebuggerInputAvailableHook)();
#if defined(EEZ_FOR_LVGL)
extern lv_obj_t *(*getLvglObjectFromIndexHook)(int32_t index);
extern const void *(*getLvglImageByNameHook)(const char *name);
extern void (*executeLvglActionHook)(int actionIndex);
#endif
extern double (*getDateNowHook)();
extern void (*onFlowErrorHook)(FlowState *flowState, int componentIndex, const char *errorMessage);
}
}
// -----------------------------------------------------------------------------
// flow/operations.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
typedef void (*EvalOperation)(EvalStack &);
extern EvalOperation g_evalOperations[];
Value op_add(const Value& a1, const Value& b1);
Value op_sub(const Value& a1, const Value& b1);
Value op_mul(const Value& a1, const Value& b1);
Value op_div(const Value& a1, const Value& b1);
Value op_mod(const Value& a1, const Value& b1);
Value op_left_shift(const Value& a1, const Value& b1);
Value op_right_shift(const Value& a1, const Value& b1);
Value op_binary_and(const Value& a1, const Value& b1);
Value op_binary_or(const Value& a1, const Value& b1);
Value op_binary_xor(const Value& a1, const Value& b1);
Value op_eq(const Value& a1, const Value& b1);
Value op_neq(const Value& a1, const Value& b1);
Value op_less(const Value& a1, const Value& b1);
Value op_great(const Value& a1, const Value& b1);
Value op_less_eq(const Value& a1, const Value& b1);
Value op_great_eq(const Value& a1, const Value& b1);
}
}
// -----------------------------------------------------------------------------
// flow/queue.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
void queueReset();
size_t getQueueSize();
size_t getMaxQueueSize();
extern unsigned g_numContinuousTaskInQueue;
bool addToQueue(FlowState *flowState, unsigned componentIndex,
    int sourceComponentIndex, int sourceOutputIndex, int targetInputIndex,
    bool continuousTask);
bool peekNextTaskFromQueue(FlowState *&flowState, unsigned &componentIndex, bool &continuousTask);
void removeNextTaskFromQueue();
bool isInQueue(FlowState *flowState, unsigned componentIndex);
}
}
// -----------------------------------------------------------------------------
// flow/watch_list.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
struct WatchListNode;
WatchListNode *watchListAdd(FlowState *flowState, unsigned componentIndex);
void watchListRemove(WatchListNode *node);
void visitWatchList();
void watchListReset();
}
}
// -----------------------------------------------------------------------------
// flow/components/call_action.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
struct CallActionActionComponent : public Component {
	int16_t flowIndex;
	uint8_t inputsStartIndex;
	uint8_t outputsStartIndex;
};
}
}
// -----------------------------------------------------------------------------
// flow/components/input.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
struct InputActionComponent : public Component {
	uint8_t inputIndex;
};
struct InputActionComponentExecutionState : public ComponenentExecutionState {
	Value value;
};
bool getCallActionValue(FlowState *flowState, unsigned componentIndex, Value &value);
}
}
// -----------------------------------------------------------------------------
// flow/components/lvgl.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
enum LVGL_ACTIONS {
    CHANGE_SCREEN,
    PLAY_ANIMATION,
    SET_PROPERTY
};
struct LVGLComponent_ActionType {
    uint32_t action;
};
struct LVGLComponent_ChangeScreen_ActionType : public LVGLComponent_ActionType {
    int32_t screen;
    uint32_t fadeMode;
    uint32_t speed;
    uint32_t delay;
};
#define ANIMATION_PROPERTY_POSITION_X 0
#define ANIMATION_PROPERTY_POSITION_Y 1
#define ANIMATION_PROPERTY_WIDTH 2
#define ANIMATION_PROPERTY_HEIGHT 3
#define ANIMATION_PROPERTY_OPACITY 4
#define ANIMATION_PROPERTY_IMAGE_ANGLE 5
#define ANIMATION_PROPERTY_IMAGE_ZOOM
#define ANIMATION_ITEM_FLAG_RELATIVE (1 << 0)
#define ANIMATION_ITEM_FLAG_INSTANT (1 << 1)
#define ANIMATION_PATH_LINEAR 0
#define ANIMATION_PATH_EASE_IN 1
#define ANIMATION_PATH_EASE_OUT 2
#define ANIMATION_PATH_EASE_IN_OUT 3
#define ANIMATION_PATH_OVERSHOOT 4
#define ANIMATION_PATH_BOUNCE 5
struct LVGLComponent_PlayAnimation_ActionType : public LVGLComponent_ActionType {
    int32_t target;
    uint32_t property;
    int32_t start;
    int32_t end;
    uint32_t delay;
    uint32_t time;
    uint32_t flags;
    uint32_t path;
};
struct LVGLComponent_SetProperty_ActionType : public LVGLComponent_ActionType {
    int32_t target;
    uint32_t property;
    AssetsPtr<uint8_t> value;
    int32_t textarea;
    uint32_t animated;
};
struct LVGLComponent : public Component {
    ListOfAssetsPtr<LVGLComponent_ActionType> actions;
};
}
}
// -----------------------------------------------------------------------------
// flow/components/lvgl_user_widget.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
struct LVGLUserWidgetExecutionState : public ComponenentExecutionState {
    FlowState *flowState;
    ~LVGLUserWidgetExecutionState() {
        freeFlowState(flowState);
    }
};
LVGLUserWidgetExecutionState *createUserWidgetFlowState(FlowState *flowState, unsigned userWidgetWidgetComponentIndex);
}
}
// -----------------------------------------------------------------------------
// flow/components/mqtt.h
// -----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif
#define MQTT_ERROR_OK 0
#define MQTT_ERROR_OTHER 1
#define MQTT_ERROR_NOT_IMPLEMENTED 2
int eez_mqtt_init(const char *protocol, const char *host, int port, const char *username, const char *password, void **handle);
int eez_mqtt_deinit(void *handle);
int eez_mqtt_connect(void *handle);
int eez_mqtt_disconnect(void *handle);
int eez_mqtt_subscribe(void *handle, const char *topic);
int eez_mqtt_unsubscribe(void *handle, const char *topic);
int eez_mqtt_publish(void *handle, const char *topic, const char *payload);
typedef enum {
    EEZ_MQTT_EVENT_CONNECT = 0,
    EEZ_MQTT_EVENT_RECONNECT = 1,
    EEZ_MQTT_EVENT_CLOSE = 2,
    EEZ_MQTT_EVENT_DISCONNECT = 3,
    EEZ_MQTT_EVENT_OFFLINE = 4,
    EEZ_MQTT_EVENT_END = 5,
    EEZ_MQTT_EVENT_ERROR = 6,
    EEZ_MQTT_EVENT_MESSAGE = 7
} EEZ_MQTT_Event;
typedef struct {
    const char *topic;
    const char *payload;
} EEZ_MQTT_MessageEvent;
void eez_mqtt_on_event_callback(void *handle, EEZ_MQTT_Event event, void *eventData);
#ifdef __cplusplus
}
#endif
// -----------------------------------------------------------------------------
// flow/components/on_event.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
struct OnEventComponent : public Component {
    uint8_t event;
};
}
}
// -----------------------------------------------------------------------------
// flow/components/set_variable.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
struct SetVariableEntry {
    AssetsPtr<uint8_t> variable;
    AssetsPtr<uint8_t> value;
};
struct SetVariableActionComponent : public Component {
    ListOfAssetsPtr<SetVariableEntry> entries;
};
}
}
// -----------------------------------------------------------------------------
// flow/components/sort_array.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
#define SORT_ARRAY_FLAG_ASCENDING   (1 << 0)
#define SORT_ARRAY_FLAG_IGNORE_CASE (1 << 1)
struct SortArrayActionComponent : public Component {
    int32_t arrayType;
    int32_t structFieldIndex;
    uint32_t flags;
};
void sortArray(SortArrayActionComponent *component, ArrayValue *array);
}
}
// -----------------------------------------------------------------------------
// flow/components/switch.h
// -----------------------------------------------------------------------------
namespace eez {
namespace flow {
struct SwitchTest {
    uint32_t outputIndex;
    AssetsPtr<uint8_t> condition;
    AssetsPtr<uint8_t> outputValue;
};
struct SwitchActionComponent : public Component {
    ListOfAssetsPtr<SwitchTest> tests;
};
}
}
#endif

// -----------------------------------------------------------------------------
// core/vars.h
// -----------------------------------------------------------------------------
#ifndef EEZ_FRAMEWORK_CORE_VARS_H
#define EEZ_FRAMEWORK_CORE_VARS_H
typedef enum {
    NATIVE_VAR_TYPE_NONE,
    NATIVE_VAR_TYPE_INTEGER,
    NATIVE_VAR_TYPE_BOOLEAN,
    NATIVE_VAR_TYPE_FLOAT,
    NATIVE_VAR_TYPE_DOUBLE,
    NATIVE_VAR_TYPE_STRING,
} NativeVarType;
typedef struct _native_var_t {
    NativeVarType type;
    void *get;
    void *set;
} native_var_t;
#ifdef __cplusplus
extern "C" {
#endif
extern native_var_t native_vars[];
#ifdef __cplusplus
}
#endif
#endif
// -----------------------------------------------------------------------------
// flow/lvgl_api.h
// -----------------------------------------------------------------------------
#include <math.h>
#if defined(EEZ_FOR_LVGL)
#include <stdint.h>
#if defined(EEZ_FOR_LVGL)
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#endif
#if LVGL_VERSION_MAJOR >= 9
#define lv_scr_load_anim_t lv_screen_load_anim_t
#define lv_scr_load_anim lv_screen_load_anim
#endif
#ifdef __cplusplus
extern "C" {
#endif
#ifndef EXT_IMG_DESC_T
#define EXT_IMG_DESC_T
typedef struct _ext_img_desc_t {
    const char *name;
    const lv_img_dsc_t *img_dsc;
} ext_img_desc_t;
#endif
typedef void (*ActionExecFunc)(lv_event_t * e);
void eez_flow_init(const uint8_t *assets, uint32_t assetsSize, lv_obj_t **objects, size_t numObjects, const ext_img_desc_t *images, size_t numImages, ActionExecFunc *actions);
void eez_flow_tick();
bool eez_flow_is_stopped();
extern int16_t g_currentScreen;
int16_t eez_flow_get_current_screen();
void eez_flow_set_screen(int16_t screenId, lv_scr_load_anim_t animType, uint32_t speed, uint32_t delay);
void eez_flow_push_screen(int16_t screenId, lv_scr_load_anim_t animType, uint32_t speed, uint32_t delay);
void eez_flow_pop_screen(lv_scr_load_anim_t animType, uint32_t speed, uint32_t delay);
void flowOnPageLoaded(unsigned pageIndex);
void *getFlowState(void *flowState, unsigned userWidgetComponentIndexOrPageIndex);
void flowPropagateValue(void *flowState, unsigned componentIndex, unsigned outputIndex);
const char *evalTextProperty(void *flowState, unsigned componentIndex, unsigned propertyIndex, const char *errorMessage);
int32_t evalIntegerProperty(void *flowState, unsigned componentIndex, unsigned propertyIndex, const char *errorMessage);
bool evalBooleanProperty(void *flowState, unsigned componentIndex, unsigned propertyIndex, const char *errorMessage);
const char *evalStringArrayPropertyAndJoin(void *flowState, unsigned componentIndex, unsigned propertyIndex, const char *errorMessage, const char *separator);
void assignStringProperty(void *flowState, unsigned componentIndex, unsigned propertyIndex, const char *value, const char *errorMessage);
void assignIntegerProperty(void *flowState, unsigned componentIndex, unsigned propertyIndex, int32_t value, const char *errorMessage);
void assignBooleanProperty(void *flowState, unsigned componentIndex, unsigned propertyIndex, bool value, const char *errorMessage);
float eez_linear(float x);
float eez_easeInQuad(float x);
float eez_easeOutQuad(float x);
float eez_easeInOutQuad(float x);
float eez_easeInCubic(float x);
float eez_easeOutCubic(float x);
float eez_easeInOutCubic(float x);
float eez_easeInQuart(float x);
float eez_easeOutQuart(float x);
float eez_easeInOutQuart(float x);
float eez_easeInQuint(float x);
float eez_easeOutQuint(float x);
float eez_easeInOutQuint(float x);
float eez_easeInSine(float x);
float eez_easeOutSine(float x);
float eez_easeInOutSine(float x);
float eez_easeInExpo(float x);
float eez_easeOutExpo(float x);
float eez_easeInOutExpo(float x);
float eez_easeInCirc(float x);
float eez_easeOutCirc(float x);
float eez_easeInOutCirc(float x);
float eez_easeInBack(float x);
float eez_easeOutBack(float x);
float eez_easeInOutBack(float x);
float eez_easeInElastic(float x);
float eez_easeOutElastic(float x);
float eez_easeInOutElastic(float x);
float eez_easeOutBounce(float x);
float eez_easeInBounce(float x);
float eez_easeOutBounce(float x);
float eez_easeInOutBounce(float x);
float getTimelinePosition(void *flowState);
extern int g_eezFlowLvlgMeterTickIndex;
bool compareRollerOptions(lv_roller_t *roller, const char *new_val, const char *cur_val, lv_roller_mode_t mode);
#ifdef __cplusplus
}
#endif
#endif

/* The latest version of this library is available on GitHub;
 * https://github.com/sheredom/utf8.h */

/* This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <http://unlicense.org/> */

#ifndef SHEREDOM_UTF8_H_INCLUDED
#define SHEREDOM_UTF8_H_INCLUDED

#if defined(_MSC_VER)
#pragma warning(push)

/* disable warning: no function prototype given: converting '()' to '(void)' */
#pragma warning(disable : 4255)

/* disable warning: '__cplusplus' is not defined as a preprocessor macro,
 * replacing with '0' for '#if/#elif' */
#pragma warning(disable : 4668)

/* disable warning: bytes padding added after construct */
#pragma warning(disable : 4820)
#endif

#include <stddef.h>
#include <stdlib.h>

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#if defined(_MSC_VER) && (_MSC_VER < 1920)
typedef __int32 utf8_int32_t;
#else
#include <stdint.h>
typedef int32_t utf8_int32_t;
#endif

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wold-style-cast"
#pragma clang diagnostic ignored "-Wcast-qual"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_MSC_VER)
#define utf8_nonnull
#define utf8_pure
#define utf8_restrict __restrict
#define utf8_weak __inline
#elif defined(__clang__) || defined(__GNUC__)
#define utf8_nonnull __attribute__((nonnull))
#define utf8_pure __attribute__((pure))
#define utf8_restrict __restrict__
#define utf8_weak __attribute__((weak))
#else
#error Non clang, non gcc, non MSVC compiler found!
#endif

#ifdef __cplusplus
#define utf8_null NULL
#else
#define utf8_null 0
#endif

#if (defined(__cplusplus) && __cplusplus >= 201402L)
#define utf8_constexpr14 constexpr
#define utf8_constexpr14_impl constexpr
#else
/* constexpr and weak are incompatible. so only enable one of them */
#define utf8_constexpr14 utf8_weak
#define utf8_constexpr14_impl
#endif

#if defined(__cplusplus) && __cplusplus >= 202002L
// using utf8_int8_t = char8_t; /* Introduced in C++20 */
typedef char utf8_int8_t;
#else
typedef char utf8_int8_t;
#endif

/* Return less than 0, 0, greater than 0 if src1 < src2, src1 == src2, src1 >
 * src2 respectively, case insensitive. */
utf8_constexpr14 utf8_nonnull utf8_pure int
utf8casecmp(const utf8_int8_t *src1, const utf8_int8_t *src2);

/* Append the utf8 string src onto the utf8 string dst. */
utf8_nonnull utf8_weak utf8_int8_t *
utf8cat(utf8_int8_t *utf8_restrict dst, const utf8_int8_t *utf8_restrict src);

/* Find the first match of the utf8 codepoint chr in the utf8 string src. */
utf8_constexpr14 utf8_nonnull utf8_pure utf8_int8_t *
utf8chr(const utf8_int8_t *src, utf8_int32_t chr);

/* Return less than 0, 0, greater than 0 if src1 < src2,
 * src1 == src2, src1 > src2 respectively. */
utf8_constexpr14 utf8_nonnull utf8_pure int utf8cmp(const utf8_int8_t *src1,
                                                    const utf8_int8_t *src2);

/* Copy the utf8 string src onto the memory allocated in dst. */
utf8_nonnull utf8_weak utf8_int8_t *
utf8cpy(utf8_int8_t *utf8_restrict dst, const utf8_int8_t *utf8_restrict src);

/* Number of utf8 codepoints in the utf8 string src that consists entirely
 * of utf8 codepoints not from the utf8 string reject. */
utf8_constexpr14 utf8_nonnull utf8_pure size_t
utf8cspn(const utf8_int8_t *src, const utf8_int8_t *reject);

/* Duplicate the utf8 string src by getting its size, malloc'ing a new buffer
 * copying over the data, and returning that. Or 0 if malloc failed. */
utf8_weak utf8_int8_t *utf8dup(const utf8_int8_t *src);

/* Number of utf8 codepoints in the utf8 string str,
 * excluding the null terminating byte. */
utf8_constexpr14 utf8_nonnull utf8_pure size_t utf8len(const utf8_int8_t *str);

/* Similar to utf8len, except that only at most n bytes of src are looked. */
utf8_constexpr14 utf8_nonnull utf8_pure size_t utf8nlen(const utf8_int8_t *str,
                                                        size_t n);

/* Return less than 0, 0, greater than 0 if src1 < src2, src1 == src2, src1 >
 * src2 respectively, case insensitive. Checking at most n bytes of each utf8
 * string. */
utf8_constexpr14 utf8_nonnull utf8_pure int
utf8ncasecmp(const utf8_int8_t *src1, const utf8_int8_t *src2, size_t n);

/* Append the utf8 string src onto the utf8 string dst,
 * writing at most n+1 bytes. Can produce an invalid utf8
 * string if n falls partway through a utf8 codepoint. */
utf8_nonnull utf8_weak utf8_int8_t *
utf8ncat(utf8_int8_t *utf8_restrict dst, const utf8_int8_t *utf8_restrict src,
         size_t n);

/* Return less than 0, 0, greater than 0 if src1 < src2,
 * src1 == src2, src1 > src2 respectively. Checking at most n
 * bytes of each utf8 string. */
utf8_constexpr14 utf8_nonnull utf8_pure int
utf8ncmp(const utf8_int8_t *src1, const utf8_int8_t *src2, size_t n);

/* Copy the utf8 string src onto the memory allocated in dst.
 * Copies at most n bytes. If n falls partway through a utf8
 * codepoint, or if dst doesn't have enough room for a null
 * terminator, the final string will be cut short to preserve
 * utf8 validity. */

utf8_nonnull utf8_weak utf8_int8_t *
utf8ncpy(utf8_int8_t *utf8_restrict dst, const utf8_int8_t *utf8_restrict src,
         size_t n);

/* Similar to utf8dup, except that at most n bytes of src are copied. If src is
 * longer than n, only n bytes are copied and a null byte is added.
 *
 * Returns a new string if successful, 0 otherwise */
utf8_weak utf8_int8_t *utf8ndup(const utf8_int8_t *src, size_t n);

/* Locates the first occurrence in the utf8 string str of any byte in the
 * utf8 string accept, or 0 if no match was found. */
utf8_constexpr14 utf8_nonnull utf8_pure utf8_int8_t *
utf8pbrk(const utf8_int8_t *str, const utf8_int8_t *accept);

/* Find the last match of the utf8 codepoint chr in the utf8 string src. */
utf8_constexpr14 utf8_nonnull utf8_pure utf8_int8_t *
utf8rchr(const utf8_int8_t *src, int chr);

/* Number of bytes in the utf8 string str,
 * including the null terminating byte. */
utf8_constexpr14 utf8_nonnull utf8_pure size_t utf8size(const utf8_int8_t *str);

/* Similar to utf8size, except that the null terminating byte is excluded. */
utf8_constexpr14 utf8_nonnull utf8_pure size_t
utf8size_lazy(const utf8_int8_t *str);

/* Similar to utf8size, except that only at most n bytes of src are looked and
 * the null terminating byte is excluded. */
utf8_constexpr14 utf8_nonnull utf8_pure size_t
utf8nsize_lazy(const utf8_int8_t *str, size_t n);

/* Number of utf8 codepoints in the utf8 string src that consists entirely
 * of utf8 codepoints from the utf8 string accept. */
utf8_constexpr14 utf8_nonnull utf8_pure size_t
utf8spn(const utf8_int8_t *src, const utf8_int8_t *accept);

/* The position of the utf8 string needle in the utf8 string haystack. */
utf8_constexpr14 utf8_nonnull utf8_pure utf8_int8_t *
utf8str(const utf8_int8_t *haystack, const utf8_int8_t *needle);

/* The position of the utf8 string needle in the utf8 string haystack, case
 * insensitive. */
utf8_constexpr14 utf8_nonnull utf8_pure utf8_int8_t *
utf8casestr(const utf8_int8_t *haystack, const utf8_int8_t *needle);

/* Return 0 on success, or the position of the invalid
 * utf8 codepoint on failure. */
utf8_constexpr14 utf8_nonnull utf8_pure utf8_int8_t *
utf8valid(const utf8_int8_t *str);

/* Similar to utf8valid, except that only at most n bytes of src are looked. */
utf8_constexpr14 utf8_nonnull utf8_pure utf8_int8_t *
utf8nvalid(const utf8_int8_t *str, size_t n);

/* Given a null-terminated string, makes the string valid by replacing invalid
 * codepoints with a 1-byte replacement. Returns 0 on success. */
utf8_nonnull utf8_weak int utf8makevalid(utf8_int8_t *str,
                                         const utf8_int32_t replacement);

/* Sets out_codepoint to the current utf8 codepoint in str, and returns the
 * address of the next utf8 codepoint after the current one in str. */
utf8_constexpr14 utf8_nonnull utf8_int8_t *
utf8codepoint(const utf8_int8_t *utf8_restrict str,
              utf8_int32_t *utf8_restrict out_codepoint);

/* Calculates the size of the next utf8 codepoint in str. */
utf8_constexpr14 utf8_nonnull size_t
utf8codepointcalcsize(const utf8_int8_t *str);

/* Returns the size of the given codepoint in bytes. */
utf8_constexpr14 size_t utf8codepointsize(utf8_int32_t chr);

/* Write a codepoint to the given string, and return the address to the next
 * place after the written codepoint. Pass how many bytes left in the buffer to
 * n. If there is not enough space for the codepoint, this function returns
 * null. */
utf8_nonnull utf8_weak utf8_int8_t *
utf8catcodepoint(utf8_int8_t *str, utf8_int32_t chr, size_t n);

/* Returns 1 if the given character is lowercase, or 0 if it is not. */
utf8_constexpr14 int utf8islower(utf8_int32_t chr);

/* Returns 1 if the given character is uppercase, or 0 if it is not. */
utf8_constexpr14 int utf8isupper(utf8_int32_t chr);

/* Transform the given string into all lowercase codepoints. */
utf8_nonnull utf8_weak void utf8lwr(utf8_int8_t *utf8_restrict str);

/* Transform the given string into all uppercase codepoints. */
utf8_nonnull utf8_weak void utf8upr(utf8_int8_t *utf8_restrict str);

/* Make a codepoint lower case if possible. */
utf8_constexpr14 utf8_int32_t utf8lwrcodepoint(utf8_int32_t cp);

/* Make a codepoint upper case if possible. */
utf8_constexpr14 utf8_int32_t utf8uprcodepoint(utf8_int32_t cp);

/* Sets out_codepoint to the current utf8 codepoint in str, and returns the
 * address of the previous utf8 codepoint before the current one in str. */
utf8_constexpr14 utf8_nonnull utf8_int8_t *
utf8rcodepoint(const utf8_int8_t *utf8_restrict str,
               utf8_int32_t *utf8_restrict out_codepoint);

/* Duplicate the utf8 string src by getting its size, calling alloc_func_ptr to
 * copy over data to a new buffer, and returning that. Or 0 if alloc_func_ptr
 * returned null. */
utf8_weak utf8_int8_t *utf8dup_ex(const utf8_int8_t *src,
                                  utf8_int8_t *(*alloc_func_ptr)(utf8_int8_t *,
                                                                 size_t),
                                  utf8_int8_t *user_data);

/* Similar to utf8dup, except that at most n bytes of src are copied. If src is
 * longer than n, only n bytes are copied and a null byte is added.
 *
 * Returns a new string if successful, 0 otherwise. */
utf8_weak utf8_int8_t *utf8ndup_ex(const utf8_int8_t *src, size_t n,
                                   utf8_int8_t *(*alloc_func_ptr)(utf8_int8_t *,
                                                                  size_t),
                                   utf8_int8_t *user_data);

#undef utf8_weak
#undef utf8_pure
#undef utf8_nonnull

utf8_constexpr14_impl int utf8casecmp(const utf8_int8_t *src1,
                                      const utf8_int8_t *src2) {
  utf8_int32_t src1_lwr_cp = 0, src2_lwr_cp = 0, src1_upr_cp = 0,
               src2_upr_cp = 0, src1_orig_cp = 0, src2_orig_cp = 0;

  for (;;) {
    src1 = utf8codepoint(src1, &src1_orig_cp);
    src2 = utf8codepoint(src2, &src2_orig_cp);

    /* lower the srcs if required */
    src1_lwr_cp = utf8lwrcodepoint(src1_orig_cp);
    src2_lwr_cp = utf8lwrcodepoint(src2_orig_cp);

    /* lower the srcs if required */
    src1_upr_cp = utf8uprcodepoint(src1_orig_cp);
    src2_upr_cp = utf8uprcodepoint(src2_orig_cp);

    /* check if the lowered codepoints match */
    if ((0 == src1_orig_cp) && (0 == src2_orig_cp)) {
      return 0;
    } else if ((src1_lwr_cp == src2_lwr_cp) || (src1_upr_cp == src2_upr_cp)) {
      continue;
    }

    /* if they don't match, then we return the difference between the characters
     */
    return src1_lwr_cp - src2_lwr_cp;
  }
}

utf8_int8_t *utf8cat(utf8_int8_t *utf8_restrict dst,
                     const utf8_int8_t *utf8_restrict src) {
  utf8_int8_t *d = dst;
  /* find the null terminating byte in dst */
  while ('\0' != *d) {
    d++;
  }

  /* overwriting the null terminating byte in dst, append src byte-by-byte */
  while ('\0' != *src) {
    *d++ = *src++;
  }

  /* write out a new null terminating byte into dst */
  *d = '\0';

  return dst;
}

utf8_constexpr14_impl utf8_int8_t *utf8chr(const utf8_int8_t *src,
                                           utf8_int32_t chr) {
  utf8_int8_t c[5] = {'\0', '\0', '\0', '\0', '\0'};

  if (0 == chr) {
    /* being asked to return position of null terminating byte, so
     * just run s to the end, and return! */
    while ('\0' != *src) {
      src++;
    }
    return (utf8_int8_t *)src;
  } else if (0 == ((utf8_int32_t)0xffffff80 & chr)) {
    /* 1-byte/7-bit ascii
     * (0b0xxxxxxx) */
    c[0] = (utf8_int8_t)chr;
  } else if (0 == ((utf8_int32_t)0xfffff800 & chr)) {
    /* 2-byte/11-bit utf8 code point
     * (0b110xxxxx 0b10xxxxxx) */
    c[0] = (utf8_int8_t)(0xc0 | (utf8_int8_t)(chr >> 6));
    c[1] = (utf8_int8_t)(0x80 | (utf8_int8_t)(chr & 0x3f));
  } else if (0 == ((utf8_int32_t)0xffff0000 & chr)) {
    /* 3-byte/16-bit utf8 code point
     * (0b1110xxxx 0b10xxxxxx 0b10xxxxxx) */
    c[0] = (utf8_int8_t)(0xe0 | (utf8_int8_t)(chr >> 12));
    c[1] = (utf8_int8_t)(0x80 | (utf8_int8_t)((chr >> 6) & 0x3f));
    c[2] = (utf8_int8_t)(0x80 | (utf8_int8_t)(chr & 0x3f));
  } else { /* if (0 == ((int)0xffe00000 & chr)) { */
    /* 4-byte/21-bit utf8 code point
     * (0b11110xxx 0b10xxxxxx 0b10xxxxxx 0b10xxxxxx) */
    c[0] = (utf8_int8_t)(0xf0 | (utf8_int8_t)(chr >> 18));
    c[1] = (utf8_int8_t)(0x80 | (utf8_int8_t)((chr >> 12) & 0x3f));
    c[2] = (utf8_int8_t)(0x80 | (utf8_int8_t)((chr >> 6) & 0x3f));
    c[3] = (utf8_int8_t)(0x80 | (utf8_int8_t)(chr & 0x3f));
  }

  /* we've made c into a 2 utf8 codepoint string, one for the chr we are
   * seeking, another for the null terminating byte. Now use utf8str to
   * search */
  return utf8str(src, c);
}

utf8_constexpr14_impl int utf8cmp(const utf8_int8_t *src1,
                                  const utf8_int8_t *src2) {
  while (('\0' != *src1) || ('\0' != *src2)) {
    if (*src1 < *src2) {
      return -1;
    } else if (*src1 > *src2) {
      return 1;
    }

    src1++;
    src2++;
  }

  /* both utf8 strings matched */
  return 0;
}

utf8_constexpr14_impl int utf8coll(const utf8_int8_t *src1,
                                   const utf8_int8_t *src2);

utf8_int8_t *utf8cpy(utf8_int8_t *utf8_restrict dst,
                     const utf8_int8_t *utf8_restrict src) {
  utf8_int8_t *d = dst;

  /* overwriting anything previously in dst, write byte-by-byte
   * from src */
  while ('\0' != *src) {
    *d++ = *src++;
  }

  /* append null terminating byte */
  *d = '\0';

  return dst;
}

utf8_constexpr14_impl size_t utf8cspn(const utf8_int8_t *src,
                                      const utf8_int8_t *reject) {
  size_t chars = 0;

  while ('\0' != *src) {
    const utf8_int8_t *r = reject;
    size_t offset = 0;

    while ('\0' != *r) {
      /* checking that if *r is the start of a utf8 codepoint
       * (it is not 0b10xxxxxx) and we have successfully matched
       * a previous character (0 < offset) - we found a match */
      if ((0x80 != (0xc0 & *r)) && (0 < offset)) {
        return chars;
      } else {
        if (*r == src[offset]) {
          /* part of a utf8 codepoint matched, so move our checking
           * onwards to the next byte */
          offset++;
          r++;
        } else {
          /* r could be in the middle of an unmatching utf8 code point,
           * so we need to march it on to the next character beginning, */

          do {
            r++;
          } while (0x80 == (0xc0 & *r));

          /* reset offset too as we found a mismatch */
          offset = 0;
        }
      }
    }

    /* found a match at the end of *r, so didn't get a chance to test it */
    if (0 < offset) {
      return chars;
    }

    /* the current utf8 codepoint in src did not match reject, but src
     * could have been partway through a utf8 codepoint, so we need to
     * march it onto the next utf8 codepoint starting byte */
    do {
      src++;
    } while ((0x80 == (0xc0 & *src)));
    chars++;
  }

  return chars;
}

utf8_int8_t *utf8dup(const utf8_int8_t *src) {
  return utf8dup_ex(src, utf8_null, utf8_null);
}

utf8_int8_t *utf8dup_ex(const utf8_int8_t *src,
                        utf8_int8_t *(*alloc_func_ptr)(utf8_int8_t *, size_t),
                        utf8_int8_t *user_data) {
  utf8_int8_t *n = utf8_null;

  /* figure out how many bytes (including the terminator) we need to copy first
   */
  size_t bytes = utf8size(src);

  if (alloc_func_ptr) {
    n = alloc_func_ptr(user_data, bytes);
  } else {
    n = (utf8_int8_t *)malloc(bytes);
  }

  if (utf8_null == n) {
    /* out of memory so we bail */
    return utf8_null;
  } else {
    bytes = 0;

    /* copy src byte-by-byte into our new utf8 string */
    while ('\0' != src[bytes]) {
      n[bytes] = src[bytes];
      bytes++;
    }

    /* append null terminating byte */
    n[bytes] = '\0';
    return n;
  }
}

utf8_constexpr14_impl utf8_int8_t *utf8fry(const utf8_int8_t *str);

utf8_constexpr14_impl size_t utf8len(const utf8_int8_t *str) {
  return utf8nlen(str, SIZE_MAX);
}

utf8_constexpr14_impl size_t utf8nlen(const utf8_int8_t *str, size_t n) {
  const utf8_int8_t *t = str;
  size_t length = 0;

  while ((size_t)(str - t) < n && '\0' != *str) {
    if (0xf0 == (0xf8 & *str)) {
      /* 4-byte utf8 code point (began with 0b11110xxx) */
      str += 4;
    } else if (0xe0 == (0xf0 & *str)) {
      /* 3-byte utf8 code point (began with 0b1110xxxx) */
      str += 3;
    } else if (0xc0 == (0xe0 & *str)) {
      /* 2-byte utf8 code point (began with 0b110xxxxx) */
      str += 2;
    } else { /* if (0x00 == (0x80 & *s)) { */
      /* 1-byte ascii (began with 0b0xxxxxxx) */
      str += 1;
    }

    /* no matter the bytes we marched s forward by, it was
     * only 1 utf8 codepoint */
    length++;
  }

  if ((size_t)(str - t) > n) {
    length--;
  }
  return length;
}

utf8_constexpr14_impl int utf8ncasecmp(const utf8_int8_t *src1,
                                       const utf8_int8_t *src2, size_t n) {
  utf8_int32_t src1_lwr_cp = 0, src2_lwr_cp = 0, src1_upr_cp = 0,
               src2_upr_cp = 0, src1_orig_cp = 0, src2_orig_cp = 0;

  do {
    const utf8_int8_t *const s1 = src1;
    const utf8_int8_t *const s2 = src2;

    /* first check that we have enough bytes left in n to contain an entire
     * codepoint */
    if (0 == n) {
      return 0;
    }

    if ((1 == n) && ((0xc0 == (0xe0 & *s1)) || (0xc0 == (0xe0 & *s2)))) {
      const utf8_int32_t c1 = (0xe0 & *s1);
      const utf8_int32_t c2 = (0xe0 & *s2);

      if (c1 < c2) {
        return c1 - c2;
      } else {
        return 0;
      }
    }

    if ((2 >= n) && ((0xe0 == (0xf0 & *s1)) || (0xe0 == (0xf0 & *s2)))) {
      const utf8_int32_t c1 = (0xf0 & *s1);
      const utf8_int32_t c2 = (0xf0 & *s2);

      if (c1 < c2) {
        return c1 - c2;
      } else {
        return 0;
      }
    }

    if ((3 >= n) && ((0xf0 == (0xf8 & *s1)) || (0xf0 == (0xf8 & *s2)))) {
      const utf8_int32_t c1 = (0xf8 & *s1);
      const utf8_int32_t c2 = (0xf8 & *s2);

      if (c1 < c2) {
        return c1 - c2;
      } else {
        return 0;
      }
    }

    src1 = utf8codepoint(src1, &src1_orig_cp);
    src2 = utf8codepoint(src2, &src2_orig_cp);
    n -= utf8codepointsize(src1_orig_cp);

    src1_lwr_cp = utf8lwrcodepoint(src1_orig_cp);
    src2_lwr_cp = utf8lwrcodepoint(src2_orig_cp);

    src1_upr_cp = utf8uprcodepoint(src1_orig_cp);
    src2_upr_cp = utf8uprcodepoint(src2_orig_cp);

    /* check if the lowered codepoints match */
    if ((0 == src1_orig_cp) && (0 == src2_orig_cp)) {
      return 0;
    } else if ((src1_lwr_cp == src2_lwr_cp) || (src1_upr_cp == src2_upr_cp)) {
      continue;
    }

    /* if they don't match, then we return the difference between the characters
     */
    return src1_lwr_cp - src2_lwr_cp;
  } while (0 < n);

  /* both utf8 strings matched */
  return 0;
}

utf8_int8_t *utf8ncat(utf8_int8_t *utf8_restrict dst,
                      const utf8_int8_t *utf8_restrict src, size_t n) {
  utf8_int8_t *d = dst;

  /* find the null terminating byte in dst */
  while ('\0' != *d) {
    d++;
  }

  /* overwriting the null terminating byte in dst, append src byte-by-byte
   * stopping if we run out of space */
  while (('\0' != *src) && (0 != n--)) {
    *d++ = *src++;
  }

  /* write out a new null terminating byte into dst */
  *d = '\0';

  return dst;
}

utf8_constexpr14_impl int utf8ncmp(const utf8_int8_t *src1,
                                   const utf8_int8_t *src2, size_t n) {
  while ((0 != n--) && (('\0' != *src1) || ('\0' != *src2))) {
    if (*src1 < *src2) {
      return -1;
    } else if (*src1 > *src2) {
      return 1;
    }

    src1++;
    src2++;
  }

  /* both utf8 strings matched */
  return 0;
}

utf8_int8_t *utf8ncpy(utf8_int8_t *utf8_restrict dst,
                      const utf8_int8_t *utf8_restrict src, size_t n) {
  utf8_int8_t *d = dst;
  size_t index = 0, check_index = 0;

  if (n == 0) {
    return dst;
  }

  /* overwriting anything previously in dst, write byte-by-byte
   * from src */
  for (index = 0; index < n; index++) {
    d[index] = src[index];
    if ('\0' == src[index]) {
      break;
    }
  }

  for (check_index = index - 1;
       check_index > 0 && 0x80 == (0xc0 & d[check_index]); check_index--) {
    /* just moving the index */
  }

  if (check_index < index &&
      (index - check_index) < utf8codepointsize(d[check_index])) {
    index = check_index;
  }

  /* append null terminating byte */
  for (; index < n; index++) {
    d[index] = 0;
  }

  return dst;
}

utf8_int8_t *utf8ndup(const utf8_int8_t *src, size_t n) {
  return utf8ndup_ex(src, n, utf8_null, utf8_null);
}

utf8_int8_t *utf8ndup_ex(const utf8_int8_t *src, size_t n,
                         utf8_int8_t *(*alloc_func_ptr)(utf8_int8_t *, size_t),
                         utf8_int8_t *user_data) {
  utf8_int8_t *c = utf8_null;
  size_t bytes = 0;

  /* Find the end of the string or stop when n is reached */
  while ('\0' != src[bytes] && bytes < n) {
    bytes++;
  }

  /* In case bytes is actually less than n, we need to set it
   * to be used later in the copy byte by byte. */
  n = bytes;

  if (alloc_func_ptr) {
    c = alloc_func_ptr(user_data, bytes + 1);
  } else {
    c = (utf8_int8_t *)malloc(bytes + 1);
  }

  if (utf8_null == c) {
    /* out of memory so we bail */
    return utf8_null;
  }

  bytes = 0;

  /* copy src byte-by-byte into our new utf8 string */
  while ('\0' != src[bytes] && bytes < n) {
    c[bytes] = src[bytes];
    bytes++;
  }

  /* append null terminating byte */
  c[bytes] = '\0';
  return c;
}

utf8_constexpr14_impl utf8_int8_t *utf8rchr(const utf8_int8_t *src, int chr) {

  utf8_int8_t *match = utf8_null;
  utf8_int8_t c[5] = {'\0', '\0', '\0', '\0', '\0'};

  if (0 == chr) {
    /* being asked to return position of null terminating byte, so
     * just run s to the end, and return! */
    while ('\0' != *src) {
      src++;
    }
    return (utf8_int8_t *)src;
  } else if (0 == ((int)0xffffff80 & chr)) {
    /* 1-byte/7-bit ascii
     * (0b0xxxxxxx) */
    c[0] = (utf8_int8_t)chr;
  } else if (0 == ((int)0xfffff800 & chr)) {
    /* 2-byte/11-bit utf8 code point
     * (0b110xxxxx 0b10xxxxxx) */
    c[0] = (utf8_int8_t)(0xc0 | (utf8_int8_t)(chr >> 6));
    c[1] = (utf8_int8_t)(0x80 | (utf8_int8_t)(chr & 0x3f));
  } else if (0 == ((int)0xffff0000 & chr)) {
    /* 3-byte/16-bit utf8 code point
     * (0b1110xxxx 0b10xxxxxx 0b10xxxxxx) */
    c[0] = (utf8_int8_t)(0xe0 | (utf8_int8_t)(chr >> 12));
    c[1] = (utf8_int8_t)(0x80 | (utf8_int8_t)((chr >> 6) & 0x3f));
    c[2] = (utf8_int8_t)(0x80 | (utf8_int8_t)(chr & 0x3f));
  } else { /* if (0 == ((int)0xffe00000 & chr)) { */
    /* 4-byte/21-bit utf8 code point
     * (0b11110xxx 0b10xxxxxx 0b10xxxxxx 0b10xxxxxx) */
    c[0] = (utf8_int8_t)(0xf0 | (utf8_int8_t)(chr >> 18));
    c[1] = (utf8_int8_t)(0x80 | (utf8_int8_t)((chr >> 12) & 0x3f));
    c[2] = (utf8_int8_t)(0x80 | (utf8_int8_t)((chr >> 6) & 0x3f));
    c[3] = (utf8_int8_t)(0x80 | (utf8_int8_t)(chr & 0x3f));
  }

  /* we've created a 2 utf8 codepoint string in c that is
   * the utf8 character asked for by chr, and a null
   * terminating byte */

  while ('\0' != *src) {
    size_t offset = 0;

    while (src[offset] == c[offset]) {
      offset++;
    }

    if ('\0' == c[offset]) {
      /* we found a matching utf8 code point */
      match = (utf8_int8_t *)src;
      src += offset;
    } else {
      src += offset;

      /* need to march s along to next utf8 codepoint start
       * (the next byte that doesn't match 0b10xxxxxx) */
      if ('\0' != *src) {
        do {
          src++;
        } while (0x80 == (0xc0 & *src));
      }
    }
  }

  /* return the last match we found (or 0 if no match was found) */
  return match;
}

utf8_constexpr14_impl utf8_int8_t *utf8pbrk(const utf8_int8_t *str,
                                            const utf8_int8_t *accept) {
  while ('\0' != *str) {
    const utf8_int8_t *a = accept;
    size_t offset = 0;

    while ('\0' != *a) {
      /* checking that if *a is the start of a utf8 codepoint
       * (it is not 0b10xxxxxx) and we have successfully matched
       * a previous character (0 < offset) - we found a match */
      if ((0x80 != (0xc0 & *a)) && (0 < offset)) {
        return (utf8_int8_t *)str;
      } else {
        if (*a == str[offset]) {
          /* part of a utf8 codepoint matched, so move our checking
           * onwards to the next byte */
          offset++;
          a++;
        } else {
          /* r could be in the middle of an unmatching utf8 code point,
           * so we need to march it on to the next character beginning, */

          do {
            a++;
          } while (0x80 == (0xc0 & *a));

          /* reset offset too as we found a mismatch */
          offset = 0;
        }
      }
    }

    /* we found a match on the last utf8 codepoint */
    if (0 < offset) {
      return (utf8_int8_t *)str;
    }

    /* the current utf8 codepoint in src did not match accept, but src
     * could have been partway through a utf8 codepoint, so we need to
     * march it onto the next utf8 codepoint starting byte */
    do {
      str++;
    } while ((0x80 == (0xc0 & *str)));
  }

  return utf8_null;
}

utf8_constexpr14_impl size_t utf8size(const utf8_int8_t *str) {
  return utf8size_lazy(str) + 1;
}

utf8_constexpr14_impl size_t utf8size_lazy(const utf8_int8_t *str) {
  return utf8nsize_lazy(str, SIZE_MAX);
}

utf8_constexpr14_impl size_t utf8nsize_lazy(const utf8_int8_t *str, size_t n) {
  size_t size = 0;
  while (size < n && '\0' != str[size]) {
    size++;
  }
  return size;
}

utf8_constexpr14_impl size_t utf8spn(const utf8_int8_t *src,
                                     const utf8_int8_t *accept) {
  size_t chars = 0;

  while ('\0' != *src) {
    const utf8_int8_t *a = accept;
    size_t offset = 0;

    while ('\0' != *a) {
      /* checking that if *r is the start of a utf8 codepoint
       * (it is not 0b10xxxxxx) and we have successfully matched
       * a previous character (0 < offset) - we found a match */
      if ((0x80 != (0xc0 & *a)) && (0 < offset)) {
        /* found a match, so increment the number of utf8 codepoints
         * that have matched and stop checking whether any other utf8
         * codepoints in a match */
        chars++;
        src += offset;
        offset = 0;
        break;
      } else {
        if (*a == src[offset]) {
          offset++;
          a++;
        } else {
          /* a could be in the middle of an unmatching utf8 codepoint,
           * so we need to march it on to the next character beginning, */
          do {
            a++;
          } while (0x80 == (0xc0 & *a));

          /* reset offset too as we found a mismatch */
          offset = 0;
        }
      }
    }

    /* found a match at the end of *a, so didn't get a chance to test it */
    if (0 < offset) {
      chars++;
      src += offset;
      continue;
    }

    /* if a got to its terminating null byte, then we didn't find a match.
     * Return the current number of matched utf8 codepoints */
    if ('\0' == *a) {
      return chars;
    }
  }

  return chars;
}

utf8_constexpr14_impl utf8_int8_t *utf8str(const utf8_int8_t *haystack,
                                           const utf8_int8_t *needle) {
  utf8_int32_t throwaway_codepoint = 0;

  /* if needle has no utf8 codepoints before the null terminating
   * byte then return haystack */
  if ('\0' == *needle) {
    return (utf8_int8_t *)haystack;
  }

  while ('\0' != *haystack) {
    const utf8_int8_t *maybeMatch = haystack;
    const utf8_int8_t *n = needle;

    while (*haystack == *n && (*haystack != '\0' && *n != '\0')) {
      n++;
      haystack++;
    }

    if ('\0' == *n) {
      /* we found the whole utf8 string for needle in haystack at
       * maybeMatch, so return it */
      return (utf8_int8_t *)maybeMatch;
    } else {
      /* h could be in the middle of an unmatching utf8 codepoint,
       * so we need to march it on to the next character beginning
       * starting from the current character */
      haystack = utf8codepoint(maybeMatch, &throwaway_codepoint);
    }
  }

  /* no match */
  return utf8_null;
}

utf8_constexpr14_impl utf8_int8_t *utf8casestr(const utf8_int8_t *haystack,
                                               const utf8_int8_t *needle) {
  /* if needle has no utf8 codepoints before the null terminating
   * byte then return haystack */
  if ('\0' == *needle) {
    return (utf8_int8_t *)haystack;
  }

  for (;;) {
    const utf8_int8_t *maybeMatch = haystack;
    const utf8_int8_t *n = needle;
    utf8_int32_t h_cp = 0, n_cp = 0;

    /* Get the next code point and track it */
    const utf8_int8_t *nextH = haystack = utf8codepoint(haystack, &h_cp);
    n = utf8codepoint(n, &n_cp);

    while ((0 != h_cp) && (0 != n_cp)) {
      h_cp = utf8lwrcodepoint(h_cp);
      n_cp = utf8lwrcodepoint(n_cp);

      /* if we find a mismatch, bail out! */
      if (h_cp != n_cp) {
        break;
      }

      haystack = utf8codepoint(haystack, &h_cp);
      n = utf8codepoint(n, &n_cp);
    }

    if (0 == n_cp) {
      /* we found the whole utf8 string for needle in haystack at
       * maybeMatch, so return it */
      return (utf8_int8_t *)maybeMatch;
    }

    if (0 == h_cp) {
      /* no match */
      return utf8_null;
    }

    /* Roll back to the next code point in the haystack to test */
    haystack = nextH;
  }
}

utf8_constexpr14_impl utf8_int8_t *utf8valid(const utf8_int8_t *str) {
  return utf8nvalid(str, SIZE_MAX);
}

utf8_constexpr14_impl utf8_int8_t *utf8nvalid(const utf8_int8_t *str,
                                              size_t n) {
  const utf8_int8_t *t = str;
  size_t consumed = 0, remained = 0;

  while ((void)(consumed = (size_t)(str - t)), consumed < n && '\0' != *str) {
    remained = n - consumed;

    if (0xf0 == (0xf8 & *str)) {
      /* ensure that there's 4 bytes or more remained */
      if (remained < 4) {
        return (utf8_int8_t *)str;
      }

      /* ensure each of the 3 following bytes in this 4-byte
       * utf8 codepoint began with 0b10xxxxxx */
      if ((0x80 != (0xc0 & str[1])) || (0x80 != (0xc0 & str[2])) ||
          (0x80 != (0xc0 & str[3]))) {
        return (utf8_int8_t *)str;
      }

      /* ensure that our utf8 codepoint ended after 4 bytes */
      if (0x80 == (0xc0 & str[4])) {
        return (utf8_int8_t *)str;
      }

      /* ensure that the top 5 bits of this 4-byte utf8
       * codepoint were not 0, as then we could have used
       * one of the smaller encodings */
      if ((0 == (0x07 & str[0])) && (0 == (0x30 & str[1]))) {
        return (utf8_int8_t *)str;
      }

      /* 4-byte utf8 code point (began with 0b11110xxx) */
      str += 4;
    } else if (0xe0 == (0xf0 & *str)) {
      /* ensure that there's 3 bytes or more remained */
      if (remained < 3) {
        return (utf8_int8_t *)str;
      }

      /* ensure each of the 2 following bytes in this 3-byte
       * utf8 codepoint began with 0b10xxxxxx */
      if ((0x80 != (0xc0 & str[1])) || (0x80 != (0xc0 & str[2]))) {
        return (utf8_int8_t *)str;
      }

      /* ensure that our utf8 codepoint ended after 3 bytes */
      if (0x80 == (0xc0 & str[3])) {
        return (utf8_int8_t *)str;
      }

      /* ensure that the top 5 bits of this 3-byte utf8
       * codepoint were not 0, as then we could have used
       * one of the smaller encodings */
      if ((0 == (0x0f & str[0])) && (0 == (0x20 & str[1]))) {
        return (utf8_int8_t *)str;
      }

      /* 3-byte utf8 code point (began with 0b1110xxxx) */
      str += 3;
    } else if (0xc0 == (0xe0 & *str)) {
      /* ensure that there's 2 bytes or more remained */
      if (remained < 2) {
        return (utf8_int8_t *)str;
      }

      /* ensure the 1 following byte in this 2-byte
       * utf8 codepoint began with 0b10xxxxxx */
      if (0x80 != (0xc0 & str[1])) {
        return (utf8_int8_t *)str;
      }

      /* ensure that our utf8 codepoint ended after 2 bytes */
      if (0x80 == (0xc0 & str[2])) {
        return (utf8_int8_t *)str;
      }

      /* ensure that the top 4 bits of this 2-byte utf8
       * codepoint were not 0, as then we could have used
       * one of the smaller encodings */
      if (0 == (0x1e & str[0])) {
        return (utf8_int8_t *)str;
      }

      /* 2-byte utf8 code point (began with 0b110xxxxx) */
      str += 2;
    } else if (0x00 == (0x80 & *str)) {
      /* 1-byte ascii (began with 0b0xxxxxxx) */
      str += 1;
    } else {
      /* we have an invalid 0b1xxxxxxx utf8 code point entry */
      return (utf8_int8_t *)str;
    }
  }

  return utf8_null;
}

int utf8makevalid(utf8_int8_t *str, const utf8_int32_t replacement) {
  utf8_int8_t *read = str;
  utf8_int8_t *write = read;
  const utf8_int8_t r = (utf8_int8_t)replacement;
  utf8_int32_t codepoint = 0;

  if (replacement > 0x7f) {
    return -1;
  }

  while ('\0' != *read) {
    if (0xf0 == (0xf8 & *read)) {
      /* ensure each of the 3 following bytes in this 4-byte
       * utf8 codepoint began with 0b10xxxxxx */
      if ((0x80 != (0xc0 & read[1])) || (0x80 != (0xc0 & read[2])) ||
          (0x80 != (0xc0 & read[3]))) {
        *write++ = r;
        read++;
        continue;
      }

      /* 4-byte utf8 code point (began with 0b11110xxx) */
      read = utf8codepoint(read, &codepoint);
      write = utf8catcodepoint(write, codepoint, 4);
    } else if (0xe0 == (0xf0 & *read)) {
      /* ensure each of the 2 following bytes in this 3-byte
       * utf8 codepoint began with 0b10xxxxxx */
      if ((0x80 != (0xc0 & read[1])) || (0x80 != (0xc0 & read[2]))) {
        *write++ = r;
        read++;
        continue;
      }

      /* 3-byte utf8 code point (began with 0b1110xxxx) */
      read = utf8codepoint(read, &codepoint);
      write = utf8catcodepoint(write, codepoint, 3);
    } else if (0xc0 == (0xe0 & *read)) {
      /* ensure the 1 following byte in this 2-byte
       * utf8 codepoint began with 0b10xxxxxx */
      if (0x80 != (0xc0 & read[1])) {
        *write++ = r;
        read++;
        continue;
      }

      /* 2-byte utf8 code point (began with 0b110xxxxx) */
      read = utf8codepoint(read, &codepoint);
      write = utf8catcodepoint(write, codepoint, 2);
    } else if (0x00 == (0x80 & *read)) {
      /* 1-byte ascii (began with 0b0xxxxxxx) */
      read = utf8codepoint(read, &codepoint);
      write = utf8catcodepoint(write, codepoint, 1);
    } else {
      /* if we got here then we've got a dangling continuation (0b10xxxxxx) */
      *write++ = r;
      read++;
      continue;
    }
  }

  *write = '\0';

  return 0;
}

utf8_constexpr14_impl utf8_int8_t *
utf8codepoint(const utf8_int8_t *utf8_restrict str,
              utf8_int32_t *utf8_restrict out_codepoint) {
  if (0xf0 == (0xf8 & str[0])) {
    /* 4 byte utf8 codepoint */
    *out_codepoint = ((0x07 & str[0]) << 18) | ((0x3f & str[1]) << 12) |
                     ((0x3f & str[2]) << 6) | (0x3f & str[3]);
    str += 4;
  } else if (0xe0 == (0xf0 & str[0])) {
    /* 3 byte utf8 codepoint */
    *out_codepoint =
        ((0x0f & str[0]) << 12) | ((0x3f & str[1]) << 6) | (0x3f & str[2]);
    str += 3;
  } else if (0xc0 == (0xe0 & str[0])) {
    /* 2 byte utf8 codepoint */
    *out_codepoint = ((0x1f & str[0]) << 6) | (0x3f & str[1]);
    str += 2;
  } else {
    /* 1 byte utf8 codepoint otherwise */
    *out_codepoint = str[0];
    str += 1;
  }

  return (utf8_int8_t *)str;
}

utf8_constexpr14_impl size_t utf8codepointcalcsize(const utf8_int8_t *str) {
  if (0xf0 == (0xf8 & str[0])) {
    /* 4 byte utf8 codepoint */
    return 4;
  } else if (0xe0 == (0xf0 & str[0])) {
    /* 3 byte utf8 codepoint */
    return 3;
  } else if (0xc0 == (0xe0 & str[0])) {
    /* 2 byte utf8 codepoint */
    return 2;
  }

  /* 1 byte utf8 codepoint otherwise */
  return 1;
}

utf8_constexpr14_impl size_t utf8codepointsize(utf8_int32_t chr) {
  if (0 == ((utf8_int32_t)0xffffff80 & chr)) {
    return 1;
  } else if (0 == ((utf8_int32_t)0xfffff800 & chr)) {
    return 2;
  } else if (0 == ((utf8_int32_t)0xffff0000 & chr)) {
    return 3;
  } else { /* if (0 == ((int)0xffe00000 & chr)) { */
    return 4;
  }
}

utf8_int8_t *utf8catcodepoint(utf8_int8_t *str, utf8_int32_t chr, size_t n) {
  if (0 == ((utf8_int32_t)0xffffff80 & chr)) {
    /* 1-byte/7-bit ascii
     * (0b0xxxxxxx) */
    if (n < 1) {
      return utf8_null;
    }
    str[0] = (utf8_int8_t)chr;
    str += 1;
  } else if (0 == ((utf8_int32_t)0xfffff800 & chr)) {
    /* 2-byte/11-bit utf8 code point
     * (0b110xxxxx 0b10xxxxxx) */
    if (n < 2) {
      return utf8_null;
    }
    str[0] = (utf8_int8_t)(0xc0 | (utf8_int8_t)((chr >> 6) & 0x1f));
    str[1] = (utf8_int8_t)(0x80 | (utf8_int8_t)(chr & 0x3f));
    str += 2;
  } else if (0 == ((utf8_int32_t)0xffff0000 & chr)) {
    /* 3-byte/16-bit utf8 code point
     * (0b1110xxxx 0b10xxxxxx 0b10xxxxxx) */
    if (n < 3) {
      return utf8_null;
    }
    str[0] = (utf8_int8_t)(0xe0 | (utf8_int8_t)((chr >> 12) & 0x0f));
    str[1] = (utf8_int8_t)(0x80 | (utf8_int8_t)((chr >> 6) & 0x3f));
    str[2] = (utf8_int8_t)(0x80 | (utf8_int8_t)(chr & 0x3f));
    str += 3;
  } else { /* if (0 == ((int)0xffe00000 & chr)) { */
    /* 4-byte/21-bit utf8 code point
     * (0b11110xxx 0b10xxxxxx 0b10xxxxxx 0b10xxxxxx) */
    if (n < 4) {
      return utf8_null;
    }
    str[0] = (utf8_int8_t)(0xf0 | (utf8_int8_t)((chr >> 18) & 0x07));
    str[1] = (utf8_int8_t)(0x80 | (utf8_int8_t)((chr >> 12) & 0x3f));
    str[2] = (utf8_int8_t)(0x80 | (utf8_int8_t)((chr >> 6) & 0x3f));
    str[3] = (utf8_int8_t)(0x80 | (utf8_int8_t)(chr & 0x3f));
    str += 4;
  }

  return str;
}

utf8_constexpr14_impl int utf8islower(utf8_int32_t chr) {
  return chr != utf8uprcodepoint(chr);
}

utf8_constexpr14_impl int utf8isupper(utf8_int32_t chr) {
  return chr != utf8lwrcodepoint(chr);
}

void utf8lwr(utf8_int8_t *utf8_restrict str) {
  utf8_int32_t cp = 0;
  utf8_int8_t *pn = utf8codepoint(str, &cp);

  while (cp != 0) {
    const utf8_int32_t lwr_cp = utf8lwrcodepoint(cp);
    const size_t size = utf8codepointsize(lwr_cp);

    if (lwr_cp != cp) {
      utf8catcodepoint(str, lwr_cp, size);
    }

    str = pn;
    pn = utf8codepoint(str, &cp);
  }
}

void utf8upr(utf8_int8_t *utf8_restrict str) {
  utf8_int32_t cp = 0;
  utf8_int8_t *pn = utf8codepoint(str, &cp);

  while (cp != 0) {
    const utf8_int32_t lwr_cp = utf8uprcodepoint(cp);
    const size_t size = utf8codepointsize(lwr_cp);

    if (lwr_cp != cp) {
      utf8catcodepoint(str, lwr_cp, size);
    }

    str = pn;
    pn = utf8codepoint(str, &cp);
  }
}

utf8_constexpr14_impl utf8_int32_t utf8lwrcodepoint(utf8_int32_t cp) {
  if (((0x0041 <= cp) && (0x005a >= cp)) ||
      ((0x00c0 <= cp) && (0x00d6 >= cp)) ||
      ((0x00d8 <= cp) && (0x00de >= cp)) ||
      ((0x0391 <= cp) && (0x03a1 >= cp)) ||
      ((0x03a3 <= cp) && (0x03ab >= cp)) ||
      ((0x0410 <= cp) && (0x042f >= cp))) {
    cp += 32;
  } else if ((0x0400 <= cp) && (0x040f >= cp)) {
    cp += 80;
  } else if (((0x0100 <= cp) && (0x012f >= cp)) ||
             ((0x0132 <= cp) && (0x0137 >= cp)) ||
             ((0x014a <= cp) && (0x0177 >= cp)) ||
             ((0x0182 <= cp) && (0x0185 >= cp)) ||
             ((0x01a0 <= cp) && (0x01a5 >= cp)) ||
             ((0x01de <= cp) && (0x01ef >= cp)) ||
             ((0x01f8 <= cp) && (0x021f >= cp)) ||
             ((0x0222 <= cp) && (0x0233 >= cp)) ||
             ((0x0246 <= cp) && (0x024f >= cp)) ||
             ((0x03d8 <= cp) && (0x03ef >= cp)) ||
             ((0x0460 <= cp) && (0x0481 >= cp)) ||
             ((0x048a <= cp) && (0x04ff >= cp))) {
    cp |= 0x1;
  } else if (((0x0139 <= cp) && (0x0148 >= cp)) ||
             ((0x0179 <= cp) && (0x017e >= cp)) ||
             ((0x01af <= cp) && (0x01b0 >= cp)) ||
             ((0x01b3 <= cp) && (0x01b6 >= cp)) ||
             ((0x01cd <= cp) && (0x01dc >= cp))) {
    cp += 1;
    cp &= ~0x1;
  } else {
    switch (cp) {
    default:
      break;
    case 0x0178:
      cp = 0x00ff;
      break;
    case 0x0243:
      cp = 0x0180;
      break;
    case 0x018e:
      cp = 0x01dd;
      break;
    case 0x023d:
      cp = 0x019a;
      break;
    case 0x0220:
      cp = 0x019e;
      break;
    case 0x01b7:
      cp = 0x0292;
      break;
    case 0x01c4:
      cp = 0x01c6;
      break;
    case 0x01c7:
      cp = 0x01c9;
      break;
    case 0x01ca:
      cp = 0x01cc;
      break;
    case 0x01f1:
      cp = 0x01f3;
      break;
    case 0x01f7:
      cp = 0x01bf;
      break;
    case 0x0187:
      cp = 0x0188;
      break;
    case 0x018b:
      cp = 0x018c;
      break;
    case 0x0191:
      cp = 0x0192;
      break;
    case 0x0198:
      cp = 0x0199;
      break;
    case 0x01a7:
      cp = 0x01a8;
      break;
    case 0x01ac:
      cp = 0x01ad;
      break;
    case 0x01af:
      cp = 0x01b0;
      break;
    case 0x01b8:
      cp = 0x01b9;
      break;
    case 0x01bc:
      cp = 0x01bd;
      break;
    case 0x01f4:
      cp = 0x01f5;
      break;
    case 0x023b:
      cp = 0x023c;
      break;
    case 0x0241:
      cp = 0x0242;
      break;
    case 0x03fd:
      cp = 0x037b;
      break;
    case 0x03fe:
      cp = 0x037c;
      break;
    case 0x03ff:
      cp = 0x037d;
      break;
    case 0x037f:
      cp = 0x03f3;
      break;
    case 0x0386:
      cp = 0x03ac;
      break;
    case 0x0388:
      cp = 0x03ad;
      break;
    case 0x0389:
      cp = 0x03ae;
      break;
    case 0x038a:
      cp = 0x03af;
      break;
    case 0x038c:
      cp = 0x03cc;
      break;
    case 0x038e:
      cp = 0x03cd;
      break;
    case 0x038f:
      cp = 0x03ce;
      break;
    case 0x0370:
      cp = 0x0371;
      break;
    case 0x0372:
      cp = 0x0373;
      break;
    case 0x0376:
      cp = 0x0377;
      break;
    case 0x03f4:
      cp = 0x03b8;
      break;
    case 0x03cf:
      cp = 0x03d7;
      break;
    case 0x03f9:
      cp = 0x03f2;
      break;
    case 0x03f7:
      cp = 0x03f8;
      break;
    case 0x03fa:
      cp = 0x03fb;
      break;
    }
  }

  return cp;
}

utf8_constexpr14_impl utf8_int32_t utf8uprcodepoint(utf8_int32_t cp) {
  if (((0x0061 <= cp) && (0x007a >= cp)) ||
      ((0x00e0 <= cp) && (0x00f6 >= cp)) ||
      ((0x00f8 <= cp) && (0x00fe >= cp)) ||
      ((0x03b1 <= cp) && (0x03c1 >= cp)) ||
      ((0x03c3 <= cp) && (0x03cb >= cp)) ||
      ((0x0430 <= cp) && (0x044f >= cp))) {
    cp -= 32;
  } else if ((0x0450 <= cp) && (0x045f >= cp)) {
    cp -= 80;
  } else if (((0x0100 <= cp) && (0x012f >= cp)) ||
             ((0x0132 <= cp) && (0x0137 >= cp)) ||
             ((0x014a <= cp) && (0x0177 >= cp)) ||
             ((0x0182 <= cp) && (0x0185 >= cp)) ||
             ((0x01a0 <= cp) && (0x01a5 >= cp)) ||
             ((0x01de <= cp) && (0x01ef >= cp)) ||
             ((0x01f8 <= cp) && (0x021f >= cp)) ||
             ((0x0222 <= cp) && (0x0233 >= cp)) ||
             ((0x0246 <= cp) && (0x024f >= cp)) ||
             ((0x03d8 <= cp) && (0x03ef >= cp)) ||
             ((0x0460 <= cp) && (0x0481 >= cp)) ||
             ((0x048a <= cp) && (0x04ff >= cp))) {
    cp &= ~0x1;
  } else if (((0x0139 <= cp) && (0x0148 >= cp)) ||
             ((0x0179 <= cp) && (0x017e >= cp)) ||
             ((0x01af <= cp) && (0x01b0 >= cp)) ||
             ((0x01b3 <= cp) && (0x01b6 >= cp)) ||
             ((0x01cd <= cp) && (0x01dc >= cp))) {
    cp -= 1;
    cp |= 0x1;
  } else {
    switch (cp) {
    default:
      break;
    case 0x00ff:
      cp = 0x0178;
      break;
    case 0x0180:
      cp = 0x0243;
      break;
    case 0x01dd:
      cp = 0x018e;
      break;
    case 0x019a:
      cp = 0x023d;
      break;
    case 0x019e:
      cp = 0x0220;
      break;
    case 0x0292:
      cp = 0x01b7;
      break;
    case 0x01c6:
      cp = 0x01c4;
      break;
    case 0x01c9:
      cp = 0x01c7;
      break;
    case 0x01cc:
      cp = 0x01ca;
      break;
    case 0x01f3:
      cp = 0x01f1;
      break;
    case 0x01bf:
      cp = 0x01f7;
      break;
    case 0x0188:
      cp = 0x0187;
      break;
    case 0x018c:
      cp = 0x018b;
      break;
    case 0x0192:
      cp = 0x0191;
      break;
    case 0x0199:
      cp = 0x0198;
      break;
    case 0x01a8:
      cp = 0x01a7;
      break;
    case 0x01ad:
      cp = 0x01ac;
      break;
    case 0x01b0:
      cp = 0x01af;
      break;
    case 0x01b9:
      cp = 0x01b8;
      break;
    case 0x01bd:
      cp = 0x01bc;
      break;
    case 0x01f5:
      cp = 0x01f4;
      break;
    case 0x023c:
      cp = 0x023b;
      break;
    case 0x0242:
      cp = 0x0241;
      break;
    case 0x037b:
      cp = 0x03fd;
      break;
    case 0x037c:
      cp = 0x03fe;
      break;
    case 0x037d:
      cp = 0x03ff;
      break;
    case 0x03f3:
      cp = 0x037f;
      break;
    case 0x03ac:
      cp = 0x0386;
      break;
    case 0x03ad:
      cp = 0x0388;
      break;
    case 0x03ae:
      cp = 0x0389;
      break;
    case 0x03af:
      cp = 0x038a;
      break;
    case 0x03cc:
      cp = 0x038c;
      break;
    case 0x03cd:
      cp = 0x038e;
      break;
    case 0x03ce:
      cp = 0x038f;
      break;
    case 0x0371:
      cp = 0x0370;
      break;
    case 0x0373:
      cp = 0x0372;
      break;
    case 0x0377:
      cp = 0x0376;
      break;
    case 0x03d1:
      cp = 0x0398;
      break;
    case 0x03d7:
      cp = 0x03cf;
      break;
    case 0x03f2:
      cp = 0x03f9;
      break;
    case 0x03f8:
      cp = 0x03f7;
      break;
    case 0x03fb:
      cp = 0x03fa;
      break;
    }
  }

  return cp;
}

utf8_constexpr14_impl utf8_int8_t *
utf8rcodepoint(const utf8_int8_t *utf8_restrict str,
               utf8_int32_t *utf8_restrict out_codepoint) {
  const utf8_int8_t *s = (const utf8_int8_t *)str;

  if (0xf0 == (0xf8 & s[0])) {
    /* 4 byte utf8 codepoint */
    *out_codepoint = ((0x07 & s[0]) << 18) | ((0x3f & s[1]) << 12) |
                     ((0x3f & s[2]) << 6) | (0x3f & s[3]);
  } else if (0xe0 == (0xf0 & s[0])) {
    /* 3 byte utf8 codepoint */
    *out_codepoint =
        ((0x0f & s[0]) << 12) | ((0x3f & s[1]) << 6) | (0x3f & s[2]);
  } else if (0xc0 == (0xe0 & s[0])) {
    /* 2 byte utf8 codepoint */
    *out_codepoint = ((0x1f & s[0]) << 6) | (0x3f & s[1]);
  } else {
    /* 1 byte utf8 codepoint otherwise */
    *out_codepoint = s[0];
  }

  do {
    s--;
  } while ((0 != (0x80 & s[0])) && (0x80 == (0xc0 & s[0])));

  return (utf8_int8_t *)s;
}

#undef utf8_restrict
#undef utf8_constexpr14
#undef utf8_null

#ifdef __cplusplus
} /* extern "C" */
#endif

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

#endif /* SHEREDOM_UTF8_H_INCLUDED */
