// --------------------------------------------------------------------------
//
// Project       Wifi-ControlUnit
//
// File          app_gui.cpp
//
// Author        Axel Werner
//
// --------------------------------------------------------------------------
// Changelog
//
// 2023-02-21  AWe   do the setup function before the task is created
// 2019-10-01  AWe   initial version
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// debug support
// --------------------------------------------------------------------------

static const char *TAG = "app_gui";
#include "esp_log.h"

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#include "freertos/FreeRTOS.h"
#include "esp_heap_caps.h"

#include "esp_timer.h"

// Littlevgl specific
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
   #include "lvgl.h"
#else
   #include "lvgl/lvgl.h"         // LVGL header file
#endif

#include "lvgl_helpers.h"         // Assistant hardware driver related

#include "ui/ui.h"

// Include desired font here for espressif/esp-iot-solution example

// LV_IMG_DECLARE(mouse_cursor_icon);         /* Declare the image file. */

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define LV_TICK_PERIOD_MS            10

#define LINES_TO_DRAW               24
#define USE_STATIC_DISPLAY_BUFFER    0      // using static buffers will result in a crash
#define USE_DOUBLE_DISPLAY_BUFFER    0      // no double buffer saves 20KB memory

// --------------------------------------------------------------------------
// set stack size
// --------------------------------------------------------------------------

#ifndef CONFIG_TASK_STACK_SIZE_GUI
   #define TASK_STACK_SIZE_GUI            ( 6 * 1024 )
#else
   #define TASK_STACK_SIZE_GUI            CONFIG_TASK_STACK_SIZE_GUI
#endif

#ifndef CONFIG_TASK_CORE_GUI
   #define CONFIG_TASK_CORE_GUI        tskNO_AFFINITY
#endif
#ifndef CONFIG_TASK_PRIO_GUI
   #define CONFIG_TASK_PRIO_GUI        6
#endif

// --------------------------------------------------------------------------
// local functions
// --------------------------------------------------------------------------

static void lv_tick_task( void *arg );
static void appGuiTask( void *pvParameter );

// --------------------------------------------------------------------------
//  static variables
// --------------------------------------------------------------------------

DMA_ATTR uint8_t* disp_buf1 =  NULL;
DMA_ATTR uint8_t* disp_buf2 =  NULL;

// --------------------------------------------------------------------------
// gui flushing
// --------------------------------------------------------------------------

#define RGB16(r, g, b ) ( ( ( r & 0xf8 ) << 8 ) | \
                          ( ( g & 0xfc ) << 3 ) | \
                          ( ( b & 0xf8 ) >> 3 ) )

/*
 * from  ...\lvgl\src\display\lv_display.h
 *    @note To change the endianness of the rendered image in case of RGB565 format
 *      (i.e. swap the 2 bytes) call `lv_draw_sw_rgb565_swap` in the flush_cb
 *
 * found lv_draw_sw_rgb565_swap() in ...\lvgl\src\draw\sw\lv_draw_sw.c
 */

void my_display_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map )
{
   uint32_t w = lv_area_get_width( area );
   uint32_t h = lv_area_get_height( area );

   lv_draw_sw_rgb565_swap( px_map, w * h );

   disp_driver_flush( disp, area, px_map );
}

// --------------------------------------------------------------------------
// Read the touchpad
// --------------------------------------------------------------------------
#if CONFIG_LV_TOUCH_CONTROLLER

void my_touchpad_read( lv_indev_t * indev_driver, lv_indev_data_t * data )
{
   touch_driver_read( indev_driver, data );
}
#endif
// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
/*
   see also F:\Projects\InternetOfThings\Devices\ESP32-2432s028\Firmware\LVGL_Full_Test\source\components\lv_examples\examples\porting\lv_port_disp_template.c
      lv_port_disp_init()
*/

// Interface and driver initialization
lv_display_t *my_display_create( uint32_t hor_res, uint32_t ver_res )
{
   ESP_LOGI( TAG, "Display hor size: %d, ver size: %d", hor_res, ver_res );

   // allocate memory for display buffers
   uint32_t buf_size_bytes = hor_res * LINES_TO_DRAW * ( ( LV_COLOR_DEPTH + 7 ) / 8 );
   ESP_LOGI( TAG, "Display buffer size: %d", buf_size_bytes );

#if USE_STATIC_DISPLAY_BUFFER
   // !!! this will result in a crash
   uint8_t disp_buf1[ buf_size_bytes ];
 #if USE_DOUBLE_DISPLAY_BUFFER
   uint8_t disp_buf2[ buf_size_bytes ];
 #else
   uint8_t * disp_buf2 = NULL;
 #endif
#else
   disp_buf1 = ( uint8_t* )heap_caps_malloc( buf_size_bytes, MALLOC_CAP_DMA );
   LV_ASSERT_MSG( disp_buf1 != NULL, "Can't allocate display buffer 1" );
 #if USE_DOUBLE_DISPLAY_BUFFER
   // Use double buffered
   disp_buf2 = ( uint8_t* )heap_caps_malloc( buf_size_bytes, MALLOC_CAP_DMA );
   LV_ASSERT_MSG( disp_buf2 != NULL, "Can't allocate display buffer 2" );
 #else
   uint8_t* disp_buf2 = NULL;
 #endif
#endif

   // Create the display in LVGL. Set also the resolution of the display
   lv_display_t * disp = lv_display_create( hor_res, ver_res );
   if( disp == NULL )
   {
      return NULL;
   }

   ESP_LOGI( TAG, "Buffer addresses: 0x%08x, 0x%08x, size %d bytes ", ( void *)disp_buf1, (void *)disp_buf2, buf_size_bytes );
   lv_display_set_buffers( disp, (void *)disp_buf1, (void *)disp_buf2, buf_size_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL );

   // Used to copy the buffer's content to the display
   lv_display_set_flush_cb( disp, my_display_flush );

   return disp;
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

static void lv_tick_task( void *arg )
{
   ( void ) arg;
   lv_tick_inc( LV_TICK_PERIOD_MS );
}

// --------------------------------------------------------------------------
//  the Driver Monitors gui task
// --------------------------------------------------------------------------

// taken fom ...\lv_port_esp32\source\main\main.c

// Creates a semaphore to handle concurrent call to lvgl stuff
// If you wish to call *any* lvgl function from other threads/tasks
// you should lock on the very same semaphore!
SemaphoreHandle_t xGuiSemaphore;         // Create a GUI semaphore

static void appGuiTask( void *pvParameter )
{
   ESP_LOGI( TAG, "Start GUI Task" );
   ( void ) pvParameter;
   xGuiSemaphore = xSemaphoreCreateMutex();   // Create a GUI semaphore

   // Initialize LittlevGL
   lv_init();

   // Create a display and set a flush_cb
   ESP_LOGI( TAG, "Display initialization" );
   lv_display_t * disp = my_display_create( LV_HOR_RES_MAX, LV_VER_RES_MAX );

   // Initialize LCD SPI driver and touch chip SPI/IIC driver
   ESP_LOGI( TAG, "LVGL Driver initialization" );
   lvgl_driver_init();

#if CONFIG_LV_TOUCH_CONTROLLER
   ESP_LOGI( TAG, "setup Touch controller" );

   // Register an input device when enabled on the menuconfig
   lv_indev_t * indev_touchpad = lv_indev_create();
   lv_indev_set_type( indev_touchpad, LV_INDEV_TYPE_POINTER ); // Touchpad should have POINTER type
   // after calibration the callback function is replaced in lv_tc_indev_init()

   lv_indev_set_read_cb( indev_touchpad, my_touchpad_read );
   lv_indev_set_driver_data( indev_touchpad, ( void * )lv_display_get_driver_data( disp ) );
#endif
   // Create and start a periodic timer interrupt to call lv_tick_inc
   const esp_timer_create_args_t periodic_timer_args =
   {
      .callback = &lv_tick_task,
      .name = "periodic_gui"
   };
   esp_timer_handle_t periodic_timer;
   ESP_ERROR_CHECK( esp_timer_create( &periodic_timer_args, &periodic_timer ) );
   ESP_ERROR_CHECK( esp_timer_start_periodic( periodic_timer, LV_TICK_PERIOD_MS * 1000 ) );

//   app_backlight_setup( LCD_BL_Pin );

   /* Create the gui application */
   ESP_LOGI( TAG, "Create the gui application" );
   ui_init();

   // --------------------------------------------------------------------------
   // the task loop
   // --------------------------------------------------------------------------

   ESP_LOGD( TAG, "Run gui task ..." );

   while( true )
   {
      vTaskDelay( 1 ); // give other tasks a chance to run

      // Try to take the semaphore, call lvgl related function on success
      if( pdTRUE == xSemaphoreTake( xGuiSemaphore, portMAX_DELAY ) )
      {
         ui_tick();
         lv_timer_handler(); // let the GUI do its work
         xSemaphoreGive( xGuiSemaphore );
      }

//      app_backlight_loop();
   }

   // A task should NEVER return
   free(disp_buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
   free(disp_buf2);
#endif
   vTaskDelete( NULL );
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

extern "C"
void appGui( void )
{
   ESP_LOGI( TAG, "appGuiTask" );

   // gui task init
   xTaskCreatePinnedToCore( appGuiTask, "appGuiTask",  TASK_STACK_SIZE_GUI, NULL, CONFIG_TASK_PRIO_GUI, NULL, CONFIG_TASK_CORE_GUI );
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
