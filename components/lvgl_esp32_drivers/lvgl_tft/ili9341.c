/**
 * @file ili9341.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "ili9341.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/
 #define TAG "ILI9341"

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ili9341_set_orientation(uint8_t orientation);

static void ili9341_send_cmd(uint8_t cmd);
static void ili9341_send_data(void * data, uint16_t length);
static void ili9341_send_color(void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ili9341_init(void)
{
#if 1
	// from ESP32_2432s028 - 5_35_LVGL_Full_Test-S024
	lcd_init_cmd_t ili_init_cmds[]={
		{0xCF, {0x00, 0x83, 0X30}, 3},
		{0xED, {0x64, 0x03, 0X12, 0X81}, 4},
		{0xE8, {0x85, 0x01, 0x79}, 3},
		{0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
		{0xF7, {0x20}, 1},
		{0xEA, {0x00, 0x00}, 2},
		{0xC0, {0x26}, 1},          /*Power control*/
		{0xC1, {0x11}, 1},          /*Power control */
		{0xC5, {0x35, 0x3E}, 2},    /*VCOM control*/
		{0xC7, {0xBE}, 1},          /*VCOM control*/
		{0x36, {0x28}, 1},          /*Memory Access Control*/
		{0x3A, {0x55}, 1},          /*Pixel Format Set*/
		{0xB1, {0x00, 0x1B}, 2},
		{0xF2, {0x08}, 1},
		{0x26, {0x01}, 1},
		{0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
		{0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
		{0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
		{0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
		{0x2C, {0}, 0},
		{0xB7, {0x07}, 1},
		{0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
		{0x11, {0}, 0x80},
		{0x29, {0}, 0x80},
		{0, {0}, 0xff},
	};
#elif 0
	lcd_init_cmd_t ili_init_cmds[]={
		// from espressif\esp-bsp
		// espressif\esp-idf\examples\peripherals\spi_master\lcd\main\spi_master_example_main.c

		// { 0xCF, {0x03, 0x80, 0x02}, 3 },
		/*Power control B, power control = 0, DC_ENA = 1 */
		{0xCF, {0x00, 0xc1, 0X30}, 3},
		/*Power on sequence control,
		 * cp1 keeps 1 frame, 1st frame enable
		 * vcl = 0, ddvdh=3, vgh=1, vgl=2
		 * DDVDH_ENH=1
		 */
		{0xED, {0x64, 0x03, 0X12, 0X81}, 4},
		/*Driver timing control A,
		 * non-overlap=default +1
		 * EQ=default - 1, CR=default
		 * pre-charge=default - 1
		 */
		{0xE8, {0x85, 0x00, 0x78}, 3},
		/*Power control A, Vcore=1.6V, DDVDH=5.6V */
		{0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
		/*Pump ratio control, DDVDH=2xVCl */
		{0xF7, {0x20}, 1},
		/*Driver timing control, all=0 unit */
		{0xEA, {0x00, 0x00}, 2},
		/*Power control 1, GVDD=4.75V */
		{0xC0, {0x23}, 1},
		/*Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
		{0xC1, {0x10}, 1},
		/*VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
		{0xC5, {0x3e, 0x28}, 2},
		/*VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
		{0xC7, {0x86}, 1},
		/*Memory access control, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
		{0x36, {0x28}, 1},
		/*Pixel format, 16bits/pixel for RGB/MCU interface */
		{0x3A, {0x55}, 1},
		/*Frame rate control, f=fosc, 70Hz fps */
		{0xB1, {0x00, 0x1B}, 2},
		/*Enable 3G, disabled */
		{0xF2, {0x08}, 1},
		/*Gamma set, curve 1 */
		{0x26, {0x01}, 1},
		/*Positive gamma correction */
		{0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
		/*Negative gamma correction */
		{0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
		/*Column address set, SC=0, EC=0xEF */
		{0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
		/*Page address set, SP=0, EP=0x013F */
		{0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
		/*Memory write */
		{0x2C, {0}, 0},
		/*Entry mode set, Low vol detect disabled, normal display */
		{0xB7, {0x07}, 1},
		/*Display function control */
		{0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
		/*Sleep out */
		{0x11, {0}, 0x80},
		/*Display on */
		{0x29, {0}, 0x80},
		{0, {0}, 0xff},
	};
#elif 1
	// from Bodmer TFT_eSPI

	lcd_init_cmd_t ili_init_cmds[]={
		/*Power control B, power control = 0, DC_ENA = 1 */
		{0xCF, {0x03, 0x80, 0x02}, 3 },
		{0xCF, {0x00, 0xc1, 0X30}, 3},
		/*Power on sequence control,
		 * cp1 keeps 1 frame, 1st frame enable
		 * vcl = 0, ddvdh=3, vgh=1, vgl=2
		 * DDVDH_ENH=1
		 */
		{0xED, {0x64, 0x03, 0X12, 0X81}, 4},
		/*Driver timing control A,
		 * non-overlap=default +1
		 * EQ=default - 1, CR=default
		 * pre-charge=default - 1
		 */
		{0xE8, {0x85, 0x00, 0x78}, 3},
		/*Power control A, Vcore=1.6V, DDVDH=5.6V */
		{0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
		/*Pump ratio control, DDVDH=2xVCl */
		{0xF7, {0x20}, 1},
		/*Driver timing control, all=0 unit */
		{0xEA, {0x00, 0x00}, 2},
		/*Power control 1, GVDD=4.75V */
		{0xC0, {0x23}, 1},
		/*Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
		{0xC1, {0x10}, 1},
		/*VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
		{0xC5, {0x3e, 0x28}, 2},
		/*VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
		{0xC7, {0x86}, 1},
		/*Memory access control, MX=MY=1, MV=0, ML=0, BGR=0, MH=0 */
		{0x36, {0x40}, 1},
		// /* Vertical scroll zero */
		// {0x37, {0x00}, 1},
		/*Pixel format, 16bits/pixel for RGB/MCU interface */
		{0x3A, {0x55}, 1},
		/*Frame rate control, f=fosc, 0x18 79Hz, 0x1B default 70Hz, 0x13 100Hz */
		{0xB1, {0x00, 0x13}, 2},
		/*Display function control */
		{0xB6, {0x08, 0x82, 0x27}, 3},
		/*Enable 3G, disabled */
		{0xF2, {0x00}, 1},
		/*Gamma set, curve 1 */
		{0x26, {0x01}, 1},
		/*Positive gamma correction */
	//	{0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
	//	{0xE0, {0x1F, 0x24, 0x24, 0x0D, 0x12, 0x09, 0x52, 0XB7, 0x3F, 0x0C, 0x15, 0x06, 0x0E, 0x08, 0x00}, 15},
		{0XE1, {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 15},
		/*Negative gamma correction */
	//	{0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
	//	{0XE1, {0x00, 0x1B, 0x1B, 0x02, 0x0E, 0x06, 0x2E, 0x48, 0x3F, 0x03, 0x0A, 0x09, 0x31, 0x37, 0x1F}, 15},
		{0XE1, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 15},
	//	/*Column address set, SC=0, EC=0xEF */
	//	{0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
	//	/*Page address set, SP=0, EP=0x013F */
	//	{0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
	//	/*Memory write */
	//	{0x2C, {0}, 0},
	//	/*Entry mode set, Low vol detect disabled, normal display */
	//	{0xB7, {0x07}, 1},
		/*Sleep out */
		{0x11, {0}, 0x80},
		/*Display on */
		{0x29, {0}, 0x80},
		{0, {0}, 0xff},
	};
#else
	//

	lcd_init_cmd_t ili_init_cmds[]={
		/*Power contorl B, power control = 0, DC_ENA = 1 */
		{0xCF, {0x00, 0x83, 0X30}, 3},
		/*Power on sequence control,
		 * cp1 keeps 1 frame, 1st frame enable
		 * vcl = 0, ddvdh=3, vgh=1, vgl=2
		 * DDVDH_ENH=1
		 */
		{0xED, {0x64, 0x03, 0X12, 0X81}, 4},
		/*Driver timing control A,
		 * non-overlap=default +1
		 * EQ=default - 1, CR=default
		 * pre-charge=default - 1
		 */
		{0xE8, {0x85, 0x01, 0x79}, 3},
		/*Power control A, Vcore=1.6V, DDVDH=5.6V */
		{0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
		/*Pump ratio control, DDVDH=2xVCl */
		{0xF7, {0x20}, 1},
		/*Driver timing control, all=0 unit */
		{0xEA, {0x00, 0x00}, 2},
		/*Power control 1, GVDD=4.75V */
		{0xC0, {0x26}, 1},
		/*Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
		{0xC1, {0x11}, 1},
		/*VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
		{0xC5, {0x35, 0x3E}, 2},
		/*VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
		{0xC7, {0xBE}, 1},
		/*Memory access control, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
		{0x36, {0xc0}, 1},
		/*Pixel format, 16bits/pixel for RGB/MCU interface */
		{0x3A, {0x55}, 1},
		/*Frame rate control, f=fosc, 70Hz fps */
		{0xB1, {0x00, 0x13}, 2},
		/*Display function control */
		{0xB6, {0x08, 0x82, 0x27, 0x00}, 4},
		/*Enable 3G, disabled */
		{0xF2, {0x00}, 1},
		/*Gamma set, curve 1 */
		{0x26, {0x01}, 1},
		/*Positive gamma correction */
		{0xE0, {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 15},
		/*Negative gamma correction */
		{0XE1, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 15},

		// /*Column address set, SC=0, EC=0xEF */
		// {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
		// /*Page address set, SP=0, EP=0x013F */
		// {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
		// /*Memory write */
		// {0x2C, {0}, 0},
		// /*Entry mode set, Low vol detect disabled, normal display */
		// {0xB7, {0x07}, 1},
		/*Sleep out */
		{0x11, {0}, 0x80},
		/*Display on */
		{0x29, {0}, 0x80},
		{0, {0}, 0xff},
	};
#endif
	//Initialize non-SPI GPIOs
    esp_rom_gpio_pad_select_gpio(ILI9341_DC);
	gpio_set_direction(ILI9341_DC, GPIO_MODE_OUTPUT);

#if ILI9341_USE_RST
    esp_rom_gpio_pad_select_gpio(ILI9341_RST);
	gpio_set_direction(ILI9341_RST, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(ILI9341_RST, 0);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	gpio_set_level(ILI9341_RST, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);
#endif

	ESP_LOGI(TAG, "Initialization.");

	//Send all the commands
	uint16_t cmd = 0;
	while (ili_init_cmds[cmd].databytes!=0xff) {
		ili9341_send_cmd(ili_init_cmds[cmd].cmd);
		ili9341_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
		if (ili_init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
		cmd++;
	}

    ili9341_set_orientation(CONFIG_LV_DISPLAY_ORIENTATION);

#if ILI9341_INVERT_COLORS == 1
	ili9341_send_cmd(0x21);
#else
	ili9341_send_cmd(0x20);
#endif
}


void ili9341_flush(lv_display_t * drv, const lv_area_t * area, uint8_t *color_map)
{
	uint8_t data[4];

	/*Column addresses*/
	ili9341_send_cmd(0x2A);
	data[0] = (area->x1 >> 8) & 0xFF;
	data[1] = area->x1 & 0xFF;
	data[2] = (area->x2 >> 8) & 0xFF;
	data[3] = area->x2 & 0xFF;
	ili9341_send_data(data, 4);

	/*Page addresses*/
	ili9341_send_cmd(0x2B);
	data[0] = (area->y1 >> 8) & 0xFF;
	data[1] = area->y1 & 0xFF;
	data[2] = (area->y2 >> 8) & 0xFF;
	data[3] = area->y2 & 0xFF;
	ili9341_send_data(data, 4);

	/*Memory write*/
	ili9341_send_cmd(0x2C);
	uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

#if ILI9341_COLOR_16_SWAP
    uint16_t *color_p = (uint16_t *)color_map;
    for (int i = 0; i < size; i++) {
        color_p[i] = (color_p[i] >> 8) | (color_p[i] << 8);
    }
#endif
	ili9341_send_color((void*)color_map, size * 2);
}

void ili9341_sleep_in()
{
	uint8_t data[] = {0x08};
	ili9341_send_cmd(0x10);
	ili9341_send_data(&data, 1);
}

void ili9341_sleep_out()
{
	uint8_t data[] = {0x08};
	ili9341_send_cmd(0x11);
	ili9341_send_data(&data, 1);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


static void ili9341_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ILI9341_DC, 0);	 /*Command mode*/
    disp_spi_send_data(&cmd, 1);
}

static void ili9341_send_data(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ILI9341_DC, 1);	 /*Data mode*/
    disp_spi_send_data(data, length);
}

static void ili9341_send_color(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ILI9341_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
}

static void ili9341_set_orientation(uint8_t orientation)
{
    // ESP_ASSERT(orientation < 4);

    const char *orientation_str[] = {
        "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"
    };

    ESP_LOGI(TAG, "Display orientation: %s", orientation_str[orientation]);

#if defined CONFIG_LV_PREDEFINED_DISPLAY_M5STACK
    uint8_t data[] = {0x68, 0x68, 0x08, 0x08};
#elif defined (CONFIG_LV_PREDEFINED_DISPLAY_M5CORE2)
	uint8_t data[] = {0x08, 0x88, 0x28, 0xE8};
#elif defined (CONFIG_LV_PREDEFINED_DISPLAY_WROVER4)
    uint8_t data[] = {0x6C, 0xEC, 0xCC, 0x4C};
#elif defined (CONFIG_LV_PREDEFINED_DISPLAY_NONE)
 #if 0
    uint8_t data[] = {0xE8, 0x28, 0xC8, 0x08};
//  uint8_t data[] = {0x48, 0x88, 0x28, 0xE8};     // orig
//  uint8_t data[] = {0xA0, 0x60, 0x00, 0xC0};     // usb right/top
//  uint8_t data[] = {0x60, 0xA0, 0xC0, 0x00};     // usb left/bottom
#else
   uint8_t data[] =
   {
   //   MY           MX           MV           BGR
      ( 0 << 7 ) | ( 1 << 6 ) | ( 1 << 5 ) | ( 0 << 3 ),  // 0x60  PORTRAIT
      ( 1 << 7 ) | ( 0 << 6 ) | ( 1 << 5 ) | ( 0 << 3 ),  // 0xA0  PORTRAIT_INVERTED
      ( 1 << 7 ) | ( 1 << 6 ) | ( 0 << 5 ) | ( 0 << 3 ),  // 0xC0  LANDSCAPE
      ( 0 << 7 ) | ( 0 << 6 ) | ( 0 << 5 ) | ( 0 << 3 ),  // 0x00  LANDSCAPE_INVERTED
   };
 #endif
#endif

    ESP_LOGI(TAG, "0x36 command value: 0x%02X", data[orientation]);

    ili9341_send_cmd(0x36);
    ili9341_send_data((void *) &data[orientation], 1);
}
