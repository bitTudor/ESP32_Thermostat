#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

#include "u8g2_esp32_hal.h"
#include "icons.h"

// CLK - GPIO14
#define PIN_CLK 14
// MOSI - GPIO 13
#define PIN_MOSI 13
// RESET - GPIO 26
#define PIN_RESET 26
// DC - GPIO 27
#define PIN_DC 27
// CS - GPIO 15
#define PIN_CS 15
static const char *TAG = "u8g2_hal";

const u8g2_esp32_hal_t U8G2_ESP32_HAL_DEFAULT1 = {
		U8G2_ESP32_HAL_UNDEFINED,
		U8G2_ESP32_HAL_UNDEFINED,
		U8G2_ESP32_HAL_UNDEFINED,
		U8G2_ESP32_HAL_UNDEFINED,
		U8G2_ESP32_HAL_UNDEFINED
};

void app_main() {
	ESP_LOGD(TAG, "Inainte de init display.");
	// initialize the u8g2 hal
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT1;
	u8g2_esp32_hal.clk   = PIN_CLK;
	u8g2_esp32_hal.mosi  = PIN_MOSI;
	u8g2_esp32_hal.cs    = PIN_CS;
	u8g2_esp32_hal.dc    = PIN_DC;
	u8g2_esp32_hal.reset = PIN_RESET;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

	// initialize the u8g2 library
	u8g2_t u8g2;
	u8g2_Setup_ssd1309_128x64_noname2_f(
		&u8g2,
		U8G2_R0,
		u8g2_esp32_spi_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);
	
	// set the display address
	// u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);
	ESP_LOGD(TAG, "Inainte de init display.");
	// initialize the display
	u8g2_InitDisplay(&u8g2);
	ESP_LOGD(TAG, "Dupa init display.");
	// wake up the display
	u8g2_SetPowerSave(&u8g2, 0);
	ESP_LOGD(TAG, "dupa power on.");
	// loop
	while(1) {

		// draw the hourglass animation, full-half-empty
		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon1);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon2);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon3);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon4);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		/**********************************************/

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon5);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon6);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon7);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon8);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon9);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon10);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon11);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon12);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon13);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 1, 62, 62, termo_icon14);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(2000 / portTICK_RATE_MS);

		// set font and write hello world
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_timR14_tf);
		u8g2_DrawStr(&u8g2, 2,17,"Termometru!");
		u8g2_DrawStr(&u8g2, 32,50,"25.4 C");
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(500 / portTICK_RATE_MS);
	}	
}
