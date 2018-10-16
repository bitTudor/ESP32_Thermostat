#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

#include "u8g2_esp32_hal.h"
#include "icons.h"
/*
#define PIN_SDA 5
#define PIN_SCL 4
*/
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

void app_main() {

	// initialize the u8g2 hal
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	/*u8g2_esp32_hal.sda = PIN_SDA;
	u8g2_esp32_hal.scl = PIN_SCL;*/
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
	
	// initialize the display
	u8g2_InitDisplay(&u8g2);
	
	// wake up the display
	u8g2_SetPowerSave(&u8g2, 0);

	// loop
	while(1) {
		/*
		// draw the hourglass animation, full-half-empty
		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 2, 60, 60, hourglass_full);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(1000 / portTICK_RATE_MS);
		
		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 2, 60, 60, hourglass_half);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(1000 / portTICK_RATE_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 2, 60, 60, hourglass_empty);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(1000 / portTICK_RATE_MS);	
		*/
		// set font and write hello world
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_timR14_tf);
		u8g2_DrawStr(&u8g2, 2,17,"Termometru!");
		u8g2_DrawStr(&u8g2, 32,50,"25.4 º°C");
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(5000 / portTICK_RATE_MS);
	}	
}
