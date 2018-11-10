#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

#include "u8g2_esp32_hal.h"
#include "icons.h"



void app_main() {


	U8g2Hal_I2C_Oled_Init();   /* Init I2C OLED display */
	U8g2Hal_SPI_Oled_Init();   /* Init SPI OLED display */

	while(1){

		u8g2_ClearBuffer(&U8g2Hal_I2C_Oled_Handle);
		u8g2_DrawXBM(&U8g2Hal_I2C_Oled_Handle, 1, 1, 62, 62, termo_icon14);
		u8g2_SendBuffer(&U8g2Hal_I2C_Oled_Handle);
		vTaskDelay(2000 / portTICK_RATE_MS);


		u8g2_ClearBuffer(&U8g2Hal_I2C_Oled_Handle);
		u8g2_SetFont(&U8g2Hal_I2C_Oled_Handle, u8g2_font_timR14_tf);
		u8g2_DrawStr(&U8g2Hal_I2C_Oled_Handle, 2,17,"Termometru!");
		u8g2_DrawStr(&U8g2Hal_I2C_Oled_Handle, 32,50,"99.49xyC");
		u8g2_SendBuffer(&U8g2Hal_I2C_Oled_Handle);
		vTaskDelay(500 / portTICK_RATE_MS);


		/***********************************************************/
		u8g2_ClearBuffer(&U8g2Hal_SPI_Oled_Handle);
		u8g2_DrawXBM(&U8g2Hal_SPI_Oled_Handle, 34, 1, 62, 62, termo_icon14);
		u8g2_SendBuffer(&U8g2Hal_SPI_Oled_Handle);
		vTaskDelay(2000 / portTICK_RATE_MS);

						// set font and write hello world
		u8g2_ClearBuffer(&U8g2Hal_SPI_Oled_Handle);
		u8g2_SetFont(&U8g2Hal_SPI_Oled_Handle, u8g2_font_timR14_tf);
		u8g2_DrawStr(&U8g2Hal_SPI_Oled_Handle, 2,17,"Termometru!");
		u8g2_DrawStr(&U8g2Hal_SPI_Oled_Handle, 32,50,"25.4 C");
		u8g2_SendBuffer(&U8g2Hal_SPI_Oled_Handle);
		vTaskDelay(500 / portTICK_RATE_MS);
	}
	// loop
	// initialize the display
/*	u8g2_InitDisplay(&u8g2);

	// wake up the display
	u8g2_SetPowerSave(&u8g2, 0);

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
	}	*/
}
