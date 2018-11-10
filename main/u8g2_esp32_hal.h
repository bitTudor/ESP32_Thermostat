/*
*********************************************************************************************************
*                                                uC/CPU
*                                    CPU CONFIGURATION & PORT LAYER
*
*                          (c) Copyright 2004-2013; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               uC/CPU is provided in source form to registered licensees ONLY.  It is
*               illegal to distribute this source code to any third party unless you receive
*               written permission by an authorized Micrium representative.  Knowledge of
*               the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can find our product's user manual, API reference, release notes and
*               more information at https://doc.micrium.com.
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                           CORE CPU MODULE
*
* Filename      : cpu_core.h
* Version       : V1.30.01
* Programmer(s) : SR
*                 ITJ
*********************************************************************************************************
* Note(s)       : (1) Assumes the following versions (or more recent) of software modules are included in
*                     the project build :
*
*                     (a) uC/LIB V1.35.00
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               MODULE
*
* Note(s) : (1) This core CPU header file is protected from multiple pre-processor inclusion through use of
*               the  core CPU module present pre-processor macro definition.
*********************************************************************************************************
*/
#ifndef U8G2_ESP32_HAL_H_INCLUDED
#define U8G2_ESP32_HAL_H_INCLUDED

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
//#include "sdkconfig.h"
#include "esp_log.h"
#include "u8g2.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "u8g2_hal_cfg.h"
/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/

#ifdef   U8G2_ESP32_HAL_MODULE
#define  U8G2_ESP32_HAL_LCD_EXT
#else
#define  U8G2_ESP32_HAL_LCD_EXT  extern
#endif

/*
*********************************************************************************************************
*                                      LANGUAGE SUPPORT DEFINES
*********************************************************************************************************
*/

#ifdef __cplusplus
 extern "C" {
#endif



 /*
 *********************************************************************************************************
 *                                               CONSTANTS
 *********************************************************************************************************
 */


 /*
 *********************************************************************************************************
 *                                                DEFINES
 *********************************************************************************************************
 */


 /*
 *********************************************************************************************************
 *                                            GLOBAL MACRO'S
 *********************************************************************************************************
 */


 /*
 *********************************************************************************************************
 *                                          GLOBAL DATA TYPES
 *********************************************************************************************************
 */
 typedef struct {
 	gpio_num_t clk;
 	gpio_num_t mosi;
 	gpio_num_t cs;
 	gpio_num_t reset;
 	gpio_num_t dc;
 } U8G2_HAL_SPI_t ;

 typedef struct {
 	gpio_num_t sda;
 	gpio_num_t scl;
 } U8G2_HAL_I2C_t ;



 /*
 *********************************************************************************************************
 *                                           GLOBAL VARIABLES
 *********************************************************************************************************
 */
 static  U8G2_HAL_SPI_t  U8g2Hal_SPI_Pins = {
	U8G2HAL_CFG_PIN_CLK,
	U8G2HAL_CFG_PIN_MOSI,
	U8G2HAL_CFG_PIN_CS,
	U8G2HAL_CFG_PIN_RESET,
	U8G2HAL_CFG_PIN_DC
 };

 static  U8G2_HAL_I2C_t  U8g2Hal_I2C_Pins = {
	U8G2HAL_CFG_PIN_SDA,
	U8G2HAL_CFG_PIN_SCL,
 };

 U8G2_ESP32_HAL_LCD_EXT u8g2_t  U8g2Hal_SPI_Oled_Handle;
 U8G2_ESP32_HAL_LCD_EXT u8g2_t  U8g2Hal_I2C_Oled_Handle;

 /*
 *********************************************************************************************************
 *                                      GLOBAL FUNCTION PROTOTYPES
 *********************************************************************************************************
 */
#ifdef U8G2HAL_CFG_SPI_OLED_EN
U8G2_ESP32_HAL_LCD_EXT void  U8g2Hal_SPI_Oled_Init(void);
#endif /* U8G2HAL_CFG_SPI_OLED_EN */

#ifdef U8G2HAL_CFG_I2C_OLED_EN
U8G2_ESP32_HAL_LCD_EXT void  U8g2Hal_I2C_Oled_Init(void);
#endif /* U8G2HAL_CFG_I2C_OLED_EN */

#ifdef __cplusplus
}
#endif


#endif /* U8G2_ESP32_HAL_H_INCLUDED */
