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
*                                           ESP32 U8G2 HAL MODULE
*
* Filename      : cpu_core.c
* Version       : V1.30.01
* Programmer(s) : SR
*                 ITJ
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#define U8G2_ESP32_HAL_MODULE
#include "u8g2_esp32_hal.h"


/*
*********************************************************************************************************
*
EXTERNAL C LANGUAGE LINKAGE
*
* Note(s) : (1) C++ compilers MUST 'extern'ally declare ALL C function prototypes & variable/object
*
declarations for correct C language linkage.
*********************************************************************************************************
*/
#ifdef __cplusplus
extern
"C" {
/* See Note #1.
*/
#endif


/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL CONSTANTS
*********************************************************************************************************
*/
static  const char         *TAG            = "u8g2_hal";
static  const unsigned int  I2C_TIMEOUT_MS = 1000;

/*
*********************************************************************************************************
*                                           LOCAL DATA TYPES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
static  spi_device_handle_t  U8g2Hal_SpiHandle;      /* SPI handle. */
static  i2c_cmd_handle_t     handle_i2c;      // I2C handle.

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/
#ifdef U8G2HAL_CFG_SPI_OLED_EN
static uint8_t  U8g2Hal_SpiByte_CB       (u8x8_t  *u8x8,
		                                  uint8_t  msg,
										  uint8_t  arg_int,
										  void    *arg_ptr);

static uint8_t  U8g2Hal_GpioDelay_CB     (u8x8_t  *u8x8,
		                                  uint8_t  msg,
										  uint8_t  arg_int,
										  void    *arg_ptr);
#endif /* U8G2HAL_CFG_SPI_OLED_EN */

#ifdef U8G2HAL_CFG_I2C_OLED_EN
static uint8_t  U8g2Hal_I2cByte_CB       (u8x8_t  *u8x8,
 		                                  uint8_t  msg,
 							              uint8_t  arg_int,
 							              void    *arg_ptr);

static uint8_t  U8g2Hal_GpioDelay_I2C_CB (u8x8_t  *u8x8,
		                                  uint8_t  msg,
								          uint8_t  arg_int,
								          void    *arg_ptr);
#endif /* U8G2HAL_CFG_I2C_OLED_EN */

/*
*********************************************************************************************************
*********************************************************************************************************
*                                           LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/
#ifdef U8G2HAL_CFG_SPI_OLED_EN
/*
*********************************************************************************************************
*                                           U8g2Hal_SpiByte_CB()
*
* Description : Determine whether a character is an alphabetic character.
*
* Argument(s) : c           Character to examine.
*
* Return(s)   : DEF_YES, if character is     an alphabetic character.
*
*               DEF_NO,     if character is NOT an alphabetic character.
*
* Caller(s)   : Application.
*
* Note(s)     : HAL callback function as prescribed by the U8G2 library.  This callback is invoked
*                   to handle SPI communications.
*********************************************************************************************************
*/
uint8_t  U8g2Hal_SpiByte_CB(u8x8_t  *u8x8,
		                    uint8_t  msg,
							uint8_t  arg_int,
							void    *arg_ptr)
{
	static spi_bus_config_t               bus_config;
	static spi_device_interface_config_t  dev_config;
	       spi_transaction_t              trans_desc;


    ESP_LOGD(TAG, "spi_byte_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);

    switch(msg) {
        case U8X8_MSG_BYTE_SET_DC:
		     if (U8g2Hal_SPI_Pins.dc != U8G2_HAL_UNDEFINED) {
				gpio_set_level(U8g2Hal_SPI_Pins.dc, arg_int);
			 }
			 break;

		case U8X8_MSG_BYTE_INIT:
			 if (U8g2Hal_SPI_Pins.clk  == U8G2_HAL_UNDEFINED ||
				 U8g2Hal_SPI_Pins.mosi == U8G2_HAL_UNDEFINED ||
				 U8g2Hal_SPI_Pins.cs   == U8G2_HAL_UNDEFINED) {
				break;
			 }
             bus_config.sclk_io_num   = U8g2Hal_SPI_Pins.clk;                                      /* CLK      */
		     bus_config.mosi_io_num   = U8g2Hal_SPI_Pins.mosi;                                     /* MOSI     */
		     bus_config.miso_io_num   = -1;                                                        /* MISO     */
		     bus_config.quadwp_io_num = -1;                                                        /* Not used */
		     bus_config.quadhd_io_num = -1;                                                        /* Not used */

             ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 1));                       /* ESP_LOGI(TAG, "... Initializing bus."); */

             dev_config.address_bits     = 0;
             dev_config.command_bits     = 0;
             dev_config.dummy_bits       = 0;
             dev_config.mode             = 0;
             dev_config.duty_cycle_pos   = 0;
             dev_config.cs_ena_posttrans = 0;
             dev_config.cs_ena_pretrans  = 0;
             dev_config.clock_speed_hz   = U8G2HAL_CFG_SPI_BUS_SPEED;
             dev_config.spics_io_num     = U8g2Hal_SPI_Pins.cs;
             dev_config.flags            = 0;
             dev_config.queue_size       = U8G2HAL_CFG_SPI_QUEUE_SIZE;
             dev_config.pre_cb           = NULL;
             dev_config.post_cb          = NULL;

		     ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &U8g2Hal_SpiHandle));     /* ESP_LOGI(TAG, "... Adding device bus."); */
             break;

        case U8X8_MSG_BYTE_SEND:
             trans_desc.addr      = 0;
             trans_desc.cmd   	  = 0;
             trans_desc.flags     = 0;
             trans_desc.length    = 8 * arg_int;                                                  /* Number of bits NOT number of bytes. */
             trans_desc.rxlength  = 0;
             trans_desc.tx_buffer = arg_ptr;
             trans_desc.rx_buffer = NULL;

			 ESP_ERROR_CHECK(spi_device_transmit(U8g2Hal_SpiHandle, &trans_desc));                /* ESP_LOGI(TAG, "... Transmitting %d bytes.", arg_int); */
			 break;
	}
	return 0;
}



/*
*********************************************************************************************************
*                                           U8g2Hal_SpiByte_CB()
*
* Description : Determine whether a character is an alphabetic character.
*
* Argument(s) : c           Character to examine.
*
* Return(s)   : DEF_YES, if character is     an alphabetic character.
*
*               DEF_NO,     if character is NOT an alphabetic character.
*
* Caller(s)   : Application.
*
* Note(s)     : HAL callback function as prescribed by the U8G2 library.  This callback is invoked
*                   to handle callbacks for GPIO and delay functions.
*********************************************************************************************************
*/
uint8_t  U8g2Hal_GpioDelay_CB(u8x8_t  *u8x8,
		                      uint8_t  msg,
							  uint8_t  arg_int,
							  void    *arg_ptr)
{
	static gpio_config_t gpioConfig;
	uint64_t bitmask = 0;


	ESP_LOGD(TAG, "gpio_and_delay_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);

	switch(msg) {
		case U8X8_MSG_GPIO_AND_DELAY_INIT:                             /* Initialize the GPIO and DELAY HAL functions.  If the pins for DC and */
			if (U8g2Hal_SPI_Pins.dc != U8G2_HAL_UNDEFINED) {           /* RESET have been specified then we define those pins as GPIO outputs. */
				bitmask |= (1ull << U8g2Hal_SPI_Pins.dc);
			}
			if (U8g2Hal_SPI_Pins.reset != U8G2_HAL_UNDEFINED) {
				bitmask |= (1ull << U8g2Hal_SPI_Pins.reset);
			}
			if (U8g2Hal_SPI_Pins.cs != U8G2_HAL_UNDEFINED) {
				bitmask |= (1ull << U8g2Hal_SPI_Pins.cs);
			}

            if (bitmask == 0) {
            	break;
            }

			gpioConfig.pin_bit_mask = bitmask;
			gpioConfig.mode         = GPIO_MODE_OUTPUT;
			gpioConfig.pull_up_en   = GPIO_PULLUP_DISABLE;
			gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
			gpioConfig.intr_type    = GPIO_INTR_DISABLE;
			gpio_config(&gpioConfig);
			break;

		case U8X8_MSG_GPIO_RESET:                                      /* Set the GPIO reset pin to the value passed in through arg_int. */
			if (U8g2Hal_SPI_Pins.reset != U8G2_HAL_UNDEFINED) {
				gpio_set_level(U8g2Hal_SPI_Pins.reset, arg_int);
			}
			break;

		case U8X8_MSG_GPIO_CS:                                         /* Set the GPIO client select pin to the value passed in through arg_int. */
			if (U8g2Hal_SPI_Pins.cs != U8G2_HAL_UNDEFINED) {
				gpio_set_level(U8g2Hal_SPI_Pins.cs, arg_int);
			}
			break;

		case U8X8_MSG_DELAY_MILLI:                                     /* Delay for the number of milliseconds passed in through arg_int. */
			vTaskDelay(arg_int/portTICK_PERIOD_MS);
			//vTaskDelay(arg_int/10000);
			break;
	}
	return 0;
}
#endif /* U8G2HAL_CFG_SPI_OLED_EN */

/*
*********************************************************************************************************
*                                           U8g2Hal_SpiByte_CB()
*
* Description : Determine whether a character is an alphabetic character.
*
* Argument(s) : c           Character to examine.
*
* Return(s)   : DEF_YES, if character is     an alphabetic character.
*
*               DEF_NO,     if character is NOT an alphabetic character.
*
* Caller(s)   : Application.
*
* Note(s)     : HAL callback function as prescribed by the U8G2 library.  This callback is invoked
*                   to handle I2C communications.
*********************************************************************************************************
*/
#ifdef U8G2HAL_CFG_I2C_OLED_EN
uint8_t  U8g2Hal_I2cByte_CB(u8x8_t  *u8x8,
		                    uint8_t  msg,
							uint8_t  arg_int,
							void    *arg_ptr)
{
	ESP_LOGD(TAG, "i2c_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);

   static i2c_config_t conf;

	switch(msg) {

		case U8X8_MSG_BYTE_INIT: {
			if (U8g2Hal_I2C_Pins.sda == U8G2_HAL_UNDEFINED ||
					U8g2Hal_I2C_Pins.scl == U8G2_HAL_UNDEFINED) {
				break;
			}

		    conf.mode = I2C_MODE_MASTER;
			ESP_LOGI(TAG, "sda_io_num %d", U8g2Hal_I2C_Pins.sda);
		    conf.sda_io_num = U8g2Hal_I2C_Pins.sda;
		    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
			ESP_LOGI(TAG, "scl_io_num %d", U8g2Hal_I2C_Pins.scl);
		    conf.scl_io_num = U8g2Hal_I2C_Pins.scl;
		    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
			ESP_LOGI(TAG, "clk_speed %d", I2C_MASTER_FREQ_HZ);
		    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
			ESP_LOGI(TAG, "i2c_param_config %d", conf.mode);
		    ESP_ERROR_CHECK(i2c_param_config(U8G2HAL_CFG_I2C_MASTER_NUM, &conf));
			ESP_LOGI(TAG, "i2c_driver_install %d", U8G2HAL_CFG_I2C_MASTER_NUM);
		    ESP_ERROR_CHECK(i2c_driver_install(U8G2HAL_CFG_I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
			break;
		}

		case U8X8_MSG_BYTE_SEND: {
			uint8_t* data_ptr = (uint8_t*)arg_ptr;
			ESP_LOG_BUFFER_HEXDUMP(TAG, data_ptr, arg_int, ESP_LOG_VERBOSE);

			while( arg_int > 0 ) {
			   ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, *data_ptr, ACK_CHECK_EN));
			   data_ptr++;
			   arg_int--;
			}
			break;
		}

		case U8X8_MSG_BYTE_START_TRANSFER: {
			uint8_t i2c_address = u8x8_GetI2CAddress(u8x8);
			handle_i2c = i2c_cmd_link_create();
			ESP_LOGD(TAG, "Start I2C transfer to %02X.", i2c_address>>1);
			ESP_ERROR_CHECK(i2c_master_start(handle_i2c));
			ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, i2c_address | I2C_MASTER_WRITE, ACK_CHECK_EN));
			break;
		}

		case U8X8_MSG_BYTE_END_TRANSFER: {
			ESP_LOGD(TAG, "End I2C transfer.");
			ESP_ERROR_CHECK(i2c_master_stop(handle_i2c));
			ESP_ERROR_CHECK(i2c_master_cmd_begin(U8G2HAL_CFG_I2C_MASTER_NUM, handle_i2c, I2C_TIMEOUT_MS / portTICK_RATE_MS));
			i2c_cmd_link_delete(handle_i2c);
			break;
		}

		default:
			break;
	}
	return 0;
}


/*
*********************************************************************************************************
*                                           U8g2Hal_SpiByte_CB()
*
* Description : Determine whether a character is an alphabetic character.
*
* Argument(s) : c           Character to examine.
*
* Return(s)   : DEF_YES, if character is     an alphabetic character.
*
*               DEF_NO,     if character is NOT an alphabetic character.
*
* Caller(s)   : Application.
*
* Note(s)     : HAL callback function as prescribed by the U8G2 library.  This callback is invoked
*                   to handle callbacks for GPIO and delay functions.
*********************************************************************************************************
*/
uint8_t  U8g2Hal_GpioDelay_I2C_CB(u8x8_t  *u8x8,
		                          uint8_t  msg,
								  uint8_t  arg_int,
								  void    *arg_ptr)
{
	ESP_LOGD(TAG, "gpio_and_delay_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);

	switch(msg) {
		case U8X8_MSG_GPIO_I2C_CLOCK:                            // Set the Software I²C pin to the value passed in through arg_int.
			if (U8g2Hal_I2C_Pins.scl != U8G2_HAL_UNDEFINED) {
				gpio_set_level(U8g2Hal_I2C_Pins.scl, arg_int);
			}
			break;

		case U8X8_MSG_GPIO_I2C_DATA:                            // Set the Software I²C pin to the value passed in through arg_int.
			if (U8g2Hal_I2C_Pins.sda != U8G2_HAL_UNDEFINED) {
				gpio_set_level(U8g2Hal_I2C_Pins.sda, arg_int);
			}
			break;

		case U8X8_MSG_DELAY_MILLI:                              // Delay for the number of milliseconds passed in through arg_int.
			vTaskDelay(arg_int/portTICK_PERIOD_MS);
			break;

		default:
			break;
	}
	return 0;
}
#endif /* U8G2HAL_CFG_I2C_OLED_EN */


/*
*********************************************************************************************************
*********************************************************************************************************
*                                          GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           U8g2Hal_SpiByte_CB()
*
* Description : Determine whether a character is an alphabetic character.
*
* Argument(s) : c           Character to examine.
*
* Return(s)   : DEF_YES, if character is     an alphabetic character.
*
*               DEF_NO,     if character is NOT an alphabetic character.
*
* Caller(s)   : Application.
*
* Note(s)     : HAL callback function as prescribed by the U8G2 library.  This callback is invoked
*                   to handle callbacks for GPIO and delay functions.
*********************************************************************************************************
*/
#ifdef U8G2HAL_CFG_SPI_OLED_EN
U8G2_ESP32_HAL_LCD_EXT void  U8g2Hal_SPI_Oled_Init(void)
{

	U8G2HAL_CFG_OLED_SPI_CONTROLLER(&U8g2Hal_SPI_Oled_Handle,    /* initialize the u8g2 library */
		                             U8G2_R0,
		                             U8g2Hal_SpiByte_CB,
		                             U8g2Hal_GpioDelay_CB);

	u8g2_InitDisplay(&U8g2Hal_SPI_Oled_Handle);                  /* initialize the display */
	u8g2_SetPowerSave(&U8g2Hal_SPI_Oled_Handle, 0);              /* wake up the display */
}
#endif /* U8G2HAL_CFG_I2C_OLED_EN */


/*
*********************************************************************************************************
*                                           U8g2Hal_SpiByte_CB()
*
* Description : Determine whether a character is an alphabetic character.
*
* Argument(s) : c           Character to examine.
*
* Return(s)   : DEF_YES, if character is     an alphabetic character.
*
*               DEF_NO,     if character is NOT an alphabetic character.
*
* Caller(s)   : Application.
*
* Note(s)     : HAL callback function as prescribed by the U8G2 library.  This callback is invoked
*                   to handle callbacks for GPIO and delay functions.
*********************************************************************************************************
*/
#ifdef U8G2HAL_CFG_I2C_OLED_EN
U8G2_ESP32_HAL_LCD_EXT void  U8g2Hal_I2C_Oled_Init(void)
{

	U8G2HAL_CFG_OLED_I2C_CONTROLLER(&U8g2Hal_I2C_Oled_Handle,                /* initialize the u8g2 library */
		                             U8G2_R0,
		                             U8g2Hal_I2cByte_CB,
		                             U8g2Hal_GpioDelay_I2C_CB);

	u8x8_SetI2CAddress(&U8g2Hal_I2C_Oled_Handle.u8x8, U8G2HAL_CFG_DEV_ADDR); /* set the I2C  address for OLED lcd */
	u8g2_InitDisplay(&U8g2Hal_I2C_Oled_Handle);                              /* initialize the display */
	u8g2_SetPowerSave(&U8g2Hal_I2C_Oled_Handle, 0);             	         /* wake up the display */

}
#endif /* U8G2HAL_CFG_I2C_OLED_EN */




/*
*********************************************************************************************************
*
EXTERNAL C LANGUAGE LINKAGE END
*********************************************************************************************************
*/
#ifdef __cplusplus
}
#endif
