/*
 * u8g2_hal_cfg.h
 *
 *  Created on: Nov 10, 2018
 *      Author: bit
 */

#ifndef U8G2_HAL_CFG_H_
#define U8G2_HAL_CFG_H_


 /*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                                DEFINES
 *********************************************************************************************************
 *********************************************************************************************************
 */

#define U8G2HAL_CFG_SPI_OLED_EN                                              /* Enable SPI OLED functionality */
#define U8G2HAL_CFG_I2C_OLED_EN                                              /* Enable I2C OLED functionality */

#define U8G2_HAL_UNDEFINED (-1)


 /*
 *********************************************************************************************************
 *                                         OLED SPI CONFIGURATION
 *********************************************************************************************************
 */
#ifdef U8G2HAL_CFG_SPI_OLED_EN                                                                               /* SPI PINOUT      */
#define U8G2HAL_CFG_PIN_CLK               14                                   /* CLK   - GPIO 14 */
#define U8G2HAL_CFG_PIN_MOSI              13                                   /* MOSI  - GPIO 13 */
#define U8G2HAL_CFG_PIN_RESET             26                                   /* RESET - GPIO 26 */
#define U8G2HAL_CFG_PIN_DC                27                                   /* DC    - GPIO 27 */
#define U8G2HAL_CFG_PIN_CS                15                                   /* CS    - GPIO 15 */


#define U8G2HAL_CFG_SPI_BUS_SPEED         1000000                               /* clock speed in Hz */
#define U8G2HAL_CFG_SPI_QUEUE_SIZE        200                                  /* queue buffer size */


#define U8G2HAL_CFG_OLED_SPI_CONTROLLER   u8g2_Setup_ssd1309_128x64_noname2_f  /* LCD Controller type */
#endif /* U8G2HAL_CFG_SPI_OLED_EN */
 /*
 *********************************************************************************************************
 *                                         OLED I2C CONFIGURATION
 *********************************************************************************************************
 */
#ifdef U8G2HAL_CFG_I2C_OLED_EN                                                                                /* I2C PINOUT      */
#define U8G2HAL_CFG_PIN_SDA               21                                    /* SDA   - GPIO 27 */
#define U8G2HAL_CFG_PIN_SCL               22                                    /* SCL   - GPIO 15 */

#define U8G2HAL_CFG_DEV_ADDR            0x78
                                                                                 /* I2C Bus configuration                     */
#define U8G2HAL_CFG_I2C_MASTER_NUM        I2C_NUM_1                              /*  I2C port number for master dev           */
#define I2C_MASTER_TX_BUF_DISABLE         0                                      /*  I2C master do not need buffer            */
#define I2C_MASTER_RX_BUF_DISABLE         0                                      /*  I2C master do not need buffer            */
#define I2C_MASTER_FREQ_HZ               200000                                  /*  I2C master clock frequency               */
#define ACK_CHECK_EN                    0x1                                      /*  I2C master will check ack from slave     */
#define ACK_CHECK_DIS                   0x0                                      /*  I2C master will not check ack from slave */

#define U8G2HAL_CFG_OLED_I2C_CONTROLLER   u8g2_Setup_sh1106_i2c_128x64_noname_f  /* LCD Controller type */
#endif /* U8G2HAL_CFG_I2C_OLED_EN */


#endif /* MAIN_U8G2_HAL_CFG_H_ */
