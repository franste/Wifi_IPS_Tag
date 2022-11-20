#ifndef BMP390_H
#define BMP390_H

#include "esp_err.h"

//Hardware SPI pins
#define PIN_NUM_CS   10     // GPIO10 for CS
#define PIN_NUM_MOSI 11     // GPIO11 for MOSI/SDA
#define PIN_NUM_MISO 13     // GPIO13 for MISO/SDO
#define PIN_NUM_CLK  12     // GPIO12 for CLK/SCK/SCL

//Hardware SPI instance
#define BMP390_HOST    SPI2_HOST

#define BMP390_OK      0
#define BMP390_FAIL    1

// BMP390 sensor registers
#define BMP390_REG_CHIP_ID             0x00        /**< chip id register */
#define BMP390_REG_CMD                 0x7E        /**< command register */
#define BMP390_REG_ERR_REG             0x02        /**< error register */
#define BMP390_REG_NVM_PAR_T1_L        0x31        /**< NVM PAR T1 low register */
#define BMP390_REG_NVM_PAR_T1_H        0x32        /**< NVM PAR T1 high register */
#define BMP390_REG_NVM_PAR_T2_L        0x33        /**< NVM PAR T2 low register */
#define BMP390_REG_NVM_PAR_T2_H        0x34        /**< NVM PAR T2 high register */
#define BMP390_REG_NVM_PAR_T3          0x35        /**< NVM PAR T3 register */
#define BMP390_REG_NVM_PAR_P1_L        0x36        /**< NVM PAR P1 low register */
#define BMP390_REG_NVM_PAR_P1_H        0x37        /**< NVM PAR P1 hgih register */
#define BMP390_REG_NVM_PAR_P2_L        0x38        /**< NVM PAR P2 low register */
#define BMP390_REG_NVM_PAR_P2_H        0x39        /**< NVM PAR P2 hgih register */
#define BMP390_REG_NVM_PAR_P3          0x3A        /**< NVM PAR P3 register */
#define BMP390_REG_NVM_PAR_P4          0x3B        /**< NVM PAR P4 register */
#define BMP390_REG_NVM_PAR_P5_L        0x3C        /**< NVM PAR P5 low register */
#define BMP390_REG_NVM_PAR_P5_H        0x3D        /**< NVM PAR P5 hgih register */
#define BMP390_REG_NVM_PAR_P6_L        0x3E        /**< NVM PAR P6 low register */
#define BMP390_REG_NVM_PAR_P6_H        0x3F        /**< NVM PAR P6 hgih register */
#define BMP390_REG_NVM_PAR_P7          0x40        /**< NVM PAR P7 register */
#define BMP390_REG_NVM_PAR_P8          0x41        /**< NVM PAR P8 register */
#define BMP390_REG_NVM_PAR_P9_L        0x42        /**< NVM PAR P9 low register */
#define BMP390_REG_NVM_PAR_P9_H        0x43        /**< NVM PAR P9 hgih register */
#define BMP390_REG_NVM_PAR_P10         0x44        /**< NVM PAR P10 register */
#define BMP390_REG_NVM_PAR_P11         0x45        /**< NVM PAR P11 register */

//Pressure shot configuration
#define BMP390_SHOT_DEFAULT_SPI_WIRE                 BMP390_SPI_WIRE_4                        /**< 4 wire spi */
#define BMP390_SHOT_DEFAULT_IIC_WATCHDOG_TIMER       BMP390_BOOL_TRUE                         /**< enable iic watchdog timer */
#define BMP390_SHOT_DEFAULT_IIC_WATCHDOG_PERIOD      BMP390_IIC_WATCHDOG_PERIOD_40_MS         /**< set watchdog timer period 40ms */
#define SHOT_PRESSURE                                BMP390_BOOL_TRUE                         /**< enable pressure */
#define SHOT_TEMPERATURE                             BMP390_BOOL_TRUE                         /**< enable temperature */
#define SHOT_PRESSURE_OVERSAMPLING                   BMP390_OVERSAMPLING_x32                  /**< pressure oversampling x32 */
#define BMP390_SHOT_DEFAULT_TEMPERATURE_OVERSAMPLING BMP390_OVERSAMPLING_x8                   /**< temperature oversampling x2 */
#define SHOT_ODR                                     BMP390_ODR_12P5_HZ                       /**< output data rate 12.5Hz */
#define SHOT_FILTER_COEFFICIENT                      BMP390_FILTER_COEFFICIENT_15             /**< set filter coefficient 15 */

/**
 * @brief Initialize SPI for BMP390 sensor
 */
esp_err_t spi_bmp390_init();

/**
 * @brief Deinitialize SPI for BMP390 sensor
 */
esp_err_t spi_bmp390_deinit();

/**
 * @brief Read BMP390 sensor data. Function used by BMP390 driver, BMP390_handler_t
 */
uint8_t spi_bmp390_read(uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief Write BMP390 sensor data. Function used by BMP390 driver, BMP390_handler_t
 */
uint8_t spi_bmp390_write(uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief Write configuration to BMP390 sensor for temperature and pressure shot
 */
esp_err_t configure_BM390_for_temperature_and_pressure_shot();

/**
 * @brief Read temperature and pressure data from BMP390 sensor, 
 * Sensore needs to be configured for temperature and pressure shot
 * before calling this function and sensore need time to prepare data.
 */
esp_err_t bmp390_temperature_and_pressure_shot_read(float *temperature_c, float *pressure_pa);

/**
 * @brief Tests to so everything is working, initailize, configure and read and write to registers.
 */
esp_err_t spi_bmp390_test();

#endif