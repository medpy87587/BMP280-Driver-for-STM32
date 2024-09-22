/*
 * BMP280.h
 *
 *  Created on: Sep 21, 2024
 *      Author: medam
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_


#include "stm32f4xx_hal.h"


// Communication Modes
typedef enum {
    BMP280_MODE_I2C,
    BMP280_MODE_SPI
} bmp280_mode_t;

// Sensor Operating Modes
typedef enum {
    BMP280_MODE_SLEEP = 0x00,
    BMP280_MODE_FORCED = 0x01,
    BMP280_MODE_NORMAL = 0x03
} bmp280_operating_mode_t;

// Oversampling Settings
typedef enum {
    BMP280_OSAMPLE_1 = 0x01,
    BMP280_OSAMPLE_2 = 0x02,
    BMP280_OSAMPLE_4 = 0x03,
    BMP280_OSAMPLE_8 = 0x04,
    BMP280_OSAMPLE_16 = 0x05
} bmp280_oversampling_t;

// Standby Time
typedef enum {
    BMP280_STANDBY_0_5_MS = 0x00,
    BMP280_STANDBY_10_MS = 0x01,
    BMP280_STANDBY_20_MS = 0x02,
    BMP280_STANDBY_62_5_MS = 0x03,
    BMP280_STANDBY_125_MS = 0x04,
    BMP280_STANDBY_250_MS = 0x05,
    BMP280_STANDBY_500_MS = 0x06,
    BMP280_STANDBY_1000_MS = 0x07
} bmp280_standby_time_t;

// Filter Settings
typedef enum {
    BMP280_FILTER_OFF = 0x00,
    BMP280_FILTER_2 = 0x01,
    BMP280_FILTER_4 = 0x02,
    BMP280_FILTER_8 = 0x03,
    BMP280_FILTER_16 = 0x04
} bmp280_filter_t;

// BMP280 Structure
typedef struct {
    I2C_HandleTypeDef *hi2c;   ///< I2C Handle
    SPI_HandleTypeDef *hspi;   ///< SPI Handle
    GPIO_TypeDef* cs_port;     ///< SPI Chip Select Port
    uint16_t cs_pin;           ///< SPI Chip Select Pin
    bmp280_mode_t comm_mode;   ///< Communication mode: I2C or SPI
    uint8_t address;           ///< I2C address (default 0x76)
    // Calibration Data
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_t;

// Function Prototypes

/**
 * @brief Initialize the BMP280 sensor
 * @param bmp Pointer to bmp280_t structure
 * @return HAL status
 */
HAL_StatusTypeDef bmp280_init(bmp280_t *bmp);

/**
 * @brief Read raw temperature and pressure data
 * @param bmp Pointer to bmp280_t structure
 * @param temperature_raw Pointer to store raw temperature
 * @param pressure_raw Pointer to store raw pressure
 * @return HAL status
 */
HAL_StatusTypeDef bmp280_read_raw(bmp280_t *bmp, int32_t *temperature_raw, int32_t *pressure_raw);

/**
 * @brief Calculate actual temperature from raw data
 * @param bmp Pointer to bmp280_t structure
 * @param temperature_raw Raw temperature data
 * @return Temperature in °C
 */
float bmp280_compensate_temperature(bmp280_t *bmp, int32_t temperature_raw);

/**
 * @brief Calculate actual pressure from raw data
 * @param bmp Pointer to bmp280_t structure
 * @param pressure_raw Raw pressure data
 * @return Pressure in hPa
 */
float bmp280_compensate_pressure(bmp280_t *bmp, int32_t pressure_raw);

/**
 * @brief Calculate altitude based on pressure
 * @param pressure Pressure in hPa
 * @param sea_level_pressure Sea level pressure in hPa
 * @return Altitude in meters
 */
float bmp280_calculate_altitude(float pressure, float sea_level_pressure);

/**
 * @brief Set sensor configuration
 * @param bmp Pointer to bmp280_t structure
 * @param mode Operating mode
 * @param osrs_t Temperature oversampling
 * @param osrs_p Pressure oversampling
 * @param filter Filter setting
 * @param standby Standby time
 * @return HAL status
 */
HAL_StatusTypeDef bmp280_set_configuration(bmp280_t *bmp, bmp280_operating_mode_t mode,
                                           bmp280_oversampling_t osrs_t, bmp280_oversampling_t osrs_p,
                                           bmp280_filter_t filter, bmp280_standby_time_t standby);

/**
 * @brief Soft reset the BMP280 sensor
 * @param bmp Pointer to bmp280_t structure
 * @return HAL status
 */
HAL_StatusTypeDef bmp280_soft_reset(bmp280_t *bmp);


#endif /* INC_BMP280_H_ */
