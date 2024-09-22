/*
 * BMP280.c
 *
 *  Created on: Sep 21, 2024
 *      Author: medam
 */

#include "BMP280.h"
#include <string.h>

// BMP280 Register Addresses
#define BMP280_REG_CHIPID     0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_STATUS     0xF3
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_PRESS_LSB  0xF8
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_TEMP_MSB   0xFA
#define BMP280_REG_TEMP_LSB   0xFB
#define BMP280_REG_TEMP_XLSB  0xFC

// BMP280 Chip ID
#define BMP280_CHIPID         0x58

// BMP280 Reset Command
#define BMP280_SOFT_RESET_CMD 0xB6


// Helper functions for I2C and SPI communication

static HAL_StatusTypeDef bmp280_write_register(bmp280_t *bmp, uint8_t reg, uint8_t value) {
    if (bmp->comm_mode == BMP280_MODE_I2C) {
        uint8_t data[2] = { reg, value };
        return HAL_I2C_Master_Transmit(bmp->hi2c, bmp->address << 1, data, 2, HAL_MAX_DELAY);
    } else { // SPI
        HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_RESET);
        uint8_t tx_buffer[2] = { reg, value };
        HAL_StatusTypeDef status = HAL_SPI_Transmit(bmp->hspi, tx_buffer, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_SET);
        return status;
    }
}

static HAL_StatusTypeDef bmp280_read_registers(bmp280_t *bmp, uint8_t reg, uint8_t *buffer, uint16_t length) {
    if (bmp->comm_mode == BMP280_MODE_I2C) {
        // Write register address
        if (HAL_I2C_Master_Transmit(bmp->hi2c, bmp->address << 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
            return HAL_ERROR;
        // Read data
        return HAL_I2C_Master_Receive(bmp->hi2c, bmp->address << 1, buffer, length, HAL_MAX_DELAY);
    } else { // SPI
        HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_RESET);
        uint8_t tx_buffer[1] = { reg | 0x80 }; // Set read bit
        HAL_StatusTypeDef status = HAL_SPI_Transmit(bmp->hspi, tx_buffer, 1, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_SET);
            return status;
        }
        status = HAL_SPI_Receive(bmp->hspi, buffer, length, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_SET);
        return status;
    }
}

// Calibration Data Reading
static HAL_StatusTypeDef bmp280_read_calibration_data(bmp280_t *bmp) {
    uint8_t calib_data[24];
    // Read calibration data from 0x88 to 0xA1
    // BMP280 uses different registers for calibration
    // For simplicity, assuming calibration data is contiguous (verify from datasheet)
    // In reality, BMP280 has calibration registers spread out
    // Adjust according to datasheet

    // BMP280 has calibration registers from 0x88 to 0xA1
    // Read all calibration data
    if (bmp->comm_mode == BMP280_MODE_I2C) {
        if (HAL_I2C_Mem_Read(bmp->hi2c, bmp->address << 1, 0x88, I2C_MEMADD_SIZE_8BIT, calib_data, 24, HAL_MAX_DELAY) != HAL_OK)
            return HAL_ERROR;
    } else { // SPI
        HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_RESET);
        uint8_t tx_buffer[1] = { 0x88 | 0x80 }; // Read starting at 0x88
        HAL_StatusTypeDef status = HAL_SPI_Transmit(bmp->hspi, tx_buffer, 1, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_SET);
            return status;
        }
        status = HAL_SPI_Receive(bmp->hspi, calib_data, 24, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(bmp->cs_port, bmp->cs_pin, GPIO_PIN_SET);
        if (status != HAL_OK)
            return status;
    }

    // Parse calibration data
    bmp->dig_T1 = (uint16_t)(calib_data[1] << 8 | calib_data[0]);
    bmp->dig_T2 = (int16_t)(calib_data[3] << 8 | calib_data[2]);
    bmp->dig_T3 = (int16_t)(calib_data[5] << 8 | calib_data[4]);

    bmp->dig_P1 = (uint16_t)(calib_data[7] << 8 | calib_data[6]);
    bmp->dig_P2 = (int16_t)(calib_data[9] << 8 | calib_data[8]);
    bmp->dig_P3 = (int16_t)(calib_data[11] << 8 | calib_data[10]);
    bmp->dig_P4 = (int16_t)(calib_data[13] << 8 | calib_data[12]);
    bmp->dig_P5 = (int16_t)(calib_data[15] << 8 | calib_data[14]);
    bmp->dig_P6 = (int16_t)(calib_data[17] << 8 | calib_data[16]);
    bmp->dig_P7 = (int16_t)(calib_data[19] << 8 | calib_data[18]);
    bmp->dig_P8 = (int16_t)(calib_data[21] << 8 | calib_data[20]);
    bmp->dig_P9 = (int16_t)(calib_data[23] << 8 | calib_data[22]);

    return HAL_OK;
}

// Initialization Function
HAL_StatusTypeDef bmp280_init(bmp280_t *bmp) {
    HAL_StatusTypeDef status;

    // Verify communication mode
    if (bmp->comm_mode == BMP280_MODE_I2C && bmp->hi2c == NULL) {
        return HAL_ERROR;
    }
    if (bmp->comm_mode == BMP280_MODE_SPI && (bmp->hspi == NULL || bmp->cs_port == NULL)) {
        return HAL_ERROR;
    }

    // Read Chip ID
    uint8_t chip_id;
    status = bmp280_read_registers(bmp, BMP280_REG_CHIPID, &chip_id, 1);
    if (status != HAL_OK || chip_id != BMP280_CHIPID) {
        return HAL_ERROR;
    }

    // Read calibration data
    status = bmp280_read_calibration_data(bmp);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    // Set default configuration: Normal mode, oversampling x1, filter off, standby time 1000ms
    status = bmp280_set_configuration(bmp, BMP280_MODE_NORMAL, BMP280_OSAMPLE_1, BMP280_OSAMPLE_1, BMP280_FILTER_OFF, BMP280_STANDBY_1000_MS);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Soft Reset Function
HAL_StatusTypeDef bmp280_soft_reset(bmp280_t *bmp) {
    HAL_StatusTypeDef status = bmp280_write_register(bmp, BMP280_REG_RESET, BMP280_SOFT_RESET_CMD);
    HAL_Delay(10); // Wait for reset to complete
    return status;
}

// Set Configuration Function
HAL_StatusTypeDef bmp280_set_configuration(bmp280_t *bmp, bmp280_operating_mode_t mode,
                                           bmp280_oversampling_t osrs_t, bmp280_oversampling_t osrs_p,
                                           bmp280_filter_t filter, bmp280_standby_time_t standby) {
    uint8_t ctrl_meas = 0;
    uint8_t config = 0;

    // Configure CTRL_MEAS register
    ctrl_meas |= (osrs_t << 5); // Temperature oversampling
    ctrl_meas |= (osrs_p << 2); // Pressure oversampling
    ctrl_meas |= mode;          // Mode

    // Configure CONFIG register
    config |= (filter << 2);    // Filter
    config |= (standby << 5);   // Standby time

    // Write to CTRL_MEAS register
    if (bmp280_write_register(bmp, BMP280_REG_CTRL_MEAS, ctrl_meas) != HAL_OK)
        return HAL_ERROR;

    // Write to CONFIG register
    if (bmp280_write_register(bmp, BMP280_REG_CONFIG, config) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

// Read Raw Data Function
HAL_StatusTypeDef bmp280_read_raw(bmp280_t *bmp, int32_t *temperature_raw, int32_t *pressure_raw) {
    uint8_t data[6];
    HAL_StatusTypeDef status;

    // Read temperature and pressure data
    status = bmp280_read_registers(bmp, BMP280_REG_PRESS_MSB, data, 6);
    if (status != HAL_OK)
        return HAL_ERROR;

    *pressure_raw = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)(data[2] >> 4));
    *temperature_raw = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)(data[5] >> 4));

    return HAL_OK;
}

// Temperature Compensation Function
float bmp280_compensate_temperature(bmp280_t *bmp, int32_t temperature_raw) {
    float var1, var2, T;
    static float T_fine;

    var1 = (((float)temperature_raw) / 16384.0 - ((float)bmp->dig_T1) / 1024.0) * ((float)bmp->dig_T2);
    var2 = ((((float)temperature_raw) / 131072.0 - ((float)bmp->dig_T1) / 8192.0) *
            (((float)temperature_raw) / 131072.0 - ((float)bmp->dig_T1) / 8192.0)) *
           ((float)bmp->dig_T3);
    T_fine = var1 + var2;
    T = T_fine / 5120.0;
    return T;
}

// Pressure Compensation Function
float bmp280_compensate_pressure(bmp280_t *bmp, int32_t pressure_raw) {
    float var1, var2, p;
    var1 = ((float)T_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((float)bmp->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((float)bmp->dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((float)bmp->dig_P4) * 65536.0);
    var1 = (((float)bmp->dig_P3) * var1 * var1 / 524288.0 + ((float)bmp->dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((float)bmp->dig_P1);
    if (var1 == 0.0)
        return 0; // Avoid division by zero
    p = 1048576.0 - (float)pressure_raw;
    p = ((p - (var2 / 4096.0)) * 6250.0) / var1;
    var1 = ((float)bmp->dig_P9) * p * p / 2147483648.0;
    var2 = p * ((float)bmp->dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((float)bmp->dig_P7)) / 16.0;
    return p / 100.0; // Convert to hPa
}

// Altitude Calculation Function
float bmp280_calculate_altitude(float pressure, float sea_level_pressure) {
    return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
}

