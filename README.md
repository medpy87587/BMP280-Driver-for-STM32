# BMP280-Driver-for-STM32
This repository contains a C driver for the BMP280 sensor, compatible with STM32 microcontrollers using either I2C or SPI communication.
## Features

- I2C and SPI support
- Read temperature, pressure, and calculate altitude
- Compensation for temperature and pressure using sensor calibration data

## Files

- **BMP280.c**: Contains functions to interface with the BMP280 sensor.
- **BMP280.h**: Header file with function declarations and macros.

## Usage

### Initialization
```c
bmp280_init(&bmp);
Read Sensor Data

int32_t temp_raw, press_raw;
bmp280_read_raw(&bmp, &temp_raw, &press_raw);
float temperature = bmp280_compensate_temperature(&bmp, temp_raw);
float pressure = bmp280_compensate_pressure(&bmp, press_raw);
```
## Calculate Altitude
```c
float altitude = bmp280_calculate_altitude(pressure, SEA_LEVEL_PRESSURE);
```
## Communication Modes
I2C: Provide hi2c handle and I2C address.
SPI: Provide hspi handle, CS GPIO port, and pin.

## Contributing
Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.
