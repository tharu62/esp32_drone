#pragma once

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"

#include "state.h"
#include "filter.h"

/**
 * @brief Read <len> bytes from a MPU6050 sensor registers at address <reg_addr> and write them into <data>.
 */
esp_err_t bmi088_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Write 1 byte to a MPU6050 sensor register at address <reg_addr> into <data>.
 */
esp_err_t bmi088_register_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

/**
 * @brief i2c master bus initialization
 */
void spi_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Calibration function for MPU6050, calculates offsets for accelerometer mesurements.
 *        !!!Requires stable platform. 
 */
void bmi088_calibrate(i2c_master_dev_handle_t dev_handle, KF* ekf);

/**
 * @brief Setup function for MPU6050 by :
 *       1. Wake up the sensor.
 *       2. Read and log the WHO_AM_I register.
 *       3. Set the sample rate.
 *       4. Configure the accelerometer range. (±2g)
 *       5. Configure the gyroscope range. (250°/s)
 */
void bmi088_setup(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Read accelerometer and gyroscope data from MPU6050, apply calibration offsets, 
 *        and update the state with accelerations and angular velocities.
 */
void bmi088_update(i2c_master_dev_handle_t dev_handle, i2c_master_bus_handle_t bus_handle, State *state, float dt);





