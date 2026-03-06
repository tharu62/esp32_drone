#ifndef MPU6050_H
#define MPU6050_H

#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "state.h"

/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Write 1 byte to a MPU6050 sensor register
 */
esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

/**
 * @brief i2c master bus initialization
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Calibration function for MPU6050, calculates offsets for accelerometer and gyroscope.
 *        Requires stable platform. 
 */
void mpu6050_calibrate(i2c_master_dev_handle_t dev_handle, uint8_t *data);

/**
 * @brief Setup function for MPU6050 by :
 *       1. Wake up the sensor
 *       2. Read and log the WHO_AM_I register
 *       3. Set the sample rate
 *       4. Configure the accelerometer range (±2g)
 *       5. Configure the gyroscope range (250°/s)
 */
void mpu6050_setup(i2c_master_dev_handle_t dev_handle, uint8_t *data);

/**
 * @brief Read accelerometer and gyroscope data from MPU6050, apply calibration offsets, 
 *        and update the state with accelerations and angular velocities.
 */
void mpu6050_update(i2c_master_dev_handle_t dev_handle, i2c_master_bus_handle_t bus_handle, uint8_t *data, State *state, float dt);

#endif // MPU6050_H