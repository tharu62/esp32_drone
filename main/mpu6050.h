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
 * @brief i2c master initialization
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Calibration function for MPU6050, calculates offsets for gyroscope. 
 */
void mpu6050_calibration(i2c_master_dev_handle_t dev_handle, uint8_t *data);

/**
 * @brief Setup function for MPU6050 by :
 *       1. Waking up the sensor
 *       2. Reading and logging the WHO_AM_I register
 *       3. Setting the sample rate
 *       4. Configuring the accelerometer range (±8g)
 * 
 */
void mpu6050_setup(i2c_master_dev_handle_t dev_handle, uint8_t *data);

/**
 * @brief Get roll, pitch, and yaw angles from accelerometer data
 */
void mpu6050_get_angle(i2c_master_dev_handle_t dev_handle, i2c_master_bus_handle_t bus_handle, uint8_t *data, State *state);

/**
 * @brief Get rotation rates from gyroscope data
 */
void mpu6050_get_rotation_rate(i2c_master_dev_handle_t dev_handle, i2c_master_bus_handle_t bus_handle, uint8_t *data, State *state, float dt);

#endif // MPU6050_H