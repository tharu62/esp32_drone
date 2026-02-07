/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/* Simple Firmaware for ESP32

   This code initializes the I2C bus and communicates with a MPU6050 sensor
   to read accelerometer and gyroscope data in a loop.

*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

#include "state.h"
#include "mpu6050.h"
#include "angle_controller.h"
#include "motor_controller.h"
#include "kalman_filter.h"
#include "input_receiver.h"

static const char *TAG = "drone";

#define DEBUG // Comment this out to disable debug features (MPU6050, PID, Kalman filter)

/**
 * @brief Main application
 */
void app_main(void)
{
    uint8_t data[10]; // Buffer for I2C data, oversized for safety.
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    State drone_state;
    init_state(&drone_state); // Zeroes roll and pitch angles

#ifdef DEBUG
    mpu6050_setup(dev_handle, data);
    mpu6050_calibration(dev_handle, data); // Calibrate the MPU6050 to get accurate readings
    angle_controller_init();
    motor_controller_init();
    reset_kalman_filter();
#endif
    
    init_espnow();

    float desired_angles[2] = {0.0f, 0.0f};                 // Angles given by user (roll, pitch)
    float throttle = 100.0f;                                // Throttle input from user (0-100%)
    float pid_angle_error[2] = {0.0f, 0.0f};                // Placeholder for angle controller outputs

    float dt = 0.01f; // Time step for control loop (5 ms)
    float t_start = 0.0f;
    float t_end = 0.0f;
    float elapsed = 0.0f;
    // Main control loop
    while (true) {
        t_start = esp_timer_get_time() / 1000.0f; // Get current time in milliseconds

        // Read Input from user (desired angles and throttle)
        get_control_inputs(&throttle, &desired_angles[0], &desired_angles[1]);

        if (throttle < 0.0f) break; // Exit loop if throttle is negative (used as a signal to stop)

#ifndef DEBUG        
        printf("%f,%f,%f\n", drone_state.k_angle[0], drone_state.k_angle[1], throttle);
#endif

#ifdef DEBUG        
        // Run angle controller pid to get desired rotation rates
        angle_pid_controller(desired_angles, &drone_state, dt, pid_angle_error);

        // Update motor speeds by pwm signals (temporarely disabled for testing)
        motor_controller(throttle, pid_angle_error);

        // Read sensor data & Run Kalman filter to update state estimation
        mpu6050_get_angle(dev_handle, bus_handle, data,&drone_state);
        mpu6050_get_rotation_rate(dev_handle, bus_handle, data, &drone_state, dt);
        kalman_filter(&drone_state, dt); 
#endif
        // Make sure to run at fixed time interval
        t_end = esp_timer_get_time() / 1000.0f; // Get current time in milliseconds
        elapsed = t_end - t_start;
        if (elapsed < dt) {
            vTaskDelay((dt - elapsed) / portTICK_PERIOD_MS); // Delay to maintain loop timing
        }
    }

    // Cleanup
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT)); /* Resetting the MPU6050 */
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
    ESP_LOGI(TAG, "Drone OFF.");
}


