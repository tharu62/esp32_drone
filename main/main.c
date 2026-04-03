/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
*/

/* Simple Firmaware for ESP32

   This code initializes the I2C bus and communicates with a MPU6050 sensor
   to read accelerometer and gyroscope data in a loop. 
   ...
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
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

#define MAIN_LOOP // Comment this out to disable main control loop (for testing...)
#define DEBUG     // Comment this out to disable debug features (MPU6050, PID, Kalman filter)


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
    
    mpu6050_setup(dev_handle, data);
    mpu6050_calibrate(dev_handle, data); // Calibrate sensor to get accurate readings
    
    angle_controller_init();
    motor_controller_init();
    // reset_kalman_filter();
    
    // init_espnow();

    ESP_LOGI(TAG, "Drone ON.");

    float throttle = 0.0f;                         // Throttle input from user (0-100%)
    float desired_angles[3] = {0.0f, 0.0f, 0.0f};  // Angles given by user (roll, pitch, yaw)
    float pid_angle_error[3] = {0.0f, 0.0f, 0.0f}; // Placeholder for angle controller outputs
    
    // Time step for control loop in microseconds (1000 us = 1 ms => 1 KHz control loop)
    int64_t dt = 1000; 
    float dt_sec = 0.0f; // Convert dt to seconds for use in calculations
    int64_t t_start, t_end = 0.0f;
    t_start = esp_timer_get_time(); // Get current time in microseconds
    // MAIN CONTROL LOOP
    while (true) {

#ifdef MAIN_LOOP        

        // Get current time in microseconds at the end of the loop to calculate elapsed time
        t_end = esp_timer_get_time();

        if ((t_end - t_start) >= dt) {

            dt_sec = (float)(t_end - t_start) / 1000000.0f; // Calculate elapsed time in seconds

            // Read Input from user (desired angles and throttle)
            get_control_inputs(&throttle, &desired_angles[0], &desired_angles[1]);

            // Exit loop if throttle is negative (used as a signal to stop)
            if (throttle < 0.0f) break; 

            // Run angle controller pid to get desired rotation rates
            angle_pid_controller(desired_angles, &drone_state, dt_sec, pid_angle_error);

            //@todo : adding roll, pitch and yaw in the motor control logic.
            // Update motor speeds by pwm signals 
            motor_controller(throttle, pid_angle_error);

            // Read sensor data & Run Kalman filter to update state estimation
            mpu6050_update(dev_handle, bus_handle, data, &drone_state, dt_sec);

            // @todo : calibrate kalman filter parameters for better performance.
            kalman_filter(&drone_state, dt_sec); 

            // Make sure to run at fixed time interval
            t_start = esp_timer_get_time(); // Get current time in microseconds
        }
#endif
    
#ifdef DEBUG        
        printf("%f,%f,%f\n", drone_state.k_angle[0], drone_state.k_angle[1], drone_state.k_angle[2]);
        // printf("%f\n", throttle);
        // printf("looping...\n");
        // vTaskDelay((500) / portTICK_PERIOD_MS); // Delay to maintain loop timing
#endif
    }

    // Cleanup
    // ESP_ERROR_CHECK(mpu6050_register_write(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT)); /* Resetting the MPU6050 */
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
    ESP_LOGI(TAG, "Drone OFF.");
}


