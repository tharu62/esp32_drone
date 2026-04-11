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
#include "input_receiver.h"
#include "angle_controller.h"
#include "motor_controller.h"
#include "mpu6050.h"
#include "kalman_filter.h"

static const char *TAG = "drone";

#define MAIN_LOOP // Comment this out to disable main control loop (for testing...)
#define DEBUG     // Comment this out to disable debug features (MPU6050, PID, Kalman filter)


/**
 * @brief Main application
 */
void app_main(void)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    State drone_state;
    init_state(&drone_state);
    
    mpu6050_setup(dev_handle);
    // mpu6050_calibrate(dev_handle); // unused
    
    angle_controller_init();
    motor_controller_init();
    // reset_kalman_filter(); 
    
    // init_espnow();

    ESP_LOGI(TAG, "Drone ON.");
    
    // Time step for control loop in microseconds (5 us = 0.005 ms => 200 kHz control loop)
    const int64_t dt = 5; 
    float dt_sec = 0.0f;
    int64_t t_start, t_end = 0.0f;
    t_start = esp_timer_get_time(); // Get current time in microseconds
    // MAIN CONTROL LOOP
    while (true) {
#ifdef MAIN_LOOP        

        // Get current time in microseconds at the end of the loop to calculate elapsed time
        t_end = esp_timer_get_time();

        if ((t_end - t_start) >= dt) {

            // Calculate elapsed time in seconds
            dt_sec = (float)(t_end - t_start) / 1000000.0f; 

            // Reset start time for next loop iteration
            t_start = esp_timer_get_time();

            // Read Input from user (desired angles and throttle)
            get_control_inputs(&drone_state);

            // Exit loop if throttle is negative (used as a signal to stop)
            if (drone_state.throttle < 0.0f) break; 

            // Run angle controller pid to get desired rotation rates
            angle_pid_controller(&drone_state, dt_sec);

            //@todo : adding roll, pitch and yaw in the motor control logic.
            // Update motor speeds by pwm signals 
            motor_controller(&drone_state);

            // Read sensor data from MPU6050 and update drone state
            mpu6050_update(dev_handle, bus_handle, &drone_state, dt_sec);

            // Run Kalman filter to update state estimation
            kalman_filter(&drone_state, dt_sec); 
        }
#endif
    
#ifdef DEBUG        
        printf("%f,%f,%f\n", drone_state.k_angle[0], drone_state.k_angle[1], drone_state.k_angle[2]);
        // printf("%f\n", drone_state.throttle);
        // printf("%f\n", dt_sec);
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


