/* Simple Firmaware for ESP32 DRONE CONTROLLER

   This code initializes the I2C bus and communicates 
   with a imu sensor to read accelerometer and 
   gyroscope data in a loop. 
   ...
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "driver/i2c_master.h"

#include "state.h"
#include "input_receiver.h"
#include "angle_controller.h"
// #include "motor_controller.h"
#include "motor_controller_mcpwm.h"
#include "bmi088.h"
#include "mpu6050.h"
#include "filter.h"

static const char *TAG = "drone";

#define MAIN_LOOP // Comment this out to disable main control loop (for testing...)
// #define DEBUG     // Comment this out to disable debug features (MPU6050, PID, Kalman filter)

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
    KF kf; 
    init_kf(&kf);
    motor_controller_init();

    mpu6050_setup(dev_handle);

    // blocking function waiting
    // for user input to give time
    // to set up drone for calibration
    init_espnow();
    
    mpu6050_calibrate(dev_handle, &kf); 

    // Delay of 5 seconds to allow user 
    // to prepare for takeoff after calibration 
    // and to ensure stable esc arming before 
    // starting the control loop.
    vTaskDelay((3000) / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Drone ON.");

#ifdef MAIN_LOOP

    // Time step for control loop in microseconds (500 us = 0.5 ms => 2 kHz control loop)
    const int64_t dt = 500; 
    float dt_sec = 0.0f;
    int64_t t_start, t_end;
    t_start = esp_timer_get_time(); // Get timestamp at control loop start
    float dt_loop = 0; // Variable to track actual loop time for debugging
    while (true) {
        // Get current time in microseconds to check if dt has elapsed
        t_end = esp_timer_get_time();

        if ((t_end - t_start) >= dt) {

            // Calculate elapsed time in seconds for control algorithms
            dt_sec = (float)dt / 1000000.0f;

            // Keep fixed loop timing (prevents drift)
            t_start += dt;

        #ifdef DEBUG  
            // Start execution timing
            dt_loop = (float)esp_timer_get_time() / 1000000.0f;
        #endif

            // Read sensor data from MPU6050 and update drone state (no filtering)
            mpu6050_update(dev_handle, bus_handle, &drone_state, dt_sec);

            // Run Filter to update state estimation
            kalman_filter(&drone_state, &kf, dt_sec); 
            // complementary_filter(&drone_state, dt_sec); 

            // Read Input from user (desired angles and throttle)
            get_control_inputs(&drone_state);

            // Exit loop if throttle is negative (used as a signal to stop the drone)
            if (drone_state.throttle < 0.0f) break; 

            // Run angle controller pid to get proper motor input
            pid_angle(&drone_state, dt_sec);
            // pid_angle_to_rate(&drone_state, dt_sec);

            // Update motor speeds by pwm signals 
            motor_controller(&drone_state);

            
        #ifdef DEBUG        
        
            // Calculate actual loop execution time for debugging
            dt_loop = (float)(esp_timer_get_time() / 1000000.0f) - dt_loop;
            
            // printf("%f,%f,%f\n", drone_state.k_angle[0], drone_state.k_angle[1], drone_state.k_angle[2]);
            // printf("%f\n", drone_state.throttle);

            // Print less frequently to avoid disturbing loop timing
            static int debug_counter = 0;

            if (++debug_counter >= 100) {
                debug_counter = 0;

                printf("dt_sec=%f\n", dt_sec);
                printf("dt_loop=%f\n", dt_loop);
            }

            // printf("looping...\n");
            // vTaskDelay((500) / portTICK_PERIOD_MS); // Delay to maintain loop timing
        #endif
        }
    }

#endif

    // Cleanup
    motor_off();
    // ESP_ERROR_CHECK(mpu6050_register_write(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT)); /* Resetting the MPU6050 */
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
    ESP_LOGI(TAG, "Drone OFF.");
    esp_deep_sleep_start();
}


