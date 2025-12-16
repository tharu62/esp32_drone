/*************************************************************************************************************************************
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 * 
 * This firmware, for an esp32 board running on a 4 motor drone,
 * is responsible of stabilization and speed control of the drone 
 * and also receiving and executing the commands from the user.   
 * The use of this software is allowed by anyone in any capacity for free. 
 *          
 *              The author, 
 *                  Deshan Tharindu Edirisinghe Edirisinghe Mudiyanselage.
 */

/************************************************************* LIBS *****************************************************************/
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "esp_log.h"

#include "mpu6050.h"

/*********************************************************** CONSTANTS *************************************************************/

// TEST MOTOR
#define IN2 25 
#define IN1 26 

// MPU6050 COM
#define I2C_NUM I2C_NUM_0       
#define I2C_SCL GPIO_NUM_18     
#define I2C_SDA GPIO_NUM_19     
#define MPU_ADDR 0x68

// PWM 
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (33) 
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT  
#define LEDC_DUTY               (4096)             
#define LEDC_CLK_SRC            LEDC_AUTO_CLK
#define LEDC_FREQUENCY          (4000)             

/****************************************************** FUNCTION DECLARATIONS ********************************************************/

static esp_err_t i2c_master_init(void); 
static void pwm_init(void);

esp_err_t mpu_read_register(uint8_t reg, uint8_t* read_buffer, size_t len);
esp_err_t mpu_write_register(uint8_t reg, uint8_t data);
void mpu6050_setup();
bool input_from_user();

/************************************************************************************************************************************************/

void app_main(void)
{
    printf("ESP32 BOARD STARTED!");
    fflush(stdout);
    
    /********************* MPU6050 SET UP ***************************/
    mpu6050_setup();

    uint8_t data[10];

    while(true){

        /************* ACCELEROMETER *************/
        mpu_read_register(0x3B, data, 6);

        int16_t RAWX = (data[0]<<8) | data[1];
        int16_t RAWY = (data[2]<<8) | data[3];
        int16_t RAWZ = (data[4]<<8) | data[5];

        // UPDATED FOR ±8g RANGE  → 4096 LSB/g
        float xg = (float) RAWX / 4096.0f;
        float yg = (float) RAWY / 4096.0f;
        float zg = (float) RAWZ / 4096.0f;

        ESP_LOGI("acceleration : ", "x=%f y=%f z=%f", xg, yg, zg);


        /************* GYROSCOPE *************/
        mpu_read_register(0x43, data, 6);

        int16_t GYRO_RAWX = (data[0] << 8) | data[1];
        int16_t GYRO_RAWY = (data[2] << 8) | data[3];
        int16_t GYRO_RAWZ = (data[4] << 8) | data[5];

        float gyroX = (float) GYRO_RAWX / 131.0f;
        float gyroY = (float) GYRO_RAWY / 131.0f;
        float gyroZ = (float) GYRO_RAWZ / 131.0f;

        ESP_LOGI("gyroscope : ", "x=%f y=%f z=%f deg/s", gyroX, gyroY, gyroZ);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    return;
}

/**
 * 
 */
bool input_from_user()
{
    return false;
}

/**
 * 
 */
esp_err_t mpu_read_register(uint8_t reg, uint8_t* read_buffer, size_t len) 
{
    return (i2c_master_write_read_device(I2C_NUM, MPU_ADDR, &reg, 1, read_buffer, len , 2000));
}

esp_err_t mpu_write_register(uint8_t reg, uint8_t data)
{
    uint8_t write_buffer[2];
    write_buffer[0] = reg;
    write_buffer[1] = data;
    return (i2c_master_write_to_device(I2C_NUM, MPU_ADDR, write_buffer, 2, 1000));
}

/**
 * Initilize the i2c port on come specific gpio ports.
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM;
    i2c_config_t conf = {
        .mode =  I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

/**
 * 
 */
void mpu6050_setup()
{
    uint8_t data[2];

    ESP_ERROR_CHECK(i2c_master_init());

    mpu_read_register(0x75, data, 1);
    ESP_LOGI("mpu6050 id check : " ,"%X\n", data[0]);

    mpu_write_register(0x6B, 0);     // Wake up MPU6050
    mpu_write_register(0x19, 7);     // Sample rate divider
   
    // NEW: Set accelerometer to ±8g
    mpu_write_register(0x1C, 0x10);  // AFS_SEL = 2 → ±8g
}

/**
 * 
 */
static void pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_CLK_SRC,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
