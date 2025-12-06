/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 * 
 * This software is a firmaware for an esp32 board running on a 4 motor drone.
 * It is responsible of stabilization and speed control of the drone 
 * and also receiving and executing the commands from the user.   
 * The use of this software is allowed by anyone in any capacity for free. 
 * 		
 *					The author, 
 * 					Deshan Tharindu Edirisinghe Edirisinghe Mudiyanselage.
 *
 */

#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_pm.h"
#include "esp_log.h"

#define IN2 25 // D25
#define IN1 26 // D26

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (33) // Define the output GPIO -> D33
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#if CONFIG_PM_ENABLE
#define LEDC_CLK_SRC            LEDC_USE_RC_FAST_CLK // choose a clock source that can maintain during light sleep
#define LEDC_FREQUENCY          (400) // Frequency in Hertz. Set frequency at 400 Hz
#else
#define LEDC_CLK_SRC            LEDC_AUTO_CLK
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz
#endif

/* Warning:
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2 (rev < 1.2), ESP32P4 (rev < 3.0) targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */
static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_CLK_SRC,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
#if CONFIG_PM_ENABLE
        .sleep_mode     = LEDC_SLEEP_MODE_KEEP_ALIVE,
#endif
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

typedef enum {
	UP = 0,
	DOWN = 1,
	RIGHT = 2,
	LEFT = 3,
	FORWARD = 4,
	BACK = 5
}cmd;

typedef struct {
	float pwm1;
	float pwm2;
	float pwm3;
	float pwm4;
	cmd user_input;
} state;

bool input_from_user();

void app_main(void)
{
    printf("ESP32 BOARD STARTED!");
    fflush(stdout);
	
	/** init tools :
		- init motors pwm
		- init mpu6050 sensor com
		- init user input com
	*/ 
	// main loop for drone controll and input read
	while(false){
		// 1. check input from user
		if(input_from_user()){}
		// 2. if input is present -> set new state
		else {}
		// 3. else keep last state
		// 4. run pid controll using current state, sensor mesurement and last state.
		// 5. go back to (1)
		;;
	}
	
	#if CONFIG_PM_ENABLE
	    esp_pm_config_t pm_config = {
	        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
	        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
	#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
	        .light_sleep_enable = true
	#endif
	    };
	    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
	#endif
    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
	
    // Configure both pins at once
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1) | (1ULL << IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);

    // Now set them high or low
    gpio_set_level(IN1, 1);
    gpio_set_level(IN2, 0);
	
	
	// TEST LOOP FOR MOTOR PWM CONTROL
	uint32_t duty_cycle = 4096;
	int increment = 0;
	while(true){
		
		/*vTaskDelay(10 / portTICK_PERIOD_MS); */  // 1000 ms delay
		
		// Set duty dinamically
		duty_cycle += increment;
		if (duty_cycle > 8000) {
			increment = -1;
		}
		if (duty_cycle < 1000) {
			increment = 1;
		}
    	ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle));
    	// Update duty to apply the new value
    	ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
	}
	
	return;
}


bool input_from_user(){
	
	
	
	return false;
}


