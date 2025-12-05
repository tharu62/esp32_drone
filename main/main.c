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

/*#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"*/


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
	while(true){
			
		// 1. check input from user
		if(input_from_user()){
			
		}
		
		// 2. if input is present -> set new state
		else {
			
		}
		
		// 3. else keep last state
		
		// 4. run pid controll using current state, sensor mesurement and last state.
		
		// 5. go back to (1)
	
		;;
	}
	
	
	
	return;
}


bool input_from_user(){
	
	
	
	return false;
}


