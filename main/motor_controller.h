/**
 * Motor control implementation,
 * handles motor speed adjustments based on input commands.
 * 
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

// Initialize the motor controller variables.
void motor_controller_init(void);

void motor_set_speed_percent(void);

/**
 * @brief Control motors based on input commands
 * @param throttle 
 * @param rotation_rate_output
 */
void motor_controller(float throttle, float* rotation_rate_output);

#endif // MOTOR_H