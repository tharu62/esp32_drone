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

/**
 * @brief Initialize ledc_timer and ledc_channel (PWM setup)
 */
void motor_controller_init(void);

/**
 * @brief Set motor speeds based on throttle and rotation rate outputs
 */
void motor_set_speed_percent(void);

/**
 * @brief Control motors based on input commands by adjusting PWM duty cycles for quadcopter logic.
 *        Calls motor_set_speed_percent() to apply the changes.
 * @param throttle User input throttle (0-100%)
 * @param rotation_rate_output Array of rotation rates for roll and pitch (output from angle controller)
 */
void motor_controller(float throttle, float* rotation_rate_output);

#endif // MOTOR_H