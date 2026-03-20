/**
 * Motor control implementation,
 * handles motor speed adjustments based on input commands.
 * 
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

/**
 * @brief Initialize ledc_timer and ledc_channel (PWM setup).
 */
void motor_controller_init(void);

/**
 * @brief Control motors based on input commands by adjusting PWM duty cycles for quadcopter logic.
 *        Calls motor_set_speed_percent() to apply the changes.
 * @param throttle User input throttle (0-100%)
 * @param pid_angle_error Array of PID angle errors for roll and pitch (output from angle controller)
 */
void motor_controller(float throttle, float* pid_angle_error);

#endif // MOTOR_H