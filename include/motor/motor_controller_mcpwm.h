#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_prelude.h"
#include "state.h"

/**
 * @brief Initializes the n motors timers, operators and comparators.
 */
void motor_controller_init(void);

/**
 * @brief update the motor pwm values with proper conversion from pid out
 * to motor pwm and correct resolution. 
 * @param ds drone state (throttle, yaw and pid out values -> roll pitch)
 */
void motor_controller(State *ds);

/**
 * @brief Proper shutdown procedure for all motors. 
 */
void motor_off(void);