/**
 * 
 */
#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

#include <math.h>
#include "state.h"

/**
 * @brief Implements the PID controller for angle error control by using a two 
 *        layer loop. The outer loop is the pure angle error controller and the inner loop is the 
 *        the pid on the angular velocities on roll and pitch converted from angle errors.
 * @note This function saves the previous state of the controller autonomously, 
 *       so it should be called in a loop with consistent time steps (dt) to function correctly.
 * @param drone_state Pointer to the drone's state.
 * @param dt Time step for the controller.
 */
void pid_angle_to_rate(State* drone_state, float dt);

/**
* @brief Implements the PID controller for angle error control.
 * @note This function saves the previous state of the controller autonomously, 
 *       so it should be called in a loop with consistent time steps (dt) to function correctly.
 * @param drone_state Pointer to the drone's state.
 * @param dt Time step for the controller.
 */
void pid_angle(State* s, float dt);

#endif // ANGLE_CONTROLLER_H