/**
 * 
 */
#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

#include <math.h>
#include "state.h"

/**
 * @brief Initializes the pid integral temrs and previous error values to zero.
 * This should be called once at the start of the program to ensure the controller starts with a clean state.
 */
void pid_init(void);

/**
 * @brief Implements the PID controller for angle control.
 * @note This function saves the previous state of the controller autonomously, 
 *       so it should be called in a loop with consistent time steps (dt) to function correctly.
 * @param drone_state Pointer to the drone's state.
 * @param dt Time step for the controller.
 */
void pid(State* drone_state, float dt);

#endif // ANGLE_CONTROLLER_H