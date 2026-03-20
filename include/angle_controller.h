/**
 * 
 */
#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

#include "state.h"

/**
 * @brief Initializes the pid integral temrs and previous error values to zero.
 * This should be called once at the start of the program to ensure the controller starts with a clean state.
 */
void angle_controller_init(void);

/**
 * @brief Implements the PID controller for angle control.
 * @note This function saves the previous state of the controller autonomously, 
 *       so it should be called in a loop with consistent time steps (dt) to function correctly.
 * @param desired_angles Array of desired angles (roll, pitch, yaw).
 * @param drone_state Pointer to the drone's state.
 * @param dt Time step for the controller.
 * @param pid_angle_error Array to store the PID angle errors.
 */
void angle_pid_controller(float* desired_angles, State* drone_state, float dt, float* pid_angle_error);

#endif // ANGLE_CONTROLLER_H