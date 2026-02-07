/**
 * 
 */
#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

#include "state.h"

void angle_controller_init(void);
void angle_pid_controller(float* desired_angles, State* drone_state, float dt, float* pid_angle_error);

#endif // ANGLE_CONTROLLER_H