#include "angle_controller.h"

#define P_ROLL_ANGLE 0.6f
#define I_ROLL_ANGLE 1.f
#define D_ROLL_ANGLE 0.03f

#define P_PITCH_ANGLE 0.6f
#define I_PITCH_ANGLE 1.0f
#define D_PITCH_ANGLE 0.03f


float previous_roll_angle_error = 0.0f;
float previous_pitch_angle_error = 0.0f;

float roll_angle_integral = 0.0f;
float pitch_angle_integral = 0.0f;


void angle_controller_init(void) 
{
    previous_roll_angle_error = 0.0f;
    previous_pitch_angle_error = 0.0f;
    roll_angle_integral = 0.0f;
    pitch_angle_integral = 0.0f;
    return;
}


void angle_pid_controller(float* desired_angles, State* drone_state, float dt, float* pid_angle_error)
{
    float roll_angle_error  = desired_angles[0] - drone_state->k_angle[0];
    float pitch_angle_error = desired_angles[1] - drone_state->k_angle[1];
    roll_angle_integral  +=  roll_angle_error;
    pitch_angle_integral +=  pitch_angle_error;
    
    float roll_angle_derivative  = (roll_angle_error  - previous_roll_angle_error ) / dt;
    float pitch_angle_derivative = (pitch_angle_error - previous_pitch_angle_error) / dt;
    
    pid_angle_error[0] = P_ROLL_ANGLE  * roll_angle_error  + I_ROLL_ANGLE  * roll_angle_integral  + D_ROLL_ANGLE  * roll_angle_derivative;
    pid_angle_error[1] = P_PITCH_ANGLE * pitch_angle_error + I_PITCH_ANGLE * pitch_angle_integral + D_PITCH_ANGLE * pitch_angle_derivative;
    
    previous_roll_angle_error  = roll_angle_error;
    previous_pitch_angle_error = pitch_angle_error;
    return;
}