#include <math.h>
#include "angle_controller.h"

// Angle PID (outer loop)
#define ANGLE_KP_ROLL   4.0f
#define ANGLE_KP_PITCH  4.0f

// Rate PID (inner loop)
#define RATE_KP_ROLL    0.08f
#define RATE_KI_ROLL    0.04f
#define RATE_KD_ROLL    0.002f

#define RATE_KP_PITCH   0.08f
#define RATE_KI_PITCH   0.04f
#define RATE_KD_PITCH   0.002f

#define I_LIMIT         0.3f
#define OUTPUT_LIMIT    1.0f


static float roll_rate_integral = 0.0f;
static float pitch_rate_integral = 0.0f;


void angle_controller_init(void)
{
    roll_rate_integral = 0.0f;
    pitch_rate_integral = 0.0f;
}


void angle_pid_controller(State* s, float dt)
{
    // new angle error 
    float roll_error  = s->d_angle[0] - s->k_angle[0];
    float pitch_error = s->d_angle[1] - s->k_angle[1];

    // initial roll and pitch rate setpoints from angle error,
    // the KP values determine how aggressively the controller 
    // tries to correct angle errors by setting rate targets, 
    // higher KP means more aggressive correction, but can lead to instability if too high.
    float roll_rate_setpoint  = ANGLE_KP_ROLL  * roll_error;
    float pitch_rate_setpoint = ANGLE_KP_PITCH * pitch_error;

    // measured angular velocities
    float roll_rate_error  = roll_rate_setpoint  - s->w[0];
    float pitch_rate_error = pitch_rate_setpoint - s->w[1];

    // integrate angular velocities
    roll_rate_integral  += roll_rate_error  * dt;
    pitch_rate_integral += pitch_rate_error * dt;

    // anti-windup [-0.3, 0.3] deg/sec
    if (roll_rate_integral > I_LIMIT) roll_rate_integral = I_LIMIT;
    if (roll_rate_integral < -I_LIMIT) roll_rate_integral = -I_LIMIT;
    if (pitch_rate_integral > I_LIMIT) pitch_rate_integral = I_LIMIT;
    if (pitch_rate_integral < -I_LIMIT) pitch_rate_integral = -I_LIMIT;

    // derivative (using negative measured rate for better response)
    float roll_derivative  = -s->w[0];
    float pitch_derivative = -s->w[1];

    // upate PID outputs for roll and pitch
    float roll_output = RATE_KP_ROLL * roll_rate_error + RATE_KI_ROLL * roll_rate_integral + RATE_KD_ROLL * roll_derivative;
    float pitch_output = RATE_KP_PITCH * pitch_rate_error + RATE_KI_PITCH * pitch_rate_integral + RATE_KD_PITCH * pitch_derivative;

    // output limits to prevent excessive control signals [-1.0, 1.0] degree/sec
    if (roll_output > OUTPUT_LIMIT) roll_output = OUTPUT_LIMIT;
    if (roll_output < -OUTPUT_LIMIT) roll_output = -OUTPUT_LIMIT;
    if (pitch_output > OUTPUT_LIMIT) pitch_output = OUTPUT_LIMIT;
    if (pitch_output < -OUTPUT_LIMIT) pitch_output = -OUTPUT_LIMIT;

    // update the state with the PID outputs for roll and pitch
    s->pid_output[0] = roll_output;
    s->pid_output[1] = pitch_output;
}