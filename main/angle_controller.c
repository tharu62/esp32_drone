#include "angle_controller.h"

// Rate PID (inner loop)
#define RATE_KP_ROLL    0.05f
#define RATE_KI_ROLL    0.5f
#define RATE_KD_ROLL    0.05f

#define RATE_KP_PITCH   0.05f
#define RATE_KI_PITCH   0.5f
#define RATE_KD_PITCH   0.05f

#define MAX_RATE        3.0f
#define K               0.3f         

#define I_LIMIT         0.5f
#define OUTPUT_LIMIT    1.0f

static float roll_rate_integral = 0.0f;
static float pitch_rate_integral = 0.0f;

inline void clamp(float* value)
{
    if(*value > OUTPUT_LIMIT) {
        *value = OUTPUT_LIMIT;
    }
    if(*value < -OUTPUT_LIMIT) {
        *value = -OUTPUT_LIMIT;
    }
}

inline void clamp_I(float* value)
{
    if(*value > I_LIMIT) {
        *value = I_LIMIT;
    }
    if(*value < -I_LIMIT) {
        *value = -I_LIMIT;
    }
}

inline float GAIN(float error) {
    return MAX_RATE * tanhf(K * error);
}


void pid_angle_to_rate(State* s, float dt)
{
    // new angle error 
    float roll_error  = s->d_angle[0] - s->k_angle[0];
    float pitch_error = s->d_angle[1] - s->k_angle[1];

    // conversion from angle error to desired angular velocity
    float roll_rate_setpoint  = GAIN(roll_error);
    float pitch_rate_setpoint = GAIN(pitch_error);

    // new angular velocity error
    float roll_rate_error  = roll_rate_setpoint  - s->w[0];
    float pitch_rate_error = pitch_rate_setpoint - s->w[1];

    // integrate angular velocities
    roll_rate_integral  += roll_rate_error  * dt;
    pitch_rate_integral += pitch_rate_error * dt;

    // integration limits to prevent excessive control signals [-I_LIMIT, I_LIMIT] degree/sec
    clamp_I(&roll_rate_integral);
    clamp_I(&pitch_rate_integral);

    // derivative (using negative measured rate for better response)
    float roll_derivative  = -s->w[0];
    float pitch_derivative = -s->w[1];

    // upate PID outputs for roll and pitch
    float roll_output = RATE_KP_ROLL * roll_rate_error + RATE_KI_ROLL * roll_rate_integral + RATE_KD_ROLL * roll_derivative;
    float pitch_output = RATE_KP_PITCH * pitch_rate_error + RATE_KI_PITCH * pitch_rate_integral + RATE_KD_PITCH * pitch_derivative;

    // output limits to prevent excessive control signals [-OUTPUT_LIMIT, OUTPUT_LIMIT] degree/sec
    clamp(&roll_output);
    clamp(&pitch_output);

    // update the state with the PID outputs for roll and pitch
    s->pid_output[0] = roll_output;
    s->pid_output[1] = pitch_output;
}

#define KP 0.1f  // bigger KP => more aggressive response, but more overshoot and potential instability. 
#define KI 0.5f  // bigger KI => faster response, but more overshoot and potential instability.
#define KD 0.01f  // bigger KD => more damping, less overshoot, but slower response.

static float roll_integral = 0.0f;
static float pitch_integral = 0.0f;

static float last_roll_error = 0.0f;
static float last_pitch_error = 0.0f;

void pid_angle(State* s, float dt)
{
    // new angle error. 
    float roll_error  = s->d_angle[0] - s->k_angle[0];
    float pitch_error = s->d_angle[1] - s->k_angle[1];

    // integrate angle error.
    roll_integral  += roll_error;
    pitch_integral += pitch_error;

    // integration limits to prevent excessive control signals [-I_LIMIT, I_LIMIT] degree.
    clamp_I(&roll_integral);
    clamp_I(&pitch_integral);

    // derivative
    float roll_derivative  = (last_roll_error - roll_error) / dt;
    float pitch_derivative = (last_pitch_error - pitch_error) / dt;

    // upate PID outputs for roll and pitch
    float roll_output = KP * roll_error + KI * roll_integral + KD * roll_derivative;
    float pitch_output = KP * pitch_error + KI * pitch_integral + KD * pitch_derivative;

    // output limits to prevent excessive control signals [-OUTPUT_LIMIT, OUTPUT_LIMIT] degree.
    clamp(&roll_output);
    clamp(&pitch_output);

    // update the state with the PID outputs for roll and pitch.
    s->pid_output[0] = roll_output;
    s->pid_output[1] = pitch_output;

    // update last roll and pitch error.
    last_roll_error = roll_error;
    last_pitch_error = pitch_error;
}