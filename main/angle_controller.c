#include "angle_controller.h"

#define I_LIMIT      0.4f
#define OUTPUT_LIMIT 0.5f

#define KP 0.1f
#define KI 0.4f
#define KD 0.02f

static float roll_integral = 0.0f;
static float pitch_integral = 0.0f;

static float last_roll_error = 0.0f;
static float last_pitch_error = 0.0f;

inline float clamp_fast(float x, float lim)
{
    return (x > lim) ? lim : (x < -lim) ? -lim : x;
}

void pid_angle(State* s, float dt)
{
    float inv_dt = 1.0f / dt;

    float roll_error  = s->d_angle[0] - s->k_angle[0];
    float pitch_error = s->d_angle[1] - s->k_angle[1];

    roll_integral  += roll_error;
    pitch_integral += pitch_error;

    roll_integral  = clamp_fast(roll_integral, I_LIMIT);
    pitch_integral = clamp_fast(pitch_integral, I_LIMIT);

    float roll_derivative  = (roll_error  - last_roll_error)  * inv_dt;
    float pitch_derivative = (pitch_error - last_pitch_error) * inv_dt;

    float roll_output = KP * roll_error + KI * roll_integral + KD * roll_derivative;

    float pitch_output = KP * pitch_error + KI * pitch_integral + KD * pitch_derivative;

    s->pid_output[0] = clamp_fast(roll_output, OUTPUT_LIMIT);
    s->pid_output[1] = clamp_fast(pitch_output, OUTPUT_LIMIT);

    last_roll_error = roll_error;
    last_pitch_error = pitch_error;
}