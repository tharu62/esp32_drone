#include "state.h"

void init_state(State *state) 
{
    state->throttle = 0.0f;
    for (int i = 0; i < 3; i++) {
        state->d_angle[i] = 0.0f;
        state->m_angle[i] = 0.0f;
        state->k_angle[i] = 0.0f;
        state->w[i] = 0.0f;
        state->a[i] = 0.0f;
        state->pid_output[i] = 0.0f;
    }
}

void update_state(State *state, float angle_roll, float angle_pitch, float gyro_roll, float gyro_pitch) 
{
    // Simple state update logic (to be replaced with actual Kalman filter logic)
    state->m_angle[0] = angle_roll;
    state->m_angle[1] = angle_pitch;
    state->w[0] = gyro_roll;
    state->w[1] = gyro_pitch;
}