#include "state.h"

void init_state(State *state) 
{
    for (int i = 0; i < 3; i++) {
        state->d_angle[i] = 0.0f;
        state->m_angle[i] = 0.0f;
        state->k_angle[i] = 0.0f;
    }
}

void update_state(State *state, float angle_roll, float angle_pitch, float gyro_roll, float gyro_pitch) 
{
    // Simple state update logic (to be replaced with actual Kalman filter logic)
    state->m_angle[0] = angle_roll;
    state->m_angle[1] = angle_pitch;
    state->angular_velocity[0] = gyro_roll;
    state->angular_velocity[1] = gyro_pitch;
}