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