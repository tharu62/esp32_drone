#include "kalman_filter.h" 

#define STANDARD_DEV_ACCEL_NOISE_SQRD 3.0f // Standard deviation squared of acceleration noise (deg) 
#define STANDARD_DEV_GYRO_NOISE_SQRD 3.0f  // Standard deviation squared of gyro 

float k_uncertainty = 0.0f; // Initial estimation uncertainty for roll 
float k_gain = 0.0f;        // Kalman gain for roll 

static float integrated_roll = 0.0f;  // Integrated roll angle from gyro
static float integrated_pitch = 0.0f; // Integrated pitch angle from gyro

void reset_kalman_filter() { 
    k_uncertainty = 1.0f; 
    k_gain = 0.0f;
    integrated_roll = 0.0f;
    integrated_pitch = 0.0f; 
} 

inline void kalman_filter(State *drone_state, float dt) { 

    // Prediction step: integrate gyro angles to get angle estimates
    drone_state->k_angle[0] += drone_state->angular_velocity[0] * dt;
    drone_state->k_angle[1] += drone_state->angular_velocity[1] * dt;

    // Calculate Kalman gain
    k_uncertainty += dt * dt * STANDARD_DEV_GYRO_NOISE_SQRD;
    k_gain = k_uncertainty / (k_uncertainty + STANDARD_DEV_ACCEL_NOISE_SQRD);

    // Update angle by combining angle from accelerometer measurement and integrated gyro angle 
    // with Kalman gain on complementary filter logic 
    drone_state->k_angle[0] = k_gain * drone_state->m_angle[0] + (1 - k_gain) * drone_state->k_angle[0];
    drone_state->k_angle[1] = k_gain * drone_state->m_angle[1] + (1 - k_gain) * drone_state->k_angle[1];

    // Update uncertainty
    k_uncertainty *= (1 - k_gain);
}