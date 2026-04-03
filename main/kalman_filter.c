#include "kalman_filter.h" 

#define STANDARD_DEV_ACCEL_NOISE_SQRD 2.0f // Standard deviation squared of acceleration noise (deg) 
#define STANDARD_DEV_GYRO_NOISE_SQRD 4.0f  // Standard deviation squared of gyro 

float k_uncertainty = 1.0f; // Initial estimation uncertainty for roll 
float k_gain = 0.0f;        // Kalman gain for roll 

static float integrated_roll = 0.0f;  // Integrated roll angle from gyro
static float integrated_pitch = 0.0f; // Integrated pitch angle from gyro

void reset_kalman_filter() { 
    k_uncertainty = 1.0f * 1.0f; 
    k_gain = 0.0f;
    integrated_roll = 0.0f;
    integrated_pitch = 0.0f; 
} 

void kalman_filter(State *state, float dt) { 
 
    // Predict roll angle by integrating angular velocity 
    integrated_roll += state->angular_velocity[0] * dt;
    integrated_pitch += state->angular_velocity[1] * dt;
    
    // Update uncertainty from rotation integration 
    k_uncertainty  += dt * dt * STANDARD_DEV_GYRO_NOISE_SQRD; 
 
    // Calculate Kalman gain for roll and pitch 
    k_gain  = k_uncertainty  / (k_uncertainty  + STANDARD_DEV_ACCEL_NOISE_SQRD); 

    // Update roll and pitch angle with measurement, weighted by Kalman gain
    // No yaw beacouse there is no controll over yaw movements. 
    state->k_angle[0] += k_gain  * (state->m_angle[0] - integrated_roll); 
    state->k_angle[1] += k_gain  * (state->m_angle[1] - integrated_pitch); 
    
    // Update uncertainty for roll and pitch 
    k_uncertainty *= (1 - k_gain); 
}