#include "kalman_filter.h"

// Measurement noise (deg²)
#define STANDARD_DEV_ACCEL_NOISE_SQRD (10.0f * 10.0f)

// Process noise
#define Q_ANGLE 0.001f
#define Q_BIAS  0.003f

// ===== Roll Kalman state =====
static float roll_bias = 0.0f;
static float P_roll[2][2] = {
    {1.0f, 0.0f},
    {0.0f, 1.0f}
};

// ===== Pitch Kalman state =====
static float pitch_bias = 0.0f;
static float P_pitch[2][2] = {
    {1.0f, 0.0f},
    {0.0f, 1.0f}
};

void reset_kalman_filter(void)
{
    roll_bias  = 0.0f;
    pitch_bias = 0.0f;

    P_roll[0][0] = P_pitch[0][0] = 1.0f;
    P_roll[0][1] = P_pitch[0][1] = 0.0f;
    P_roll[1][0] = P_pitch[1][0] = 0.0f;
    P_roll[1][1] = P_pitch[1][1] = 1.0f;
}

void kalman_filter(State *state, float dt)
{
    /* ===================== ROLL ===================== */

    // ---- Prediction ----
    state->k_angle[0] += (state->angular_velocity[0] - roll_bias) * dt;

    P_roll[0][0] += dt * (dt * P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_ANGLE);
    P_roll[0][1] -= dt * P_roll[1][1];
    P_roll[1][0] -= dt * P_roll[1][1];
    P_roll[1][1] += Q_BIAS * dt;

    // ---- Update ----
    float y_roll = state->m_angle[0] - state->k_angle[0];
    float S_roll = P_roll[0][0] + STANDARD_DEV_ACCEL_NOISE_SQRD;

    float K_roll_0 = P_roll[0][0] / S_roll;
    float K_roll_1 = P_roll[1][0] / S_roll;

    state->k_angle[0] += K_roll_0 * y_roll;
    roll_bias         += K_roll_1 * y_roll;

    float P00_temp = P_roll[0][0];
    float P01_temp = P_roll[0][1];

    P_roll[0][0] -= K_roll_0 * P00_temp;
    P_roll[0][1] -= K_roll_0 * P01_temp;
    P_roll[1][0] -= K_roll_1 * P00_temp;
    P_roll[1][1] -= K_roll_1 * P01_temp;

    /* ===================== PITCH ===================== */

    // ---- Prediction ----
    state->k_angle[1] += (state->angular_velocity[1] - pitch_bias) * dt;

    P_pitch[0][0] += dt * (dt * P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_ANGLE);
    P_pitch[0][1] -= dt * P_pitch[1][1];
    P_pitch[1][0] -= dt * P_pitch[1][1];
    P_pitch[1][1] += Q_BIAS * dt;

    // ---- Update ----
    float y_pitch = state->m_angle[1] - state->k_angle[1];
    float S_pitch = P_pitch[0][0] + STANDARD_DEV_ACCEL_NOISE_SQRD;

    float K_pitch_0 = P_pitch[0][0] / S_pitch;
    float K_pitch_1 = P_pitch[1][0] / S_pitch;

    state->k_angle[1] += K_pitch_0 * y_pitch;
    pitch_bias        += K_pitch_1 * y_pitch;

    P00_temp = P_pitch[0][0];
    P01_temp = P_pitch[0][1];

    P_pitch[0][0] -= K_pitch_0 * P00_temp;
    P_pitch[0][1] -= K_pitch_0 * P01_temp;
    P_pitch[1][0] -= K_pitch_1 * P00_temp;
    P_pitch[1][1] -= K_pitch_1 * P01_temp;
}
