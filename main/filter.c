#include "filter.h"

#define Q_ANGLE   0.001f
#define Q_BIAS    0.03f
#define R_MEASURE 1.0f


void init_kf(KF* kf)
{
    kf->bias[0] = 0.0f;
    kf->bias[1] = 0.0f;

    kf->Qx[0] = Q_ANGLE;
    kf->Qx[1] = Q_BIAS;

    kf->Qy[0] = Q_ANGLE;
    kf->Qy[1] = Q_BIAS;

    kf->Px[0][0] = 1.0f;
    kf->Px[1][1] = 1.0f;

    kf->Py[0][0] = 1.0f;
    kf->Py[1][1] = 1.0f;
}

void kalman_filter(State *ds, KF *kf, float dt)
{
    // 1. Prediction (gyro integration)
    float wx = ds->w[0] - kf->bias[0];
    float wy = ds->w[1] - kf->bias[1];

    ds->k_angle[0] += wx * dt;
    ds->k_angle[1] += wy * dt;

    // 2. Covariance prediction (DIAGONAL approx)
    kf->Px[0][0] += dt * (kf->Px[1][1] * dt + kf->Qx[0]);
    kf->Px[1][1] += kf->Qx[1] * dt;

    kf->Py[0][0] += dt * (kf->Py[1][1] * dt + kf->Qy[0]);
    kf->Py[1][1] += kf->Qy[1] * dt;

    // 3. Innovation
    float ex = ds->m_angle[0] - ds->k_angle[0];
    float ey = ds->m_angle[1] - ds->k_angle[1];

    // 4. Kalman gain (optimized division)
    float Sx = kf->Px[0][0] + R_MEASURE;
    float Sy = kf->Py[0][0] + R_MEASURE;

    float Kx = kf->Px[0][0] * (1.0f / Sx);
    float Ky = kf->Py[0][0] * (1.0f / Sy);

    float Kb_x = kf->Px[1][1] * (1.0f / Sx);
    float Kb_y = kf->Py[1][1] * (1.0f / Sy);

    // 5. Correction
    ds->k_angle[0] += Kx * ex;
    ds->k_angle[1] += Ky * ey;

    kf->bias[0] += Kb_x * ex;
    kf->bias[1] += Kb_y * ey;

    // 6. Covariance update
    kf->Px[0][0] *= (1.0f - Kx);
    kf->Px[1][1] *= (1.0f - Kb_x);

    kf->Py[0][0] *= (1.0f - Ky);
    kf->Py[1][1] *= (1.0f - Kb_y);
}


#define STANDARD_DEV_ACCEL_NOISE_SQRD 8.0f // Standard deviation squared of a noise (deg) 
#define STANDARD_DEV_GYRO_NOISE_SQRD 16.0f  // Standard deviation squared of gyro noise (deg/s)

float k_uncertainty = {0.0f}; // Initial estimation uncertainty for roll and pitch
float k_gain = {0.0f};        // Kalman gain for roll and pitch angles

void complementary_filter(State *ds, float dt) { 
    
    // Prediction step: integrate gyro angles to get angle estimates
    ds->k_angle[0] += ds->w[0] * dt;
    ds->k_angle[1] += ds->w[1] * dt;

    // Calculate Kalman gain
    k_uncertainty += dt * dt * STANDARD_DEV_GYRO_NOISE_SQRD;
    k_gain = k_uncertainty / (k_uncertainty + STANDARD_DEV_ACCEL_NOISE_SQRD);

    // Update angle by combining angle from accelerometer measurement and integrated gyro angle 
    // with Kalman gain on complementary filter logic.  
    ds->k_angle[0] = k_gain * ds->m_angle[0] + (1 - k_gain) * ds->k_angle[0];
    ds->k_angle[1] = k_gain * ds->m_angle[1] + (1 - k_gain) * ds->k_angle[1];

    // Update uncertainty
    k_uncertainty *= (1 - k_gain);
}