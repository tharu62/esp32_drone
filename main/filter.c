#include "filter.h"

#define Q_ANGLE 0.001f
#define Q_BIAS 0.1f
#define R_MEASURE 1.0f

void init_kf(KF* kf)
{
    kf->bias[0] = 0.0f;
    kf->bias[1] = 0.0f;
    
    kf->Qx[0] = Q_ANGLE;
    kf->Qx[1] = Q_BIAS;
    kf->Rx = R_MEASURE;
    kf->Px[0][0] = 1.0f;
    kf->Px[0][1] = 1.0f;
    kf->Px[1][0] = 1.0f;
    kf->Px[1][1] = 1.0f;
    kf->Kx[0] = 0.0f;
    kf->Kx[1] = 0.0f;

    kf->Qy[0] = Q_ANGLE;
    kf->Qy[1] = Q_BIAS;
    kf->Ry = R_MEASURE;
    kf->Py[0][0] = 1.0f;
    kf->Py[0][1] = 1.0f;
    kf->Py[1][0] = 1.0f;
    kf->Py[1][1] = 1.0f;
    kf->Ky[0] = 0.0f;
    kf->Ky[1] = 0.0f;
}

void kalman_filter(State *ds, KF *kf, float dt) 
{    
    // A priori state estimate
    ds->w[0] -= kf->bias[0]; 
    ds->w[1] -= kf->bias[1]; 
    ds->k_angle[0] += ds->w[0] * dt; 
    ds->k_angle[1] += ds->w[1] * dt; 

    // A priori error covariance matrix estimate
    kf->Px[0][0] += dt * (dt*kf->Px[1][1] - kf->Px[0][1] - kf->Px[1][0] + kf->Qx[0]);
    kf->Px[0][1] -= dt * kf->Px[1][1];
    kf->Px[1][0] -= dt * kf->Px[1][1];
    kf->Px[1][1] += dt * kf->Qx[1];

    kf->Py[0][0] += dt * (dt*kf->Py[1][1] - kf->Py[0][1] - kf->Py[1][0] + kf->Qy[0]);
    kf->Py[0][1] -= dt * kf->Py[1][1];
    kf->Py[1][0] -= dt * kf->Py[1][1];
    kf->Py[1][1] += dt * kf->Qy[1];

    // Innovation
    float y[2] = {0.0f};
    y[0] = ds->m_angle[0] - ds->k_angle[0];
    y[1] = ds->m_angle[1] - ds->k_angle[1];

    // Innovation covariance
    float Sx = kf->Px[0][0] + kf->Rx;
    float Sy = kf->Py[0][0] + kf->Ry;
    
    // Kalman Gain
    kf->Kx[0] = kf->Px[0][0] / Sx;
    kf->Kx[1] = kf->Px[1][0] / Sx;

    kf->Ky[0] = kf->Py[0][0] / Sy;
    kf->Ky[1] = kf->Py[1][0] / Sy;

    // A posteriori state estimate
    ds->k_angle[0] += kf->Kx[0] * y[0];
    ds->k_angle[1] += kf->Ky[0] * y[1];

    kf->bias[0] += kf->Kx[1] * y[0];
    kf->bias[1] += kf->Ky[1] * y[1];

    // A posteriori error covariance matrix estimate
    float P00_temp = kf->Px[0][0];
    float P01_temp = kf->Px[0][1];    
    kf->Px[0][0] -= kf->Kx[0] * P00_temp;
    kf->Px[0][1] -= kf->Kx[0] * P01_temp;
    kf->Px[1][0] -= kf->Kx[1] * P00_temp;
    kf->Px[1][1] -= kf->Kx[1] * P01_temp;

    P00_temp = kf->Py[0][0];
    P01_temp = kf->Py[0][1];    
    kf->Py[0][0] -= kf->Ky[0] * P00_temp;
    kf->Py[0][1] -= kf->Ky[0] * P01_temp;
    kf->Py[1][0] -= kf->Ky[1] * P00_temp;
    kf->Py[1][1] -= kf->Ky[1] * P01_temp;
}


#define STANDARD_DEV_ACCEL_NOISE_SQRD 3.0f // Standard deviation squared of a noise (deg) 
#define STANDARD_DEV_GYRO_NOISE_SQRD 3.0f  // Standard deviation squared of gyro noise (deg/s)

float k_uncertainty = {0.0f}; // Initial estimation uncertainty for roll and pitch
float k_gain = {0.0f};        // Kalman gain for roll and pitch angles

// UNUSED: simple complementary filter with Kalman gain logic (not a true Kalman filter implementation, but a simplified version for angle estimation)
inline void complementary_filter(State *ds, float dt) { 
    
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