#include "filter.h"

#define Q_ANGLE 0.001f
#define Q_BIAS 0.1f
#define R_MEASURE 1.0f

void init_ekf(EKF* ekf)
{
    ekf->bias[0] = 0.0f;
    ekf->bias[1] = 0.0f;
    
    ekf->Qx[0] = Q_ANGLE;
    ekf->Qx[1] = Q_BIAS;
    ekf->Rx = R_MEASURE;
    ekf->Px[0][0] = 1.0f;
    ekf->Px[0][1] = 1.0f;
    ekf->Px[1][0] = 1.0f;
    ekf->Px[1][1] = 1.0f;
    ekf->Kx[0] = 0.0f;
    ekf->Kx[1] = 0.0f;

    ekf->Qy[0] = Q_ANGLE;
    ekf->Qy[1] = Q_BIAS;
    ekf->Ry = R_MEASURE;
    ekf->Py[0][0] = 1.0f;
    ekf->Py[0][1] = 1.0f;
    ekf->Py[1][0] = 1.0f;
    ekf->Py[1][1] = 1.0f;
    ekf->Ky[0] = 0.0f;
    ekf->Ky[1] = 0.0f;
}

void kalman_filter(State *ds, EKF *ekf, float dt) 
{    
    // A priori state estimate
    ds->w[0] -= ekf->bias[0]; 
    ds->w[1] -= ekf->bias[1]; 
    ds->k_angle[0] += ds->w[0] * dt; 
    ds->k_angle[1] += ds->w[1] * dt; 

    // A priori error covariance matrix estimate
    ekf->Px[0][0] += dt * (dt*ekf->Px[1][1] - ekf->Px[0][1] - ekf->Px[1][0] + ekf->Qx[0]);
    ekf->Px[0][1] -= dt * ekf->Px[1][1];
    ekf->Px[1][0] -= dt * ekf->Px[1][1];
    ekf->Px[1][1] += dt * ekf->Qx[1];

    ekf->Py[0][0] += dt * (dt*ekf->Py[1][1] - ekf->Py[0][1] - ekf->Py[1][0] + ekf->Qy[0]);
    ekf->Py[0][1] -= dt * ekf->Py[1][1];
    ekf->Py[1][0] -= dt * ekf->Py[1][1];
    ekf->Py[1][1] += dt * ekf->Qy[1];

    // Innovation
    float y[2] = {0.0f};
    y[0] = ds->m_angle[0] - ds->k_angle[0];
    y[1] = ds->m_angle[1] - ds->k_angle[1];

    // Innovation covariance
    float Sx = ekf->Px[0][0] + ekf->Rx;
    float Sy = ekf->Py[0][0] + ekf->Ry;
    

    // Kalman Gain
    ekf->Kx[0] = ekf->Px[0][0] / Sx;
    ekf->Kx[1] = ekf->Px[1][0] / Sx;

    ekf->Ky[0] = ekf->Py[0][0] / Sy;
    ekf->Ky[1] = ekf->Py[1][0] / Sy;

    // A posteriori state estimate
    ds->k_angle[0] += ekf->Kx[0] * y[0];
    ds->k_angle[1] += ekf->Ky[0] * y[1];

    ekf->bias[0] += ekf->Kx[1] * y[0];
    ekf->bias[1] += ekf->Ky[1] * y[1];

    // A posteriori error covariance matrix estimate
    float P00_temp = ekf->Px[0][0];
    float P01_temp = ekf->Px[0][1];    
    ekf->Px[0][0] -= ekf->Kx[0] * P00_temp;
    ekf->Px[0][1] -= ekf->Kx[0] * P01_temp;
    ekf->Px[1][0] -= ekf->Kx[1] * P00_temp;
    ekf->Px[1][1] -= ekf->Kx[1] * P01_temp;

    P00_temp = ekf->Py[0][0];
    P01_temp = ekf->Py[0][1];    
    ekf->Py[0][0] -= ekf->Ky[0] * P00_temp;
    ekf->Py[0][1] -= ekf->Ky[0] * P01_temp;
    ekf->Py[1][0] -= ekf->Ky[1] * P00_temp;
    ekf->Py[1][1] -= ekf->Ky[1] * P01_temp;
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