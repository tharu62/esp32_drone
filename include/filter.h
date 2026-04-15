#ifndef FILTER_H
#define FILTER_H

#include <math.h>
#include "state.h"

typedef struct {
    float bias[2]; // Gyro fixed bias
    
    // roll
    float Qx[2];    // Process covariance
    float Rx;       // Accelerometer covariance 
    float Px[2][2]; // Error covariance matrix
    float Kx[2];    // Kalman Gain
    
    //pitch
    float Qy[2];    // Process covariance
    float Ry;       // Accelerometer covariance 
    float Py[2][2]; // Error covariance matrix
    float Ky[2];    // Kalman Gain
} EKF;

void init_ekf(EKF *ekf);
 
void kalman_filter(State* ds, EKF* ekf, float dt);

// void complementary_filter(State *ds, float dt);

#endif // KALMAN_FILTER_H