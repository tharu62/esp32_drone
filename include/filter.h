#ifndef FILTER_H
#define FILTER_H

#include <math.h>
#include "state.h"

// KALMAN FILTER DATA STRUCTURE
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
} KF;

/**
 * @brief Initializes the Kalman Filter struct with correct covariances and uncertainties.
 * @param kf kalman filter struct
 */
void init_kf(KF *kf);
 
/**
 * @brief Implements the Kalman Filter algorithm using the corrent state of the drone
 * and the Kalman Filter struct status to predict and correct the current status of the 
 * drone (roll and pitch angle)
 * @param ds drone state
 * @param kf Kalman Filter struct
 * @param dt time step
 */
void kalman_filter(State* ds, KF* kf, float dt);

/**
 * @brief Implements a complementary filter that uses a Kalman FIlter style statistic 
 * method to reduce noise on measurement of the current status of the drone (roll and pith angle)
 * @param ds drone state
 * @param dt time step
 */
void complementary_filter(State *ds, float dt);

#endif // KALMAN_FILTER_H