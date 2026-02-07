#ifndef STATE_H
#define STATE_H

typedef struct {
    float d_angle[2];           // Desired angles: roll, pitch, yaw
    float m_angle[2];           // Measured angles from accelerometer: roll, pitch, yaw 
    float k_angle[2];           // Kalman filtered angles: roll, pitch, yaw
    float angular_velocity[3];  // Angular velocity from gyroscope: roll rate, pitch rate, yaw rate
} State;

void init_state(State *state);

void update_state(State *state, float angle_roll, float angle_pitch, float gyro_roll, float gyro_pitch);

#endif // STATE_H