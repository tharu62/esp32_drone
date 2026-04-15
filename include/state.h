#ifndef STATE_H
#define STATE_H

typedef struct {
    float throttle;
    float d_angle[3];           // Desired angles: roll, pitch, yaw
    float m_angle[3];           // Measured angles from accelerometer: roll, pitch, yaw 
    float k_angle[3];           // Kalman filtered angles: roll, pitch, yaw
    float w[3];                 // Angular velocity from gyroscope: roll rate, pitch rate, yaw rate
    float a[3];                 // Acceleration from accelerometer: ax, ay, az
    float pid_output[3];        // PID output for angle control: roll, pitch, yaw
} State;

void init_state(State *state);

void update_state(State *state, float angle_roll, float angle_pitch, float gyro_roll, float gyro_pitch);

#endif // STATE_H