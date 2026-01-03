#ifndef STATE_H
#define STATE_H

typedef struct {
    float position[3];          // x, y, z
    float acceleration[3];      // ax, ay, az
    float d_angle[3];           // Desired angles: roll, pitch, yaw
    float m_angle[3];           // Measured angles from accelerometer: roll, pitch, yaw 
    float k_angle[3];           // Kalman filtered angles: roll, pitch, yaw
    float angular_velocity[3];  // wx, wy, wz
    float angular_acceleration[3]; // alpha_x, alpha_y, alpha_z
} State;

void init_state(State *state);

void update_state(State *state, float angle_roll, float angle_pitch, float gyroX, float gyroY);

#endif // STATE_H