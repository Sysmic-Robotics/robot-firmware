#ifndef EKF_H
#define EKF_H

#include <stdint.h>

#define EKF_N 7        // State dimension
#define EKF_NP 28      // Upper triangle of 7x7: 7*(7+1)/2
#define EKF_HIST_SIZE 48  // State history for delayed measurements (~48ms @1kHz)

typedef struct {
    /* State vector: [x, y, sinθ, cosθ, vx_robot, vy_robot, ω] */
    float x[EKF_N];

    /* Covariance matrix P (upper triangle, row-major) */
    float P[EKF_NP];

    /* Process noise diagonal (7 elements) */
    float Q[EKF_N];

    /* Measurement noise — encoder (3 elements: vx, vy, ω) */
    float R_enc[3];

    /* Measurement noise — engine (5 elements: x, y, vx_world, vy_world, θ) */
    float R_eng[5];

    /* Time step */
    float dt;

    /* State history for delayed measurements (circular buffer) */
    float hist[EKF_HIST_SIZE][EKF_N];
    float hist_P[EKF_HIST_SIZE][EKF_NP]; /* covariance at each history point */
    float hist_time[EKF_HIST_SIZE];
    uint8_t hist_idx;
    uint8_t hist_count;

    /* Current time accumulator (seconds) */
    float time;
} EKF_t;

void EKF_Init(EKF_t *ekf, float dt);
void EKF_Predict(EKF_t *ekf, float ax_body, float ay_body, float gyro_z);
void EKF_UpdateEncoders(EKF_t *ekf, float vx_enc, float vy_enc, float vth_enc);
void EKF_UpdateEngine(EKF_t *ekf, float x_e, float y_e, float vx_e, float vy_e, float theta_e);
void EKF_SaveHistory(EKF_t *ekf);
/* Delayed measurement update: apply a camera correction that was valid at
 * time t_cam (seconds, in the EKF time base). Restores the state+covariance
 * from the history buffer at t_cam, applies the Kalman correction, then
 * re-propagates the corrected state forward to the present using the latest
 * IMU inputs (ax_body, ay_body, gyro_z). */
void EKF_UpdateEngineDelayed(EKF_t *ekf, float x_cam, float y_cam, float theta_cam,
                             float t_cam, float ax_body, float ay_body, float gyro_z);

#endif
