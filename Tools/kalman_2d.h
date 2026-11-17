#ifndef KALMAN_2D_H
#define KALMAN_2D_H

typedef struct {
    float w;
    float bias;
    float P[4];
    float Qw;
    float Qb;
    float R;
} KalmanAngular2D_t;

void KalmanAngular2D_Init(KalmanAngular2D_t *kf, float Qw, float Qb, float R);
void KalmanAngular2D_Predict(KalmanAngular2D_t *kf, float gyro_input, float dt);
float KalmanAngular2D_Update(KalmanAngular2D_t *kf, float encoder_omega);

#endif
