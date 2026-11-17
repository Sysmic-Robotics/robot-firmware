#ifndef KALMAN_1D_H
#define KALMAN_1D_H

typedef struct {
    float x;
    float P;
    float Q;
    float R;
} KalmanVelocity1D_t;

void KalmanVelocity1D_Init(KalmanVelocity1D_t *kf, float Q, float R);
float KalmanVelocity1D_Predict(KalmanVelocity1D_t *kf, float control, float dt);
float KalmanVelocity1D_Update(KalmanVelocity1D_t *kf, float measurement);

#endif
