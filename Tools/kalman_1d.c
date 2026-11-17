#include "kalman_1d.h"

void KalmanVelocity1D_Init(KalmanVelocity1D_t *kf, float Q, float R)
{
    kf->x = 0.0f;
    kf->P = 1.0f;
    kf->Q = Q;
    kf->R = R;
}

float KalmanVelocity1D_Predict(KalmanVelocity1D_t *kf, float control, float dt)
{
    kf->x += control * dt;
    kf->P += kf->Q;
    return kf->x;
}

float KalmanVelocity1D_Update(KalmanVelocity1D_t *kf, float measurement)
{
    float K = kf->P / (kf->P + kf->R);
    kf->x += K * (measurement - kf->x);
    kf->P *= (1.0f - K);
    return kf->x;
}
