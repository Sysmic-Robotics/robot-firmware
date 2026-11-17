#include "kalman_2d.h"

void KalmanAngular2D_Init(KalmanAngular2D_t *kf, float Qw, float Qb, float R)
{
    kf->w = 0.0f;
    kf->bias = 0.0f;
    kf->P[0] = 1.0f; kf->P[1] = 0.0f;
    kf->P[2] = 0.0f; kf->P[3] = 1.0f;
    kf->Qw = Qw; kf->Qb = Qb; kf->R = R;
}

void KalmanAngular2D_Predict(KalmanAngular2D_t *kf, float gyro_input, float dt)
{
    float w_pred = kf->w + (gyro_input - kf->bias) * dt;
    float b_pred = kf->bias;

    float P00 = kf->P[0] + dt * (-kf->P[1] - kf->P[2] + dt * kf->P[3]) + kf->Qw;
    float P01 = kf->P[1] - dt * kf->P[3];
    float P10 = kf->P[2] - dt * kf->P[3];
    float P11 = kf->P[3] + kf->Qb;

    kf->w = w_pred;
    kf->bias = b_pred;
    kf->P[0] = P00; kf->P[1] = P01;
    kf->P[2] = P10; kf->P[3] = P11;
}

float KalmanAngular2D_Update(KalmanAngular2D_t *kf, float encoder_omega)
{
    float innov = encoder_omega - kf->w;
    float S = kf->P[0] + kf->R;
    float K0 = kf->P[0] / S;
    float K1 = kf->P[2] / S;

    kf->w += K0 * innov;
    kf->bias += K1 * innov;

    float p00 = kf->P[0] * (1.0f - K0);
    float p01 = kf->P[1] * (1.0f - K0);
    float p10 = kf->P[2] - K1 * kf->P[0];
    float p11 = kf->P[3] - K1 * kf->P[1];

    kf->P[0] = p00; kf->P[1] = p01;
    kf->P[2] = p10; kf->P[3] = p11;

    return kf->w;
}
