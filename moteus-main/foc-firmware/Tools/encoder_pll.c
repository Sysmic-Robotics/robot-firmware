/**
 * @file encoder_pll.c
 * @brief 2nd-order type-II PLL for encoder filtering — Implementation
 *
 * @see encoder_pll.h for the full PLL structure and gain derivation.
 */

#include "encoder_pll.h"
#include <math.h>

/* ---------------------------------------------------------------------------
 * Internal: two-pi constant (avoid dependency on foc.h / encoder.c)
 * --------------------------------------------------------------------------- */
#define PLL_2PI  (6.283185307179586f)

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

void EncoderPLL_Init(EncoderPLL *pll, float bw_hz, float dt) {
    /* Natural frequency from bandwidth */
    const float omega_n = 3.141592653589793f * bw_hz;   /* ω_n = π · bw */

    /* Gains for critical damping (ζ = 1) */
    pll->pll_kp = 2.0f * omega_n;                       /* Kp = 2·ω_n   [rad/s/rad] */
    pll->pll_ki = omega_n * omega_n;                     /* Ki = ω_n²   [rad/s²/rad] */

    /* State — zeroed, will be seeded on first update */
    pll->theta           = 0.0f;
    pll->omega           = 0.0f;
    pll->theta_error_sum = 0.0f;
    pll->prev_error      = 0.0f;

    /* Configuration */
    pll->dt = (dt > 0.0f) ? dt : ENCODER_PLL_DT_DEFAULT;

    /* Flag: first EncoderPLL_Update will latch theta */
    pll->initialized = false;
}

void EncoderPLL_Update(EncoderPLL *pll, float meas_theta) {
    /* ---- First call: latch angle and begin tracking -------------------- */
    if (!pll->initialized) {
        pll->theta           = meas_theta;
        pll->omega           = 0.0f;
        pll->theta_error_sum = 0.0f;
        pll->prev_error      = 0.0f;
        pll->initialized     = true;
        return;
    }

    const float dt = pll->dt;

    /* ---- 1. Phase detector --------------------------------------------- */
    /* sin(θ_meas - θ_est) handles wrap-around smoothly; for small errors
     * sin(x) ≈ x, so this is a linear phase detector near lock.         */
    float error = sinf(meas_theta - pll->theta);

    /* ---- 2. PI loop filter --------------------------------------------- */
    /* Integrate the phase error */
    pll->theta_error_sum += error * dt;

    /* PI output = filtered velocity [rad/s] */
    pll->omega = pll->pll_kp * error + pll->pll_ki * pll->theta_error_sum;

    /* ---- 3. VCO (numerically controlled oscillator) -------------------- */
    pll->theta += pll->omega * dt;

    /* Wrap theta to [-π, π] to keep sinf() argument bounded and avoid
     * precision loss as theta grows unbounded. */
    if (pll->theta > PLL_2PI) {
        pll->theta -= PLL_2PI;
    } else if (pll->theta < -PLL_2PI) {
        pll->theta += PLL_2PI;
    }

    /* Store for derivative term (reserved for future Kd extension) */
    pll->prev_error = error;
}

void EncoderPLL_Reset(EncoderPLL *pll) {
    pll->theta           = 0.0f;
    pll->omega           = 0.0f;
    pll->theta_error_sum = 0.0f;
    pll->prev_error      = 0.0f;
    pll->initialized     = false;
}
