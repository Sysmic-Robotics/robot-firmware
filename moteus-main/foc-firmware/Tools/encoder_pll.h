/**
 * @file encoder_pll.h
 * @brief 2nd-order type-II PLL for encoder angle/velocity filtering
 *
 * Implements a PI + VCO structure (type-II PLL) to produce smooth
 * velocity estimates from incremental encoder readings.  Unlike a
 * simple 1st-order LPF on the differentiated position, the PLL
 * provides zero-lag velocity tracking (zero steady-state error for
 * constant velocity ramps) and natural noise rejection.
 *
 * ## Structure
 * ```
 * error = sin(θ_meas - θ_est)       // phase detector (sin ≈ linear near 0)
 * θ_err_sum += error · dt            // integral accumulator
 * ω_est = Kp · error + Ki · θ_err_sum  // PI output = filtered velocity
 * θ_est += ω_est · dt               // VCO integration
 * ```
 *
 * ## Gain derivation
 * Closed-loop characteristic:  s² + Kp·s + Ki  =  0
 *   ω_n = sqrt(Ki)         natural frequency [rad/s]
 *   ζ   = Kp / (2·ω_n)     damping factor
 *
 * For critical damping (ζ = 1) and a given bandwidth bw_hz:
 *   ω_n = π · bw_hz         (the effective -3 dB bandwidth ≈ 0.32·bw_hz)
 *   Kp  = 2 · ω_n = 2π · bw_hz
 *   Ki  = ω_n² = (π · bw_hz)²
 *
 * Default bw_hz = 50 Hz gives:
 *   Kp = 314.2 [rad/s/rad],  Ki = 24674 [rad/s²/rad]
 *   Settling time ≈ 3/ω_n ≈ 19 ms  (~380 cycles @ 20 kHz)
 *
 * Hardware: STM32G4 with FPU — uses sinf() from <math.h> (~15 cycles).
 */

#ifndef ENCODER_PLL_H_
#define ENCODER_PLL_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Defaults
 * --------------------------------------------------------------------------- */
#define ENCODER_PLL_BW_HZ_DEFAULT   50.0f    /**< PLL bandwidth [Hz]          */
#define ENCODER_PLL_DT_DEFAULT      (1.0f / 20000.0f) /**< 20 kHz default  */

/* ---------------------------------------------------------------------------
 * PLL state
 * --------------------------------------------------------------------------- */
typedef struct {
    /* --- Gains ---------------------------------------------------------- */
    float pll_kp;              /**< Proportional gain [rad/s/rad]            */
    float pll_ki;              /**< Integral gain   [rad/s²/rad]             */

    /* --- State ---------------------------------------------------------- */
    float theta;               /**< Filtered mechanical angle [rad]          */
    float omega;               /**< Filtered angular velocity [rad/s]        */
    float theta_error_sum;     /**< Integral of phase error [rad·s]          */
    float prev_error;          /**< Previous phase error [rad] (reserved)    */

    /* --- Configuration -------------------------------------------------- */
    float dt;                  /**< Sample period [s]                        */

    /* --- Flags ---------------------------------------------------------- */
    bool  initialized;         /**< PLL has been seeded with initial angle   */
} EncoderPLL;

/* ---------------------------------------------------------------------------
 * API
 * --------------------------------------------------------------------------- */

/**
 * @brief Initialise the PLL with bandwidth and sample period.
 *
 * Gains are set for critical damping (ζ = 1):
 *   ω_n = π · bw_hz
 *   Kp  = 2 · ω_n
 *   Ki  = ω_n²
 *
 * The PLL starts uninitialised — the first call to EncoderPLL_Update
 * will seed theta from the measurement and begin tracking.
 *
 * @param pll    PLL instance
 * @param bw_hz  Desired natural bandwidth [Hz]  (50 Hz typical)
 * @param dt     Sample period [s]               (50 µs for 20 kHz)
 */
void EncoderPLL_Init(EncoderPLL *pll, float bw_hz, float dt);

/**
 * @brief Run one PLL update cycle.
 *
 * On the first call after init, theta is latched to meas_theta and
 * tracking begins (avoids a large initial transient).  Subsequent
 * calls run the PI + VCO loop.
 *
 * @param pll         PLL instance
 * @param meas_theta  Measured mechanical angle [rad]  (typically from
 *                    Encoder_Update via the mechanical position)
 */
void EncoderPLL_Update(EncoderPLL *pll, float meas_theta);

/**
 * @brief Reset the PLL to its post-init state (uninitialised).
 */
void EncoderPLL_Reset(EncoderPLL *pll);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_PLL_H_ */
