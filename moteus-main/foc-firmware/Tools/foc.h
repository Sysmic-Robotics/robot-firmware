/**
 * @file foc.h
 * @brief Field-Oriented Control for Phoenix FOC Firmware (STM32G431)
 *
 * Implements the complete FOC current loop:
 *   - Clarke transform (abc → αβ)
 *   - Park transform (αβ → dq)
 *   - PI current controllers (d and q axis)
 *   - Inverse Park (dq → αβ)
 *   - Space Vector Modulation via balanced voltage (min-max injection)
 *   - PWM duty write to TIM2 CCR
 *
 * References:
 *   - moteus bldc_servo_control.h for balanced-voltage SVM, PI, transforms
 *   - "Field Oriented Control of Permanent Magnet Synchronous Motors"
 *
 * Hardware:
 *   - TIM2: center-aligned PWM @ 20 kHz on CH1/2/3 (PA0/PA1/PA2)
 *   - ADC1 CH4  (PA3) = Phase A current
 *   - ADC1 CH12 (PB1) = Phase C current
 *   - ADC2 CH3  (PA6) = Phase B current
 *   - Encoder (TIM1) provides electrical angle
 *
 * VBUS (bus voltage) is passed as a parameter to FOC_Run / FOC_RunWithRefs.
 * It must be provided by the caller (e.g. from an ADC reading or a
 * fixed nominal value until a proper measurement is implemented).
 */

#ifndef FOC_H_
#define FOC_H_

#include "main.h"
#include "pid.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Default configuration constants
 * --------------------------------------------------------------------------- */
#define FOC_PWM_FREQ_DEFAULT       20000.0f     /**< PWM & control rate [Hz]  */
#define FOC_DT_DEFAULT             (1.0f / 20000.0f)

/* Current sense: 0.5 mOhm shunt, DRV8323 CSA gain 20 V/V */
#define FOC_DEFAULT_RSHUNT         0.0005f      /**< shunt resistor [Ohm]     */
#define FOC_DEFAULT_CSA_GAIN       20.0f        /**< DRV8323 CSA gain [V/V]   */

/* PI current controller defaults — tune per motor */
#define FOC_PI_D_KP_DEFAULT        0.08f
#define FOC_PI_D_KI_DEFAULT        4.0f
#define FOC_PI_Q_KP_DEFAULT        0.08f
#define FOC_PI_Q_KI_DEFAULT        4.0f
#define FOC_PI_LIMIT_DEFAULT       24.0f        /**< output voltage limit [V] */
#define FOC_PI_ILIMIT_DEFAULT      12.0f        /**< integrator windup [V]    */
#define FOC_MAX_CURRENT_DEFAULT    30.0f        /**< phase current limit [A]  */

/* ---------------------------------------------------------------------------
 * PI controller
 * --------------------------------------------------------------------------- */
typedef struct {
    float kp;         /**< proportional gain                       */
    float ki;         /**< integral gain                           */
    float integral;   /**< integrator state                        */
    float limit;      /**< output clamp symmetric ±limit           */
    float ilimit;     /**< integrator windup clamp ±ilimit         */
    float output;     /**< last output value                       */
    float error;      /**< last error (target - measured)          */
} FOC_PI;

/* ---------------------------------------------------------------------------
 * FOC run-time state  (all SI units except duty cycles)
 * --------------------------------------------------------------------------- */
typedef struct {
    /* --- configuration (set at init, read-only during control) ------------ */
    uint32_t pole_pairs;            /**< motor pole pairs                     */
    float    pwm_freq;              /**< PWM & control loop frequency [Hz]    */
    float    dt;                    /**< 1/pwm_freq [s]                       */
    float    rshunt;                /**< shunt resistor [Ω]                   */
    float    csa_gain;              /**< DRV8323 CSA gain [V/V]               */
    float    adc_to_amps;           /**< precomputed: ADC counts → Amperes    */
    float    max_current;           /**< hard phase current limit [A]         */

    /* --- PI gains (copied to controller on init) ------------------------- */
    float    pi_d_kp, pi_d_ki;
    float    pi_q_kp, pi_q_ki;
    float    pi_current_limit;      /**< PI output voltage clamp [V]          */
    float    pi_i_limit;            /**< PI integrator clamp [V]              */

    /* --- phase currents [A] ---------------------------------------------- */
    float ia, ib, ic;

    /* --- stationary frame (Clarke) --------------------------------------- */
    float ialpha, ibeta;

    /* --- rotating frame (Park) — measured & reference [A] ---------------- */
    float id_meas, iq_meas;
    float id_ref,   iq_ref;

    /* --- voltage commands [V] -------------------------------------------- */
    float vd, vq;

    /* --- normalised duty cycles [0..1] for TIM2 CCR ---------------------- */
    float duty_a, duty_b, duty_c;

    /* --- PI controller state --------------------------------------------- */
    FOC_PI pid;      /**< d-axis current controller                         */
    FOC_PI piq;      /**< q-axis current controller                         */

    /* --- motor parameters for advanced control --------------------------- */
    float   motor_Ld;              /**< d-axis inductance [H]               */
    float   motor_Lq;              /**< q-axis inductance [H]               */
    float   bemf_v_per_hz;         /**< back-EMF constant [V/(rad/s_elec)]  */
    float   prev_theta_elec;       /**< previous theta for ωe computation   */

    /* --- direct voltage mode (bypass PI, used by Ld/Lq calibration) ----- */
    bool    direct_voltage_mode; /**< when true, vd_direct/vq_direct used  */
    float   vd_direct;           /**< direct d-axis voltage command [V]    */
    float   vq_direct;           /**< direct q-axis voltage command [V]    */

    /* --- control flags --------------------------------------------------- */
    bool    enabled;       /**< global enable (FOC_Enable/FOC_Disable)      */
    uint8_t fault_flags;   /**< fault bitmask (reserved)                    */
} FOC_State;

/* ---------------------------------------------------------------------------
 * Constants (shared with .c)
 * --------------------------------------------------------------------------- */
#define FOC_2PI          (6.283185307179586f)
#define FOC_1_OVER_3     (0.3333333333333333f)
#define FOC_1_OVER_SQRT3 (0.5773502691896257f)
#define FOC_SQRT3_4      (0.8660254037844386f)
#define FOC_SQRT3        (1.7320508075688772f)

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

/** @brief Initialise FOC with default parameters.
 *         Must be called once after MX_TIM2_Init and MX_ADC*_Init.
 */
void FOC_Init(void);

/** @brief Set current references for the current control loop.
 *  @param id_ref  Direct-axis current reference [A]  (usually 0 for non-FW)
 *  @param iq_ref  Quadrature-axis current reference [A] (proportional to torque)
 */
void FOC_SetCurrent(float id_ref, float iq_ref);

/** @brief Run one iteration of the FOC current loop.
 *
 *  Expected call rate: g_foc.pwm_freq (20 kHz by default).
 *  Reads ADCs, runs transforms, PI control, SVM, writes PWM.
 *  Uses g_foc.id_ref / g_foc.iq_ref as current targets.
 *
 *  @param theta_elec  Electrical angle [rad] (from encoder)
 *  @param vbus        Instantaneous bus voltage [V]
 */
void FOC_Run(float theta_elec, float vbus);

/** @brief FOC current loop with explicit current references.
 *
 *  Same as FOC_Run but directly accepts id_ref/iq_ref instead of
 *  reading them from g_foc.  Designed for use with ServoControl,
 *  which updates the references every control cycle.
 *
 *  @param theta_elec  Electrical angle [rad]
 *  @param vbus        Bus voltage [V]
 *  @param id_ref      D-axis current reference [A]
 *  @param iq_ref      Q-axis current reference [A]
 */
void FOC_RunWithRefs(float theta_elec, float vbus,
                     float id_ref, float iq_ref);

/** @brief Enable PWM output and FOC control. */
void FOC_Enable(void);

/** @brief Disable PWM output (all phases low-side) and FOC. */
void FOC_Disable(void);

/** @brief Set motor parameters for BEMF feedforward and cross-coupling.
 *  @param Ld            D-axis inductance [H]
 *  @param Lq            Q-axis inductance [H]
 *  @param bemf_v_per_hz Back-EMF constant [V/(rad/s_elec)]
 */
void FOC_SetMotorParams(float Ld, float Lq, float bemf_v_per_hz);

/** @brief Set direct voltage command, bypassing PI current controllers.
 *
 *  Single-shot: applies vd/vq for exactly one FOC_Run/FOC_RunWithRefs call,
 *  then reverts to PI control.  Must be called every cycle during the
 *  measurement window to maintain open-loop voltage.
 *
 *  Used by Ld/Lq calibration to inject a controlled Vd step.
 *
 *  @param vd  Direct-axis voltage [V]
 *  @param vq  Quadrature-axis voltage [V]
 */
void FOC_SetDirectVd(float vd, float vq);

/** @brief Poke a new dt for the FOC loop (e.g. if PWM frequency changed). */
void FOC_SetDt(float dt);

/* ---------------------------------------------------------------------------
 * Global state for telemetry / debug
 * --------------------------------------------------------------------------- */
extern FOC_State g_foc;

#ifdef __cplusplus
}
#endif

#endif /* FOC_H_ */
