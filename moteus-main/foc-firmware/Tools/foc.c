/**
 * @file foc.c
 * @brief Field-Oriented Control — Implementation
 *
 * FOC current loop sequence (20 kHz target):
 *   1. Sample phase currents (ADC1 CH4=Ia, CH12=Ic; ADC2 CH3=Ib)
 *   2. Clarke transform  (abc → αβ)
 *   3. Park transform    (αβ → dq) using sin(θ), cos(θ)
 *   4. PI current control (d and q axes)
 *   5. Inverse Park      (dq → αβ)
 *   6. SVM — balanced-voltage min-max injection  (αβ → duty_a,b,c)
 *   7. Write TIM2 CCRs
 *
 * Hardware: STM32G431CBU6 @ 170 MHz
 *   TIM2:  center-aligned PWM, CH1/2/3 (PA0/PA1/PA2)
 *   ADC1:  CH4=PA3 (Ia), CH12=PB1 (Ic), scan mode
 *   ADC2:  CH3=PA6 (Ib)
 */

#include "foc.h"
#include "pid.h"
#include "tim.h"
#include "adc.h"
#include "trig_lut.h"

/* ====== PI controller helpers =========================================== */

/** @brief Update one PI controller.
 *  @return Output value (clamped).
 */
static inline float pi_update(FOC_PI *pi, float error, float dt) {
    pi->error = error;
    pi->integral += error * pi->ki * dt;

    /* Anti-windup: clamp integrator */
    if (pi->integral >  pi->ilimit) pi->integral =  pi->ilimit;
    if (pi->integral < -pi->ilimit) pi->integral = -pi->ilimit;

    pi->output = error * pi->kp + pi->integral;

    /* Clamp total output */
    if (pi->output >  pi->limit) pi->output =  pi->limit;
    if (pi->output < -pi->limit) pi->output = -pi->limit;

    return pi->output;
}

/** @brief Reset PI controller state to zero. */
static inline void pi_reset(FOC_PI *pi) {
    pi->integral = 0.0f;
    pi->output   = 0.0f;
    pi->error    = 0.0f;
}

/* ====== Current sampling ================================================ */

/** @brief Read all three phase currents from the ADCs.
 *
 *  ADC1 has scan mode with 2 channels:
 *    rank 1 = CH4  (PA3) → Phase A
 *    rank 2 = CH12 (PB1) → Phase C
 *
 *  ADC2 single channel:
 *    rank 1 = CH3  (PA6) → Phase B
 *
 *  Converts raw 12-bit counts to Amperes using the precomputed factor.
 */
static void read_phase_currents(float *ia, float *ib, float *ic) {
    uint32_t raw_a, raw_b, raw_c;
    const float zero = 2048.0f;       /* VREF/2 = 1.65V @ 12-bit */

    /* ADC1: Ia on CH4 (rank 1), Ic on CH12 (rank 2) */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    raw_a = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    raw_c = HAL_ADC_GetValue(&hadc1);

    /* ADC2: Ib on CH3 */
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    raw_b = HAL_ADC_GetValue(&hadc2);

    /* Convert to Amperes */
    *ia = ((float)raw_a - zero) * g_foc.adc_to_amps;
    *ib = ((float)raw_b - zero) * g_foc.adc_to_amps;
    *ic = ((float)raw_c - zero) * g_foc.adc_to_amps;
}

/* ====== Clarke transform   abc → αβ ===================================== */

static inline void clarke_transform(float ia, float ib, float ic,
                                    float *ialpha, float *ibeta) {
    *ialpha = (2.0f * ia - ib - ic) * FOC_1_OVER_3;  /* = Ia */
    *ibeta  = (ib - ic) * FOC_1_OVER_SQRT3;           /* = (Ib-Ic)/√3 */
}

/* ====== Park transform     αβ → dq ====================================== */

static inline void park_transform(float ialpha, float ibeta,
                                  float sin_th, float cos_th,
                                  float *id, float *iq) {
    *id =  ialpha * cos_th + ibeta * sin_th;
    *iq = -ialpha * sin_th + ibeta * cos_th;
}

/* ====== Inverse Park       dq → αβ ====================================== */

static inline void inverse_park(float vd, float vq,
                                float sin_th, float cos_th,
                                float *valpha, float *vbeta) {
    *valpha = vd * cos_th - vq * sin_th;
    *vbeta  = vd * sin_th + vq * cos_th;
}

/* ====== Space Vector Modulation  (αβ → duty cycles) ===================== */

/** @brief Balanced-voltage SVM (min-max zero-sequence injection).
 *
 *  The method is equivalent to space-vector PWM but simpler to compute:
 *    1. Inverse Clarke from αβ to abc  (→ normalised phase voltages)
 *    2. Normalise by Vbus
 *    3. Inject zero-sequence: offset = 0.5·(max+min) − 0.5
 *    4. Subtract offset → duty cycles in [0, 1]
 *
 *  Reference: moteus bldc_servo_control.h :: ISR_DoBalancedVoltageControl
 */
static void svm(float valpha, float vbeta, float vbus,
                float *duty_a, float *duty_b, float *duty_c) {
    /* Inverse Clarke → three-phase voltages (amplitude in volts) */
    float va = valpha;
    float vb = (-0.5f * valpha + FOC_SQRT3_4 * vbeta);
    float vc = (-0.5f * valpha - FOC_SQRT3_4 * vbeta);

    /* Normalise to per-unit (Vbus → 1.0) */
    const float inv_vbus = (vbus > 0.1f) ? (1.0f / vbus) : 1.0f;
    va *= inv_vbus;
    vb *= inv_vbus;
    vc *= inv_vbus;

    /* Min-max injection (balanced midpoint) */
    float vmin = va;
    vmin = (vb < vmin) ? vb : vmin;
    vmin = (vc < vmin) ? vc : vmin;
    float vmax = va;
    vmax = (vb > vmax) ? vb : vmax;
    vmax = (vc > vmax) ? vc : vmax;

    const float offset = 0.5f * (vmin + vmax) - 0.5f;

    *duty_a = va - offset;   /* in [0, 1] */
    *duty_b = vb - offset;
    *duty_c = vc - offset;
}

/* ====== PWM write ======================================================= */

/** @brief Write normalised duty cycles [0, 1] to TIM2 CCR registers.
 *
 *  TIM2 is in center-aligned mode:
 *    duty = 0   → CCR = 0    → 0%   (always low)
 *    duty = 0.5 → CCR = ARR/2 → 50%
 *    duty = 1   → CCR = ARR  → ~100% (effectively full)
 */
static void write_pwm_duties(float da, float db, float dc) {
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);

    uint32_t ccr1 = (uint32_t)(da * (float)arr);
    uint32_t ccr2 = (uint32_t)(db * (float)arr);
    uint32_t ccr3 = (uint32_t)(dc * (float)arr);

    /* Clamp to valid range */
    if (ccr1 > arr) ccr1 = arr;
    if (ccr2 > arr) ccr2 = arr;
    if (ccr3 > arr) ccr3 = arr;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr3);
}

/* ====== Public API ====================================================== */

/* Global state */
FOC_State g_foc;

void FOC_Init(void) {
    FOC_State *f = &g_foc;

    /* Default configuration */
    f->pole_pairs       = 7;
    f->pwm_freq         = FOC_PWM_FREQ_DEFAULT;
    f->dt               = FOC_DT_DEFAULT;
    f->rshunt           = FOC_DEFAULT_RSHUNT;
    f->csa_gain         = FOC_DEFAULT_CSA_GAIN;

    /* Precompute ADC → Amperes conversion factor.
     *   Vref = 3.3 V,  12-bit ADC → 4096 codes
     *   V_adc = (code - 2048) * 3.3 / 4096
     *   I = V_adc / (csa_gain * rshunt)
     *   I = (code - 2048) * 3.3 / (4096 * csa_gain * rshunt)
     */
    const float denominator = 4096.0f * f->csa_gain * f->rshunt;
    f->adc_to_amps = (denominator > 1e-12f) ? (3.3f / denominator) : 0.0f;

    f->max_current      = FOC_MAX_CURRENT_DEFAULT;
    f->pi_d_kp          = FOC_PI_D_KP_DEFAULT;
    f->pi_d_ki          = FOC_PI_D_KI_DEFAULT;
    f->pi_q_kp          = FOC_PI_Q_KP_DEFAULT;
    f->pi_q_ki          = FOC_PI_Q_KI_DEFAULT;
    f->pi_current_limit = FOC_PI_LIMIT_DEFAULT;
    f->pi_i_limit       = FOC_PI_ILIMIT_DEFAULT;

    /* Initialise PI controllers */
    f->pid.kp     = f->pi_d_kp;
    f->pid.ki     = f->pi_d_ki;
    f->pid.limit  = f->pi_current_limit;
    f->pid.ilimit = f->pi_i_limit;
    pi_reset(&f->pid);

    f->piq.kp     = f->pi_q_kp;
    f->piq.ki     = f->pi_q_ki;
    f->piq.limit  = f->pi_current_limit;
    f->piq.ilimit = f->pi_i_limit;
    pi_reset(&f->piq);

    /* Motor parameters (defaults — set explicitly via FOC_SetMotorParams) */
    f->motor_Ld         = 1e-5f;
    f->motor_Lq         = 1e-5f;
    f->bemf_v_per_hz    = 0.0f;
    f->prev_theta_elec  = 0.0f;

    /* Clear state */
    f->ia = f->ib = f->ic = 0.0f;
    f->ialpha = f->ibeta = 0.0f;
    f->id_meas = f->iq_meas = 0.0f;
    f->id_ref  = f->iq_ref  = 0.0f;
    f->vd = f->vq = 0.0f;
    f->duty_a = f->duty_b = f->duty_c = 0.0f;

    f->direct_voltage_mode = false;
    f->vd_direct = 0.0f;
    f->vq_direct = 0.0f;

    f->enabled     = false;
    f->fault_flags = 0;

    /* Start TIM2 PWM channels at 0% duty */
    write_pwm_duties(0.0f, 0.0f, 0.0f);
}

void FOC_SetCurrent(float id_ref, float iq_ref) {
    g_foc.id_ref = id_ref;
    g_foc.iq_ref = iq_ref;
}

void FOC_SetMotorParams(float Ld, float Lq, float bemf_v_per_hz) {
    g_foc.motor_Ld      = Ld;
    g_foc.motor_Lq      = Lq;
    g_foc.bemf_v_per_hz = bemf_v_per_hz;
}

void FOC_Run(float theta_elec, float vbus) {
    FOC_State *f = &g_foc;
    if (!f->enabled) return;

    const float dt = f->dt;

    /* 1. Sample phase currents */
    read_phase_currents(&f->ia, &f->ib, &f->ic);

    /* 2. Clarke: abc → αβ */
    clarke_transform(f->ia, f->ib, f->ic, &f->ialpha, &f->ibeta);

    /* 3. Park: αβ → dq */
    uint16_t th_idx = (uint16_t)(theta_elec * (1024.0f / FOC_2PI)) & 1023;
    const float sin_th = lut_sin(th_idx);
    const float cos_th = lut_cos(th_idx);
    park_transform(f->ialpha, f->ibeta, sin_th, cos_th,
                   &f->id_meas, &f->iq_meas);

    /* 4. PI current control OR direct voltage mode */
    const bool in_direct_mode = f->direct_voltage_mode;
    if (in_direct_mode) {
        /* Bypass PI — use commanded voltage directly (single-shot) */
        f->vd = f->vd_direct;
        f->vq = f->vq_direct;
        f->direct_voltage_mode = false;
    } else {
        f->vd = pi_update(&f->pid, f->id_ref - f->id_meas, dt);
        f->vq = pi_update(&f->piq, f->iq_ref - f->iq_meas, dt);
    }

    /* 4b. BEMF feedforward + cross-coupling decoupling */
    if (!in_direct_mode) {
        float omega_e = (theta_elec - f->prev_theta_elec) * f->pwm_freq;
        if (omega_e > 1e-6f || omega_e < -1e-6f) {
            f->vd -= omega_e * f->motor_Lq * f->iq_meas;
            f->vq += omega_e * f->motor_Ld * f->id_meas;
            f->vq += omega_e * f->bemf_v_per_hz;
        }
        f->prev_theta_elec = theta_elec;
    }

    /* 4c. VBUS for SVM reference */
    const float vbus_svm = (vbus > 0.1f) ? vbus : 1.0f;

    /* 5. Inverse Park: dq → αβ */
    float valpha, vbeta;
    inverse_park(f->vd, f->vq, sin_th, cos_th, &valpha, &vbeta);

    /* 6. SVM: αβ → duty cycles */
    svm(valpha, vbeta, vbus_svm, &f->duty_a, &f->duty_b, &f->duty_c);

    /* 7. Write PWM duties */
    write_pwm_duties(f->duty_a, f->duty_b, f->duty_c);
}

void FOC_RunWithRefs(float theta_elec, float vbus,
                     float id_ref, float iq_ref) {
    FOC_State *f = &g_foc;
    if (!f->enabled) return;

    const float dt = f->dt;

    /* 1. Sample phase currents */
    read_phase_currents(&f->ia, &f->ib, &f->ic);

    /* 2. Clarke: abc → αβ */
    clarke_transform(f->ia, f->ib, f->ic, &f->ialpha, &f->ibeta);

    /* 3. Park: αβ → dq */
    uint16_t th_idx = (uint16_t)(theta_elec * (1024.0f / FOC_2PI)) & 1023;
    const float sin_th = lut_sin(th_idx);
    const float cos_th = lut_cos(th_idx);
    park_transform(f->ialpha, f->ibeta, sin_th, cos_th,
                   &f->id_meas, &f->iq_meas);

    /* 4. Use provided references */
    f->id_ref = id_ref;
    f->iq_ref = iq_ref;

    /* 5. PI current control OR direct voltage mode */
    const bool in_direct_mode = f->direct_voltage_mode;
    if (in_direct_mode) {
        f->vd = f->vd_direct;
        f->vq = f->vq_direct;
        f->direct_voltage_mode = false;
    } else {
        f->vd = pi_update(&f->pid, f->id_ref - f->id_meas, dt);
        f->vq = pi_update(&f->piq, f->iq_ref - f->iq_meas, dt);
    }

    /* 5b. BEMF feedforward + cross-coupling decoupling */
    if (!in_direct_mode) {
        float omega_e = (theta_elec - f->prev_theta_elec) * f->pwm_freq;
        if (omega_e > 1e-6f || omega_e < -1e-6f) {
            f->vd -= omega_e * f->motor_Lq * f->iq_meas;
            f->vq += omega_e * f->motor_Ld * f->id_meas;
            f->vq += omega_e * f->bemf_v_per_hz;
        }
        f->prev_theta_elec = theta_elec;
    }

    /* 5c. VBUS for SVM reference */

    /* 6. Inverse Park: dq → αβ */
    float valpha, vbeta;
    inverse_park(f->vd, f->vq, sin_th, cos_th, &valpha, &vbeta);

    /* 7. SVM: αβ → duty cycles */
    svm(valpha, vbeta, vbus_svm, &f->duty_a, &f->duty_b, &f->duty_c);

    /* 8. Write PWM duties */
    write_pwm_duties(f->duty_a, f->duty_b, f->duty_c);
}

void FOC_Enable(void) {
    if (g_foc.enabled) return;
    g_foc.enabled = true;

    /* Start PWM channels (forcing 0% CCR beforehand ensures safe start) */
    write_pwm_duties(0.0f, 0.0f, 0.0f);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

void FOC_Disable(void) {
    g_foc.enabled = false;

    /* Write 0% duty before stopping */
    write_pwm_duties(0.0f, 0.0f, 0.0f);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
}

void FOC_SetDt(float dt) {
    if (dt > 0.0f) {
        g_foc.dt = dt;
        g_foc.pwm_freq = 1.0f / dt;
    }
}

void FOC_SetDirectVd(float vd, float vq) {
    g_foc.direct_voltage_mode = true;
    g_foc.vd_direct = vd;
    g_foc.vq_direct = vq;
}
