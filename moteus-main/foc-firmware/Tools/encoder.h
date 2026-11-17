/**
 * @file encoder.h
 * @brief TIM1 Quadrature Encoder Interface (STM32G4 HAL)
 *
 * TIM1 is configured by CubeMX in Encoder Mode (TI1 or TI1+TI2):
 *   PA8 = TIM1_CH1 (Encoder A)
 *   PA9 = TIM1_CH2 (Encoder B)
 *
 * Features:
 *   - 16-bit raw counter with signed overflow handling (int16_t delta)
 *   - Accumulated 32-bit signed position (unwrapped)
 *   - Velocity estimation via delta / dt (rad/s)
 *   - 1st-order low-pass filter on velocity
 *   - Electrical angle computation (radians)
 *
 * CPR and pole pairs are configurable at init.
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "main.h"
#include "encoder_pll.h"
#include <stdint.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim1;

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Defaults
 * --------------------------------------------------------------------------- */
#define ENCODER_CPR_DEFAULT            4000U   /**< counts/rev (1000 PPR × 4) */
#define ENCODER_POLE_PAIRS_DEFAULT     7U      /**< typical gimbal BLDC */
#define ENCODER_VELOCITY_LPF_ALPHA     0.15f   /**< LPF coefficient (0..1)   */
#define ENCODER_UPDATE_FREQ_DEFAULT    20000.0f/**< typical FOC rate [Hz]    */

/* ---------------------------------------------------------------------------
 * Configuration
 * --------------------------------------------------------------------------- */
typedef struct {
  uint32_t cpr;                /**< Counts per mechanical revolution         */
  uint32_t pole_pairs;         /**< Motor pole pairs                        */
  float    velocity_lpf_alpha; /**< LPF coefficient (0=heavy, 1=no filter) */
  float    update_freq;        /**< Expected Encoder_Update rate [Hz]       */
} Encoder_Config;

/* ---------------------------------------------------------------------------
 * State
 * --------------------------------------------------------------------------- */
typedef struct {
  Encoder_Config config;

  /* Raw counter tracking */
  uint16_t last_raw;           /**< Previous TIM1 counter value             */
  int32_t  position;           /**< Accumulated position (unwrapped counts)  */

  /* Velocity (rad/s) */
  float    velocity;           /**< Instantaneous velocity [rad/s]           */
  float    velocity_filt;      /**< Low-pass filtered velocity [rad/s]      */

  /* Electrical angle */
  float    elec_angle;         /**< Electrical angle [rad] in [0, 2pi)      */

  bool     initialized;
} Encoder_TypeDef;

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
extern const Encoder_Config Encoder_ConfigDefault;

/* ---------------------------------------------------------------------------
 * API
 * --------------------------------------------------------------------------- */

/** @brief Initialise encoder with given configuration.
 *         Reads the current TIM1 counter as the zero reference.
 *  @param enc  Encoder instance
 *  @param cfg  Configuration (NULL = use defaults)
 */
void Encoder_Init(Encoder_TypeDef *enc, const Encoder_Config *cfg);

/** @brief Update position, velocity, and electrical angle.
 *         Must be called at a constant rate (config.update_freq).
 *         Typically called from the FOC control loop ISR or timer callback.
 */
void Encoder_Update(Encoder_TypeDef *enc);

/** @brief Force absolute position [counts].
 *         Use with AS5600 reading to initialise absolute position on startup.
 */
void Encoder_SetPosition(Encoder_TypeDef *enc, int32_t pos);

/** @brief Reset accumulated position to zero without touching TIM1 counter. */
void Encoder_ResetPosition(Encoder_TypeDef *enc);

/* ---------------------------------------------------------------------------
 * Inline helpers
 * --------------------------------------------------------------------------- */

/** @brief Read raw 16-bit TIM1 counter (for debugging / calibration) */
static inline uint16_t Encoder_GetRawCount(void) {
  return __HAL_TIM_GET_COUNTER(&htim1);
}

/** @brief Reset TIM1 hardware counter to zero */
static inline void Encoder_ResetRawCount(void) {
  __HAL_TIM_SET_COUNTER(&htim1, 0);
}

/** @brief Get accumulated position in counts */
static inline int32_t Encoder_GetPosition(const Encoder_TypeDef *enc) {
  return enc->position;
}

/** @brief Get filtered velocity [rad/s] */
static inline float Encoder_GetVelocity(const Encoder_TypeDef *enc) {
  return enc->velocity_filt;
}

/** @brief Get electrical angle [rad] in [0, 2pi) */
static inline float Encoder_GetElectricalAngle(const Encoder_TypeDef *enc) {
  return enc->elec_angle;
}

/* ---------------------------------------------------------------------------
 * PLL-filtered velocity
 * --------------------------------------------------------------------------- */

/** @brief Global PLL instance (initialised by Encoder_Init). */
extern EncoderPLL g_encoder_pll;

/** @brief Get PLL-filtered velocity [rev/s] (mechanical).
 *
 *  The PLL tracks the mechanical angle and produces a smooth velocity
 *  estimate via its PI + VCO structure.  This replaces the 1st-order
 *  LPF from Encoder_GetVelocity() with zero-lag ramp tracking.
 *
 *  @return Mechanical velocity [rev/s]
 */
static inline float Encoder_GetPLLVelocity(void) {
  return g_encoder_pll.omega / 6.283185307179586f;
}

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H_ */
