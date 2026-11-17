/**
 * @file encoder.c
 * @brief TIM1 Quadrature Encoder Interface — Implementation
 *
 * Handles 16-bit raw counter overflow using signed int16_t delta,
 * accumulates position in int32_t, and computes velocity + electrical
 * angle as floats (STM32G4 FPU).
 */

#include "encoder.h"

/* ---------------------------------------------------------------------------
 * Default configuration
 * --------------------------------------------------------------------------- */
const Encoder_Config Encoder_ConfigDefault = {
  .cpr                = ENCODER_CPR_DEFAULT,
  .pole_pairs         = ENCODER_POLE_PAIRS_DEFAULT,
  .velocity_lpf_alpha = ENCODER_VELOCITY_LPF_ALPHA,
  .update_freq        = ENCODER_UPDATE_FREQ_DEFAULT,
};

/* ---------------------------------------------------------------------------
 * Global PLL instance
 * --------------------------------------------------------------------------- */
EncoderPLL g_encoder_pll;

/* ---------------------------------------------------------------------------
 * Private helpers
 * --------------------------------------------------------------------------- */

/** Wrap a signed value to [0, modulus) — handles negative. */
static inline int32_t wrap_mod(int32_t val, int32_t modulus) {
  int32_t r = val % modulus;
  return (r < 0) ? r + modulus : r;
}

/** Two-pi constant for float */
#define FOC_2PI  (6.283185307179586f)

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

void Encoder_Init(Encoder_TypeDef *enc, const Encoder_Config *cfg) {
  if (cfg != NULL) {
    enc->config = *cfg;
  } else {
    enc->config = Encoder_ConfigDefault;
  }

  /* Sanity — prevent division by zero */
  if (enc->config.cpr == 0)                enc->config.cpr = ENCODER_CPR_DEFAULT;
  if (enc->config.pole_pairs == 0)         enc->config.pole_pairs = ENCODER_POLE_PAIRS_DEFAULT;
  if (enc->config.update_freq <= 0.0f)     enc->config.update_freq = ENCODER_UPDATE_FREQ_DEFAULT;
  if (enc->config.velocity_lpf_alpha < 0.001f) enc->config.velocity_lpf_alpha = 0.001f;
  if (enc->config.velocity_lpf_alpha > 1.0f)   enc->config.velocity_lpf_alpha = 1.0f;

  /* Seed from current hardware counter */
  enc->last_raw  = __HAL_TIM_GET_COUNTER(&htim1);
  enc->position  = 0;
  enc->velocity  = 0.0f;
  enc->velocity_filt = 0.0f;
  enc->elec_angle     = 0.0f;
  enc->initialized    = true;

  /* Initialise the PLL with default bandwidth and the configured update rate */
  EncoderPLL_Init(&g_encoder_pll,
                  ENCODER_PLL_BW_HZ_DEFAULT,
                  1.0f / enc->config.update_freq);
}

void Encoder_Update(Encoder_TypeDef *enc) {
  if (!enc->initialized) return;

  const float cpr_f      = (float)enc->config.cpr;
  const float freq       = enc->config.update_freq;
  const float alpha      = enc->config.velocity_lpf_alpha;
  const float one_m_alpha = 1.0f - alpha;

  /* ---- 1. Read raw counter and compute delta (handles 16-bit wrap) ---- */
  uint16_t raw    = __HAL_TIM_GET_COUNTER(&htim1);
  int16_t  delta  = (int16_t)(raw - enc->last_raw);

  enc->last_raw   = raw;
  enc->position  += delta;

  /* ---- 2. Velocity (rad/s) ---- */
  /* delta is in counts per sample interval.
   * velocity = delta_counts * (2pi / CPR) * freq           [rad/s] */
  float vel_raw = (float)delta * (FOC_2PI / cpr_f) * freq;
  enc->velocity = vel_raw;

  /* 1st-order low-pass */
  enc->velocity_filt = alpha * vel_raw + one_m_alpha * enc->velocity_filt;

  /* ---- 3. Electrical angle ---- */
  /* Mechanical portion within one revolution [counts, 0..CPR) */
  int32_t pos_mod_cpr   = wrap_mod(enc->position, (int32_t)enc->config.cpr);
  /* Scale by pole pairs, modulo CPR to get electrical angle in counts */
  int32_t elec_counts   = (pos_mod_cpr * (int32_t)enc->config.pole_pairs)
                          % (int32_t)enc->config.cpr;
  /* Convert counts to radians in [0, 2π) */
  enc->elec_angle = (float)elec_counts * (FOC_2PI / cpr_f);

  /* ---- 4. PLL update (smooth velocity from mechanical angle) ---- */
  /* Mechanical angle [rad] in [0, 2π) — unwrapped from position modulo CPR */
  float mech_angle = (float)pos_mod_cpr * (FOC_2PI / cpr_f);
  EncoderPLL_Update(&g_encoder_pll, mech_angle);
}

void Encoder_SetPosition(Encoder_TypeDef *enc, int32_t pos) {
  enc->position  = pos;
  /* Re-sync last_raw so next delta starts from zero */
  enc->last_raw  = __HAL_TIM_GET_COUNTER(&htim1);
}

void Encoder_ResetPosition(Encoder_TypeDef *enc) {
  Encoder_SetPosition(enc, 0);
}
