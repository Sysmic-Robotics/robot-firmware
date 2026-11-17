/**
 * @file pid.h
 * @brief PID controllers ported from moteus fw/pid.h + fw/simple_pi.h
 *
 * Three controllers in C:
 *   1. PID — Full PID with derivative on error_rate, anti-windup, desired ramping
 *   2. SimplePI — PI with fixed -1 sign, for d/q current loops
 *   3. ExponentialFilter — First-order low-pass filter
 *
 * Reference: moteus fw/pid.h, fw/simple_pi.h (Apache 2.0)
 *
 * Hardware: STM32G431CBU6 @ 170 MHz
 */

#ifndef MOT_PID_H_
#define MOT_PID_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Utility helpers (ported from bldc_servo_control.h)
 * --------------------------------------------------------------------------- */

/** @brief Clamp value to [min, max]. */
static inline float pid_Limit(float a, float min, float max) {
    if (a < min) return min;
    if (a > max) return max;
    return a;
}

/** @brief Threshold: zero if value in (lower, upper), else value. */
static inline float pid_Threshold(float value, float lower, float upper) {
    return (value > lower && value < upper) ? 0.0f : value;
}

/* ===================================================================
 * 1. ExponentialFilter — first-order low-pass
 * =================================================================== */

typedef struct {
    float alpha;
    float one_minus_alpha;
} ExponentialFilter;

/** @brief Initialise filter given control rate and cutoff.
 *  @param rate_hz    Control loop frequency [Hz]
 *  @param cutoff_hz  Desired -3 dB bandwidth [Hz]
 *                    (actual cutoff ≈ cutoff_hz / 2π)
 */
static inline void ExponentialFilter_Init(
    ExponentialFilter* f, float rate_hz, float cutoff_hz)
{
    f->alpha = cutoff_hz / rate_hz;
    f->one_minus_alpha = 1.0f - f->alpha;
}

/** @brief Apply filter to one sample.
 *  @param input     Raw input value
 *  @param filtered  In/out: previous filtered value, replaced on output
 */
static inline void ExponentialFilter_Apply(
    const ExponentialFilter* f, float input, float* filtered)
{
    if (!isfinite(*filtered)) {
        *filtered = input;
    } else {
        *filtered = f->alpha * input + f->one_minus_alpha * *filtered;
    }
}

/* ===================================================================
 * 2. SimplePI — for d/q current loops (output sign = -1)
 * =================================================================== */

typedef struct {
    float kp;              /**< proportional gain                        */
    float ki;              /**< integral gain                            */
    float max_desired_rate; /**< setpoint slew rate [units/s]; 0 = off  */
} SimplePI_Config;

typedef struct {
    float integral;        /**< integrator state                         */
    float desired;         /**< last desired value (for ramping)         */
    float error;           /**< last error (for telemetry)               */
    float p;               /**< last proportional term                   */
    float command;         /**< last output command                      */
} SimplePI_State;

/** @brief Reset SimplePI state to zero. */
static inline void SimplePI_Reset(SimplePI_State* s) {
    s->integral = 0.0f;
    s->desired  = NAN;
    s->error    = 0.0f;
    s->p        = 0.0f;
    s->command  = 0.0f;
}

/** @brief Apply SimplePI controller.
 *
 *  output = -1 * (kp * error + integral)
 *
 *  @param measured      Measured process variable
 *  @param input_desired Target setpoint
 *  @param period_s      Sample period [s]
 *  @return  Command output (negative of PI error sum)
 */
static inline float SimplePI_Apply(
    const SimplePI_Config* cfg, SimplePI_State* s,
    float measured, float input_desired, float period_s)
{
    float desired;

    if (cfg->max_desired_rate != 0.0f && isfinite(s->desired)) {
        float max_step = cfg->max_desired_rate * period_s;
        float proposed_step = input_desired - s->desired;
        float actual_step = pid_Limit(proposed_step, -max_step, max_step);
        desired = s->desired + actual_step;
    } else {
        desired = input_desired;
    }

    s->desired = desired;
    s->error = measured - desired;

    s->integral += s->error * cfg->ki * period_s;

    s->p = cfg->kp * s->error;
    s->command = -1.0f * (s->p + s->integral);

    return s->command;
}

/* ===================================================================
 * 3. PID — Full PID with derivative, anti-windup, rate limiting
 * =================================================================== */

typedef struct {
    float kp;               /**< proportional gain                       */
    float ki;               /**< integral gain                           */
    float kd;               /**< derivative gain                         */
    float iratelimit;       /**< integrator update rate limit [units/s];
                                 <=0 means unlimited                      */
    float ilimit;           /**< integrator windup clamp ±ilimit         */
    float max_desired_rate; /**< setpoint slew rate [units/s]; 0=unlim   */
    int8_t sign;            /**< output sign multiplier (±1)             */
} PID_Config;

typedef struct {
    float integral;         /**< integrator state                        */
    float desired;          /**< last desired (for ramping), NaN=accept  */
    float error;            /**< last position error                     */
    float error_rate;       /**< last velocity error                     */
    float p;                /**< last proportional term                  */
    float d;                /**< last derivative term                    */
    float pd;               /**< last p+d sum                            */
    float command;          /**< last output command                     */
} PID_State;

/** @brief Reset PID state to zero (fast, no memset). */
static inline void PID_Reset(PID_State* s) {
    s->integral   = 0.0f;
    s->desired    = NAN;
    s->error      = 0.0f;
    s->error_rate = 0.0f;
    s->p          = 0.0f;
    s->d          = 0.0f;
    s->pd         = 0.0f;
    s->command    = 0.0f;
}

typedef struct {
    float kp_scale;
    float kd_scale;
    float ilimit_scale;
} PID_ApplyOptions;

static inline void PID_ApplyOptions_Init(PID_ApplyOptions* o) {
    o->kp_scale = 1.0f;
    o->kd_scale = 1.0f;
    o->ilimit_scale = 1.0f;
}

/** @brief Apply full PID controller.
 *
 *  Implements the moteus PID algorithm:
 *    1. Rate-limit desired setpoint (optional)
 *    2. error = measured - desired
 *    3. error_rate = measured_rate - desired_rate
 *    4. Integrate with iratelimit anti-windup
 *    5. Clamp integrator to ±ilimit
 *    6. p = kp_scale * kp * error
 *    7. d = kd_scale * kd * error_rate
 *    8. output = sign * (p + d + integral)
 *
 *  @param measured          Position feedback
 *  @param input_desired     Position setpoint
 *  @param measured_rate     Velocity feedback (d/dt of measured)
 *  @param input_desired_rate Velocity feedforward (d/dt of desired)
 *  @param period_s          Sample time [s]
 *  @param opts              ApplyOptions (NULL = defaults)
 *  @return Command output
 */
static inline float PID_Apply(
    const PID_Config* cfg, PID_State* s,
    float measured, float input_desired,
    float measured_rate, float input_desired_rate,
    float period_s, const PID_ApplyOptions* opts)
{
    float kp_scale = 1.0f;
    float kd_scale = 1.0f;
    float ilimit_scale = 1.0f;

    if (opts) {
        kp_scale = opts->kp_scale;
        kd_scale = opts->kd_scale;
        ilimit_scale = opts->ilimit_scale;
    }

    // --- 1. Rate-limit desired setpoint
    float desired, desired_rate;

    if (cfg->max_desired_rate != 0.0f && isfinite(s->desired)) {
        float max_step = cfg->max_desired_rate * period_s;
        float proposed_step = input_desired - s->desired;
        float actual_step = pid_Limit(proposed_step, -max_step, max_step);
        desired = s->desired + actual_step;
        desired_rate = pid_Limit(input_desired_rate,
                                 -cfg->max_desired_rate, cfg->max_desired_rate);
    } else {
        desired = input_desired;
        desired_rate = input_desired_rate;
    }

    s->desired = desired;

    // --- 2. Errors
    s->error = measured - desired;
    s->error_rate = measured_rate - desired_rate;

    // --- 3. Integrate with rate limit
    float max_i_update = cfg->iratelimit * period_s;
    float to_update_i = s->error * cfg->ki * period_s;
    if (max_i_update > 0.0f) {
        to_update_i = pid_Limit(to_update_i, -max_i_update, max_i_update);
    }
    s->integral += to_update_i;

    // --- 4. Clamp integrator
    float ilimit = cfg->ilimit * ilimit_scale;
    s->integral = pid_Limit(s->integral, -ilimit, ilimit);

    // --- 5. Proportional + derivative
    s->p  = kp_scale * cfg->kp * s->error;
    s->d  = kd_scale * cfg->kd * s->error_rate;
    s->pd = s->p + s->d;

    // --- 6. Output
    s->command = (float)cfg->sign * (s->pd + s->integral);

    return s->command;
}

#ifdef __cplusplus
}
#endif

#endif /* MOT_PID_H_ */
