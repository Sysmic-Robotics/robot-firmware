/**
 * @file servo_control.c
 * @brief Servo Control State Machine — Implementation
 *
 * Reference: moteus fw/bldc_servo_control.h (Apache 2.0)
 *
 * Architecture (runs at PWM rate, e.g. 20 kHz):
 *   1. Sample phase currents + encoder → d_A, q_A (foc.c)
 *   2. ServoControl_Run → mode dispatch → i_d_ref, i_q_ref
 *   3. Current PI control + Inverse Park + SVM + PWM (foc.c)
 */

#include "servo_control.h"
#include "foc.h"
#include <string.h>
#include <float.h>

/* ===================================================================
 * Constants
 * =================================================================== */
#define K_2PI            (6.283185307179586f)
#define K_1_OVER_3       (0.3333333333333333f)

/* Default current PI: bandwidth-based tuning default */
#define DEFAULT_PID_DQ_HZ       100.0f
#define DEFAULT_CURRENT_KP      0.08f
#define DEFAULT_CURRENT_KI      4.0f
#define DEFAULT_CURRENT_LIMIT_A 30.0f
#define DEFAULT_CURRENT_ILIMIT  12.0f
#define DEFAULT_MAX_CURRENT_RATE 10000.0f

/* Default velocity PID */
#define DEFAULT_VEL_KP   0.5f
#define DEFAULT_VEL_KI   2.0f
#define DEFAULT_VEL_KD   0.0f
#define DEFAULT_VEL_ILIM 5.0f
#define DEFAULT_VEL_IRATE_LIMIT 0.0f
#define DEFAULT_VEL_MAX_RATE    0.0f

/* Default position PID */
#define DEFAULT_POS_KP   4.0f
#define DEFAULT_POS_KI   1.0f
#define DEFAULT_POS_KD   0.05f
#define DEFAULT_POS_ILIM 0.0f
#define DEFAULT_POS_IRATE_LIMIT 0.0f
#define DEFAULT_POS_MAX_RATE    0.0f

/* Limits */
#define DEFAULT_MAX_VELOCITY        500.0f
#define DEFAULT_MAX_VEL_DERATE      2.0f
#define DEFAULT_MAX_VOLTAGE         48.0f
#define DEFAULT_FAULT_TEMPERATURE   85.0f
#define DEFAULT_TIMEOUT_S           0.1f
#define DEFAULT_TIMEOUT_MODE        12   /* kZeroVelocity */

/* Cooldown */
#define DEFAULT_COOLDOWN_CYCLES     256
#define DEFAULT_COOLDOWN_BRAKE      64
#define OVERCURRENT_MARGIN          1.2f

/* ===================================================================
 * Internal helpers
 * =================================================================== */

/** Clamp val to [min, max]. */
static inline float clampf(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/** Threshold: zero if value in (lower, upper), else value. */
static inline float thresholdf(float value, float lower, float upper) {
    return (value > lower && value < upper) ? 0.0f : value;
}

/* ===================================================================
 * Config defaults
 * =================================================================== */

void ServoConfig_Init(ServoConfig* cfg) {
    memset(cfg, 0, sizeof(*cfg));

    /* Motor */
    cfg->motor.poles           = 7;
    cfg->motor.resistance_ohm  = 0.1f;
    cfg->motor.inductance_d_H  = 1e-5f;
    cfg->motor.inductance_q_H  = 1e-5f;
    cfg->motor.torque_constant = 0.05f;
    cfg->motor.v_per_hz        = 0.0f;

    /* Current control */
    cfg->pid_dq_hz             = DEFAULT_PID_DQ_HZ;
    cfg->pid_current_kp        = DEFAULT_CURRENT_KP;
    cfg->pid_current_ki        = DEFAULT_CURRENT_KI;
    cfg->current_limit_A       = DEFAULT_CURRENT_LIMIT_A;
    cfg->current_ilimit_A      = DEFAULT_CURRENT_ILIMIT;
    cfg->max_current_desired_rate = DEFAULT_MAX_CURRENT_RATE;

    /* Velocity PID */
    cfg->pid_velocity.kp       = DEFAULT_VEL_KP;
    cfg->pid_velocity.ki       = DEFAULT_VEL_KI;
    cfg->pid_velocity.kd       = DEFAULT_VEL_KD;
    cfg->pid_velocity.ilimit   = DEFAULT_VEL_ILIM;
    cfg->pid_velocity.iratelimit = DEFAULT_VEL_IRATE_LIMIT;
    cfg->pid_velocity.max_desired_rate = DEFAULT_VEL_MAX_RATE;
    cfg->pid_velocity.sign     = -1;

    /* Position PID */
    cfg->pid_position.kp       = DEFAULT_POS_KP;
    cfg->pid_position.ki       = DEFAULT_POS_KI;
    cfg->pid_position.kd       = DEFAULT_POS_KD;
    cfg->pid_position.ilimit   = DEFAULT_POS_ILIM;
    cfg->pid_position.iratelimit = DEFAULT_POS_IRATE_LIMIT;
    cfg->pid_position.max_desired_rate = DEFAULT_POS_MAX_RATE;
    cfg->pid_position.sign     = -1;

    /* Feedforward */
    cfg->current_feedforward   = 1.0f;
    cfg->bemf_feedforward      = 0.0f;
    cfg->inertia_feedforward   = 0.0f;

    /* Limits */
    cfg->max_velocity          = DEFAULT_MAX_VELOCITY;
    cfg->max_velocity_derate   = DEFAULT_MAX_VEL_DERATE;
    cfg->max_power_W           = INFINITY;
    cfg->max_voltage           = DEFAULT_MAX_VOLTAGE;

    /* Fault thresholds */
    cfg->fault_temperature      = DEFAULT_FAULT_TEMPERATURE;
    cfg->fault_position_error   = INFINITY;
    cfg->fault_velocity_error   = INFINITY;

    /* Timeout */
    cfg->default_timeout_s      = DEFAULT_TIMEOUT_S;
    cfg->timeout_mode           = DEFAULT_TIMEOUT_MODE;

    /* Cooldown */
    cfg->cooldown_cycles        = DEFAULT_COOLDOWN_CYCLES;
    cfg->cooldown_brake_cycles  = DEFAULT_COOLDOWN_BRAKE;

    /* Stall detection */
    cfg->stall_velocity_threshold   = 0.5f;
    cfg->stall_current_threshold    = DEFAULT_CURRENT_LIMIT_A * 0.8f;
    cfg->stall_timeout_cycles       = 5000;
}

/* ===================================================================
 * Gain computation (ported from moteus PID tuning)
 * =================================================================== */

/** Compute current PI gains from bandwidth and motor params.
 *  Uses the moteus approach: ω_bw = 2π * pid_dq_hz
 *    kp = ω_bw * L
 *    ki = ω_bw * R
 *  Then scales kp up by 1.5 (crossover shaping) per moteus convention.
 */
static void compute_current_gains(ServoControl* sc) {
    const float resistance = sc->config.motor.resistance_ohm;
    const float induct_d   = sc->config.motor.inductance_d_H;
    const float induct_q   = sc->config.motor.inductance_q_H;

    const float bw = K_2PI * sc->config.pid_dq_hz;

    // Use the larger inductance for both axes (conservative)
    const float L = (induct_d > induct_q) ? induct_d : induct_q;

    if (L > 1e-12f && resistance > 1e-12f) {
        // moteus-style tuning: kp = bw * L * 1.5, ki = bw * R
        const float kp = bw * L * 1.5f;
        const float ki = bw * resistance;

        sc->pid_d_cfg.kp = kp;
        sc->pid_d_cfg.ki = ki;
        sc->pid_q_cfg.kp = kp;
        sc->pid_q_cfg.ki = ki;

        // Store for diagnostic
        sc->config.pid_current_kp = kp;
        sc->config.pid_current_ki = ki;
    } else {
        // Fall back to configured defaults
        sc->pid_d_cfg.kp = sc->config.pid_current_kp;
        sc->pid_d_cfg.ki = sc->config.pid_current_ki;
        sc->pid_q_cfg.kp = sc->config.pid_current_kp;
        sc->pid_q_cfg.ki = sc->config.pid_current_ki;
    }

    sc->pid_d_cfg.max_desired_rate = sc->config.max_current_desired_rate;
    sc->pid_q_cfg.max_desired_rate = sc->config.max_current_desired_rate;
}

/* ===================================================================
 * Mode transition logic (ISR_MaybeChangeMode equivalent)
 * =================================================================== */

static bool control_allows_torque(ServoMode mode) {
    switch (mode) {
        case SERVO_MODE_CURRENT:
        case SERVO_MODE_POSITION:
        case SERVO_MODE_ZERO_VELOCITY:
        case SERVO_MODE_STAY_WITHIN_BOUNDS:
        case SERVO_MODE_BRAKE:
            return true;
        default:
            return false;
    }
}

static bool current_pid_active(ServoMode mode) {
    switch (mode) {
        case SERVO_MODE_CURRENT:
        case SERVO_MODE_POSITION:
        case SERVO_MODE_POSITION_TIMEOUT:
        case SERVO_MODE_ZERO_VELOCITY:
        case SERVO_MODE_STAY_WITHIN_BOUNDS:
            return true;
        default:
            return false;
    }
}

static bool position_pid_active(ServoMode mode) {
    switch (mode) {
        case SERVO_MODE_POSITION:
        case SERVO_MODE_POSITION_TIMEOUT:
        case SERVO_MODE_ZERO_VELOCITY:
        case SERVO_MODE_STAY_WITHIN_BOUNDS:
            return true;
        default:
            return false;
    }
}

/** Clear PID states that should be inactive for the current mode. */
static void clear_stale_pids(ServoControl* sc) {
    if (!current_pid_active(sc->status.mode)) {
        SimplePI_Reset(&sc->pid_d_state);
        SimplePI_Reset(&sc->pid_q_state);
    }
    if (!position_pid_active(sc->status.mode)) {
        PID_Reset(&sc->pid_pos_state);
        sc->status.control_position = NAN;
        sc->status.control_velocity = NAN;
        sc->status.control_accel    = 0.0f;
    }
}

/** Handle mode transition requested by SetCommand.
 *  Implements the moteus state machine rules.
 */
static void maybe_change_mode(ServoControl* sc, ServoMode requested) {
    // Validate request
    switch (requested) {
        case SERVO_MODE_NUM_MODES:
        case SERVO_MODE_FAULT:
        case SERVO_MODE_CALIBRATING:
        case SERVO_MODE_CALIB_COMPLETE:
            return;  // cannot be commanded directly
        default:
            break;
    }

    if (requested == SERVO_MODE_STOPPED) {
        sc->status.mode = SERVO_MODE_STOPPED;
        return;
    }

    ServoMode current = sc->status.mode;
    switch (current) {
        case SERVO_MODE_FAULT:
            return;  // must clear fault first
        case SERVO_MODE_STOPPED: {
            // Transition via enabling (simplified: skip calibration)
            sc->status.mode = requested;
            clear_stale_pids(sc);
            // Initialize control position from measured position
            if (position_pid_active(requested)) {
                // Will be initialized on first run with position data
                sc->status.control_velocity = 0.0f;
                sc->status.control_accel    = 0.0f;
            }
            return;
        }
        default: {
            // Direct mode transition
            sc->status.mode = requested;
            clear_stale_pids(sc);
            return;
        }
    }
}

/* ===================================================================
 * Mode dispatch implementations
 * =================================================================== */

/** kStopped / kFault: disable output, optionally brake then HiZ. */
static void do_stopped(ServoControl* sc, float* i_d_ref, float* i_q_ref) {
    if (sc->cooldown_count > 0) {
        if (sc->cooldown_count > sc->config.cooldown_brake_cycles) {
            // Short all phases (brake) — use positive i_d to dissipate
            *i_d_ref = 0.0f;
            *i_q_ref = 0.0f;
        } else {
            // HiZ: 0 current
            *i_d_ref = 0.0f;
            *i_q_ref = 0.0f;
        }
        sc->cooldown_count--;
    } else {
        *i_d_ref = 0.0f;
        *i_q_ref = 0.0f;
    }
    sc->status.power_W = 0.0f;
}

/** kBrake: all phases shorted (i_d = i_q = 0, so PI produces 0 voltage). */
static void do_brake(ServoControl* sc, float* i_d_ref, float* i_q_ref) {
    *i_d_ref = 0.0f;
    *i_q_ref = 0.0f;
}

/** kCurrent: direct d/q current control with limiting. */
static void do_current(ServoControl* sc,
                       float i_d_cmd, float i_q_cmd,
                       float* i_d_ref, float* i_q_ref)
{
    // Apply current limits
    const float i_limit = sc->config.current_limit_A;
    i_q_cmd = clampf(i_q_cmd, -i_limit, i_limit);
    i_d_cmd = clampf(i_d_cmd, -i_limit, i_limit);

    // Rate-limit through SimplePI's max_desired_rate
    *i_d_ref = i_d_cmd;
    *i_q_ref = i_q_cmd;
}

/** kVelocity: PI velocity → current. */
static void do_velocity(ServoControl* sc,
                        float measured_velocity, float target_velocity,
                        float* i_d_ref, float* i_q_ref)
{
    // Apply velocity limit
    if (sc->config.max_velocity > 0.0f) {
        target_velocity = clampf(target_velocity,
                                  -sc->config.max_velocity,
                                   sc->config.max_velocity);
    }

    // Velocity PID: measured = velocity, desired = target_velocity
    // PID outputs torque (Nm), converted to current
    float torque_Nm = PID_Apply(
        &sc->pid_vel_cfg, &sc->pid_vel_state,
        measured_velocity, target_velocity,
        0.0f, 0.0f,                      // no rate feedforward yet
        sc->period_s, NULL);

    // Convert torque to current
    const float Kt = sc->config.motor.torque_constant;
    float i_q_cmd = 0.0f;
    if (Kt > 1e-12f) {
        i_q_cmd = torque_Nm / Kt;
    }

    // Apply current limit
    const float i_limit = sc->config.current_limit_A;
    i_q_cmd = clampf(i_q_cmd, -i_limit, i_limit);

    *i_d_ref = 0.0f;
    *i_q_ref = i_q_cmd;
}

/** kPosition: PID position → velocity → PID velocity → current. */
static void do_position(ServoControl* sc,
                        float measured_position, float measured_velocity,
                        ServoCommandData* cmd,
                        float* i_d_ref, float* i_q_ref)
{
    // --- 1. Initialize control position on first run ---
    if (!isfinite(sc->status.control_position)) {
        sc->status.control_position = measured_position;
        sc->status.control_velocity = 0.0f;
        sc->status.control_accel    = 0.0f;
    }

    // --- 2. Position PID: error → velocity command ---
    // Position error in revolutions
    float pos_error = measured_position - sc->status.control_position;

    // Apply velocity_limit and accel_limit from command (simplified trajectory)
    float target_velocity;
    if (cmd->velocity_limit > 0.0f) {
        // Use velocity limit
        target_velocity = clampf(cmd->velocity, -cmd->velocity_limit, cmd->velocity_limit);
    } else {
        target_velocity = cmd->velocity;
    }

    // Simple position PID: error → velocity
    // Use the PID as an error regulator (measured=pos_error, desired=0)
    PID_ApplyOptions opts;
    PID_ApplyOptions_Init(&opts);
    opts.kp_scale = cmd->kp_scale;
    opts.kd_scale = cmd->kd_scale;
    opts.ilimit_scale = cmd->ilimit_scale;

    float velocity_cmd = PID_Apply(
        &sc->pid_pos_cfg, &sc->pid_pos_state,
        pos_error,       // measured = position error
        0.0f,            // desired = 0 (we want error → 0)
        measured_velocity, target_velocity,  // rates
        sc->period_s, &opts);

    // --- 3. Velocity clamping ---
    if (sc->config.max_velocity > 0.0f) {
        velocity_cmd = clampf(velocity_cmd,
                               -sc->config.max_velocity,
                                sc->config.max_velocity);
    }
    if (cmd->velocity_limit > 0.0f) {
        velocity_cmd = clampf(velocity_cmd,
                               -cmd->velocity_limit,
                                cmd->velocity_limit);
    }

    sc->status.control_velocity = velocity_cmd;
    sc->status.control_position = measured_position;  // update for next cycle

    // --- 4. Velocity PID ---
    float torque_Nm = PID_Apply(
        &sc->pid_vel_cfg, &sc->pid_vel_state,
        measured_velocity, velocity_cmd,
        0.0f, 0.0f,
        sc->period_s, NULL);

    // --- 5. Feedforward ---
    torque_Nm += cmd->feedforward_Nm;
    torque_Nm += K_2PI * sc->status.control_accel * sc->config.inertia_feedforward;

    // --- 6. Torque limit ---
    if (cmd->max_torque_Nm > 0.0f) {
        torque_Nm = clampf(torque_Nm, -cmd->max_torque_Nm, cmd->max_torque_Nm);
    }

    // --- 7. Torque → current ---
    const float Kt = sc->config.motor.torque_constant;
    float i_q_cmd = 0.0f;
    if (Kt > 1e-12f) {
        i_q_cmd = torque_Nm / Kt;
    }

    // --- 8. Current limit ---
    const float i_limit = sc->config.current_limit_A;
    i_q_cmd = clampf(i_q_cmd, -i_limit, i_limit);

    *i_d_ref = 0.0f;
    *i_q_ref = i_q_cmd;

    // --- 9. Position/velocity error fault check ---
    if (isfinite(sc->config.fault_position_error) &&
        fabsf(pos_error) > sc->config.fault_position_error) {
        sc->status.fault = SERVO_FAULT_POSITION_ERROR;
        sc->status.mode = SERVO_MODE_FAULT;
    }
    if ((sc->status.mode != SERVO_MODE_FAULT) &&
        isfinite(sc->config.fault_velocity_error)) {
        float vel_error = fabsf(measured_velocity - velocity_cmd);
        if (vel_error > sc->config.fault_velocity_error) {
            sc->status.fault = SERVO_FAULT_VELOCITY_ERROR;
            sc->status.mode = SERVO_MODE_FAULT;
        }
    }
}

/* ===================================================================
 * Calibration sequence implementation
 * =================================================================== */

static void do_calibrating_rs(ServoControl* sc,
                              float* i_d_ref, float* i_q_ref)
{
    /* Apply small Id reference to measure Rs from Vd/Id */
    const float id_target = 2.0f;  /* 2 A Id — below saturation */
    *i_d_ref = id_target;
    *i_q_ref = 0.0f;
    sc->cal_step++;

    /* Wait for current to settle (~5 ms at 20 kHz = 100 cycles), then measure */
    if (sc->cal_step == 100) {
        sc->status.control_position = 0.0f;  /* marker: starting measurement */
    }
    if (sc->cal_step > 3000) {  /* ~150 ms total */
        /* Average Vd / Id over last 200 cycles */
        float vd_sum  = 0.0f;
        float id_sum  = 0.0f;
        int   samples = 0;
        for (int i = 0; i < 200; i++) {
            /* We use the instantaneous values from the PID outputs.
             * At ωe=0 (rotor stopped), Vd ≈ Id·Rs. */
            (void)vd_sum; (void)id_sum; (void)samples;
        }
        /* Simple single-point estimate from last PI output */
        float Vd = sc->pid_d_state.command;  /* includes integrator */
        float Id = sc->status.d_A;
        if (fabsf(Id) > 0.1f) {
            float Rs_est = Vd / Id;
            if (Rs_est > 0.001f && Rs_est < 10.0f) {
                sc->config.motor.resistance_ohm = Rs_est;
            }
        }
        sc->cal_substate = CAL_SUB_THETA_OFFSET;
        sc->cal_step     = 0;
    }
}

static void do_calibrating_theta(ServoControl* sc,
                                 float measured_position,
                                 float theta_elec,
                                 float* i_d_ref, float* i_q_ref)
{
    /* Apply positive Id to create aligning torque (rotor snaps to d-axis) */
    *i_d_ref = 1.5f;  /* moderate alignment current */
    *i_q_ref = 0.0f;
    sc->cal_step++;

    if (sc->cal_step == 2000) {
        /* Capture electrical angle at aligned position */
        sc->cal_theta_elec = theta_elec;
    }
    if (sc->cal_step > 4000) {  /* ~200 ms */
        /* Theta offset = electrical angle when rotor is aligned to d=0.
         * We captured theta_elec at alignment. */
        sc->theta_offset = sc->cal_theta_elec;
        sc->cal_substate = CAL_SUB_LD_LQ;
        sc->cal_step     = 0;
    }
}

static void do_calibrating_offset(ServoControl* sc,
                                  float* i_d_ref, float* i_q_ref)
{
    /* HiZ — measure current ADC offset (Id and Iq should be ~0) */
    *i_d_ref = 0.0f;
    *i_q_ref = 0.0f;
    sc->cal_step++;

    if (sc->cal_step > 2000) {  /* ~100 ms settling */
        sc->cal_substate = CAL_SUB_COMPLETE;
        sc->cal_step     = 0;
    }
}

static void do_calibrating_ldlq(ServoControl* sc,
                                 float* i_d_ref, float* i_q_ref)
{
    /* Ld/Lq measurement via transient Vd step method.
     *
     * Algorithm:
     *   1. Phase 1 (cal_step 1-100):  Settle with Id=0, Iq=0.
     *   2. Phase 2 (cal_step 101-200): Apply Vd step (~3.6 V = 15% of VBUS).
     *      Accumulate first 20 Id samples for linear regression.
     *   3. Phase 3 (cal_step 201):     Compute Ld = Vd / (dId/dt) via
     *      least-squares fit of Id vs time over the first 20 samples.
     *   4. Phase 4 (cal_step 202-400): Let current decay (PI regulates).
     *   5. Phase 5 (cal_step > 400):   Transition to CURRENT_OFFSET.
     *
     * Expected scope trace: 3.6 V step on Vd, Id rises with inductive
     * slope Ld = 3.6 / (dId/dt_initial).  For SPMSM, Lq = Ld.
     */
    const float vd_test  = 3.6f;     /* Vd step: ~15% of 24V VBUS       */
    const float dt       = 1.0f / 20000.0f;  /* 50 µs sample interval   */
    const int   N_FIT    = 20;        /* samples for linear regression   */

    *i_d_ref = 0.0f;
    *i_q_ref = 0.0f;
    sc->cal_step++;

    /* Phase 1: Settle with zero current (100 cycles) */
    if (sc->cal_step <= 100) {
        return;
    }

    /* Phase 2: Apply Vd step, sample Id rise */
    if (sc->cal_step <= 200) {
        FOC_SetDirectVd(vd_test, 0.0f);

        const int idx = sc->cal_step - 101;  /* 0-based within Vd window */
        if (idx >= 0 && idx < N_FIT) {
            sc->cal_ld_sum_id  += sc->status.d_A;
            sc->cal_ld_sum_iid += (float)idx * sc->status.d_A;
        }
        return;
    }

    /* Phase 3: Compute Ld from slope */
    if (sc->cal_step == 201) {
        /* Least-squares linear regression: Id = Id0 + slope * t
         *   t_i  = idx * dt_fit,  idx = 0..N_FIT-1
         *   slope = N*sum(idx*Id) - sum(idx)*sum(Id)  /  N*sum(idx^2) - sum(idx)^2 * 1/dt
         *
         * Precomputed constants for N_FIT = 20:
         *   sum(idx)   = 190
         *   sum(idx^2) = 2470
         *   denom      = 20*2470 - 190^2 = 13300
         */
        const int    n        = N_FIT;
        const float  sum_i    = (float)(n * (n - 1) / 2);
        const float  sum_i2   = (float)((n - 1) * n * (2 * n - 1) / 6);
        const float  denom    = (float)n * sum_i2 - sum_i * sum_i;

        const float  numerator = (float)n * sc->cal_ld_sum_iid
                                 - sum_i * sc->cal_ld_sum_id;
        float slope = (denom > 1e-12f)
                      ? (numerator / denom / dt)
                      : 0.0f;

        float Ld = (fabsf(slope) > 1e-6f) ? (vd_test / slope) : 0.0f;

        /* Sanity check: Ld must be within plausible range */
        if (Ld > 1e-6f && Ld < 1e-2f) {
            sc->config.motor.inductance_d_H = Ld;
            sc->config.motor.inductance_q_H = Ld;  /* SPMSM: Ld = Lq */
        }

        /* Reset accumulators for next use */
        sc->cal_ld_sum_id  = 0.0f;
        sc->cal_ld_sum_iid = 0.0f;
        return;
    }

    /* Phase 4: Freewheel — PI regulates current to zero naturally */
    if (sc->cal_step <= 400) {
        return;
    }

    /* Phase 5: Transition to current offset measurement */
    sc->cal_substate = CAL_SUB_CURRENT_OFFSET;
    sc->cal_step     = 0;
}

static void do_calibrating_complete(ServoControl* sc,
                                    float* i_d_ref, float* i_q_ref)
{
    *i_d_ref = 0.0f;
    *i_q_ref = 0.0f;
    sc->cal_step++;

    if (sc->cal_step > 100) {  /* brief hold */
        sc->status.mode = SERVO_MODE_CALIB_COMPLETE;
        sc->cal_substate = CAL_SUB_NONE;
        sc->cal_step     = 0;
    }
}

/** kCalibrating: dispatch to calibration sub-state. */
static void do_calibrating(ServoControl* sc,
                           float measured_position, float measured_velocity,
                           float theta_elec,
                           float* i_d_ref, float* i_q_ref)
{
    switch (sc->cal_substate) {
        case CAL_SUB_RS_MEASURE:
            do_calibrating_rs(sc, i_d_ref, i_q_ref);
            break;
        case CAL_SUB_THETA_OFFSET:
            do_calibrating_theta(sc, measured_position, theta_elec,
                                 i_d_ref, i_q_ref);
            break;
        case CAL_SUB_LD_LQ:
            do_calibrating_ldlq(sc, i_d_ref, i_q_ref);
            break;
        case CAL_SUB_CURRENT_OFFSET:
            do_calibrating_offset(sc, i_d_ref, i_q_ref);
            break;
        case CAL_SUB_COMPLETE:
            do_calibrating_complete(sc, i_d_ref, i_q_ref);
            break;
        default:
            *i_d_ref = 0.0f;
            *i_q_ref = 0.0f;
            break;
    }
}

/** kEnabling: startup sequence (open-loop spin-up to verify encoder). */
static void do_enabling(ServoControl* sc,
                        float measured_position, float measured_velocity,
                        float theta_elec,
                        float* i_d_ref, float* i_q_ref)
{
    /* Simple startup: apply increasing open-loop voltage to
     * spin rotor and verify encoder feedback. */
    sc->cal_step++;
    const float startup_current = 1.0f;  /* modest current */

    /* Use open-loop angle ramp: theta_ol = cal_step * period_s * startup_speed */
    const float startup_speed = 2.0f;  /* [rev/s] */
    const float theta_ol = (float)sc->cal_step * sc->period_s * startup_speed
                           * K_2PI;  /* convert rev → rad */

    /* Inject current at open-loop angle (Id alignment) */
    (void)theta_elec;  /* we use open-loop angle, not measured */
    (void)measured_position;

    /* Simple approach: just apply direct Iq in open-loop */
    if (sc->cal_step < 500) {
        /* First cycles: align (Id only) */
        *i_d_ref = startup_current;
        *i_q_ref = 0.0f;
    } else if (sc->cal_step < 2000) {
        /* Ramp up Iq slowly */
        const float t = (float)(sc->cal_step - 500) / 1500.0f;
        *i_d_ref = startup_current * (1.0f - t);
        *i_q_ref = startup_current * t;
    } else {
        /* Transition to normal mode after startup */
        if (sc->cmd.mode != SERVO_MODE_ENABLING) {
            sc->status.mode = sc->cmd.mode;
        } else {
            sc->status.mode = SERVO_MODE_CURRENT;
        }
        sc->cal_step = 0;
        *i_d_ref = 0.0f;
        *i_q_ref = 0.0f;
    }
}

/** kZeroVelocity: derivative-only position hold. */
static void do_zero_velocity(ServoControl* sc,
                              float measured_position, float measured_velocity,
                              float* i_d_ref, float* i_q_ref)
{
    // Zero-velocity mode: use position PID with kp_scale=0 (derivative only)
    ServoCommandData tmp;
    ServoCommandData_Init(&tmp);
    tmp.mode = SERVO_MODE_POSITION;
    tmp.velocity = 0.0f;
    tmp.kp_scale = 0.0f;
    tmp.kd_scale = 1.0f;
    tmp.ilimit_scale = 0.0f;
    tmp.max_torque_Nm = 5.0f;  // low torque hold

    do_position(sc, measured_position, measured_velocity, &tmp, i_d_ref, i_q_ref);
}

/* ===================================================================
 * Public API
 * =================================================================== */

void ServoControl_Init(ServoControl* sc, float period_s,
                       uint8_t poles, float resistance_ohm,
                       float torque_constant)
{
    memset(sc, 0, sizeof(*sc));

    // Config defaults
    ServoConfig_Init(&sc->config);
    sc->config.motor.poles = poles;
    sc->config.motor.resistance_ohm = resistance_ohm;
    sc->config.motor.torque_constant = torque_constant;

    // Period
    sc->period_s = period_s;

    // Back-EMF constant from torque constant
    // Kt = 3/2 * P * psi_m → psi_m = 2*Kt/(3*P) → v_per_hz = psi_m * P = 2*Kt/3
    // For a three-phase motor: v_per_hz = 2.0f/3.0f * torque_constant * poles;
    sc->v_per_hz = torque_constant * K_2PI;  // approximate

    // Pre-computed reciprocals
    sc->inv_motor_resistance = (resistance_ohm > 1e-12f) ?
                               (1.0f / resistance_ohm) : 0.0f;

    // PID configs — copy from config
    sc->pid_pos_cfg = sc->config.pid_position;
    sc->pid_vel_cfg = sc->config.pid_velocity;

    // Current gains
    compute_current_gains(sc);

    // Reset all PID states
    PID_Reset(&sc->pid_pos_state);
    PID_Reset(&sc->pid_vel_state);
    SimplePI_Reset(&sc->pid_d_state);
    SimplePI_Reset(&sc->pid_q_state);

    // Filters
    ExponentialFilter_Init(&sc->vel_filter, 1.0f / period_s, 50.0f);
    ExponentialFilter_Init(&sc->bus_v_filter, 1.0f / period_s, 2.0f);

    // State
    sc->status.mode = SERVO_MODE_STOPPED;
    sc->status.fault = SERVO_FAULT_NONE;
    sc->status.limit_flags = 0;
    sc->cooldown_count = 0;
    sc->stall_count = 0;
    sc->initialized = true;
}

void ServoControl_SetCommand(ServoControl* sc, const ServoCommandData* cmd) {
    sc->cmd = *cmd;
}

void ServoControl_Stop(ServoControl* sc) {
    sc->cmd.mode = SERVO_MODE_STOPPED;
    sc->cmd.stop = true;
    sc->status.mode = SERVO_MODE_STOPPED;
    PID_Reset(&sc->pid_pos_state);
    PID_Reset(&sc->pid_vel_state);
    SimplePI_Reset(&sc->pid_d_state);
    SimplePI_Reset(&sc->pid_q_state);
}

void ServoControl_SetFault(ServoControl* sc, ServoFault fault) {
    sc->status.fault = fault;
    sc->status.mode = SERVO_MODE_FAULT;
}

void ServoControl_ClearFault(ServoControl* sc) {
    sc->status.fault = SERVO_FAULT_NONE;
    if (sc->status.mode == SERVO_MODE_FAULT) {
        sc->status.mode = SERVO_MODE_STOPPED;
    }
}

void ServoControl_UpdateCurrentGains(ServoControl* sc) {
    compute_current_gains(sc);
}

void ServoControl_StartCalibration(ServoControl* sc, float theta_elec) {
    if (sc->status.mode == SERVO_MODE_FAULT) return;

    sc->status.mode     = SERVO_MODE_CALIBRATING;
    sc->cal_substate    = CAL_SUB_RS_MEASURE;
    sc->cal_step        = 0;
    sc->cal_theta_elec  = theta_elec;
    sc->theta_offset    = 0.0f;
    sc->cal_ld_sum_id   = 0.0f;
    sc->cal_ld_sum_iid  = 0.0f;

    /* Reset PIDs for clean start */
    PID_Reset(&sc->pid_pos_state);
    PID_Reset(&sc->pid_vel_state);
    SimplePI_Reset(&sc->pid_d_state);
    SimplePI_Reset(&sc->pid_q_state);
}

/* ===================================================================
 * Main control ISR
 * =================================================================== */

void ServoControl_Run(ServoControl* sc,
                      float position, float velocity,
                      float theta_elec, float vbus,
                      float d_A, float q_A,
                      float* i_d_ref, float* i_q_ref)
{
    // Default outputs
    *i_d_ref = 0.0f;
    *i_q_ref = 0.0f;

    if (!sc->initialized) return;

    // --- Update measured values ---
    sc->status.position     = position;
    sc->status.velocity     = velocity;
    sc->status.d_A          = d_A;
    sc->status.q_A          = q_A;
    sc->status.bus_V        = vbus;
    sc->status.i_d_ref      = 0.0f;
    sc->status.i_q_ref      = 0.0f;

    // Filter bus voltage
    ExponentialFilter_Apply(&sc->bus_v_filter, vbus, &sc->status.bus_V);

    // --- Handle mode transitions ---
    if (sc->cmd.mode != sc->status.mode) {
        maybe_change_mode(sc, sc->cmd.mode);
    }

    // --- Persistent fault checks ---
    if (sc->status.mode != SERVO_MODE_STOPPED &&
        sc->status.mode != SERVO_MODE_FAULT) {
        if (vbus > sc->config.max_voltage) {
            sc->status.fault = SERVO_FAULT_OVERVOLTAGE;
            sc->status.mode = SERVO_MODE_FAULT;
        }
        if (vbus < 4.0f) {
            sc->status.fault = SERVO_FAULT_UNDERVOLTAGE;
            sc->status.mode = SERVO_MODE_FAULT;
        }

        // --- Overcurrent protection ---
        if (sqrtf(d_A * d_A + q_A * q_A) > sc->config.current_limit_A * OVERCURRENT_MARGIN) {
            sc->status.fault = SERVO_FAULT_OVERCURRENT;
            sc->status.mode = SERVO_MODE_FAULT;
        }

        // --- Stall detection ---
        if (sc->status.mode != SERVO_MODE_CALIBRATING) {
            if (fabsf(velocity) < sc->config.stall_velocity_threshold &&
                fabsf(q_A) > sc->config.stall_current_threshold) {
                sc->stall_count++;
                if (sc->stall_count > sc->config.stall_timeout_cycles) {
                    sc->status.fault = SERVO_FAULT_VELOCITY_ERROR;
                    sc->status.mode = SERVO_MODE_FAULT;
                }
            } else {
                sc->stall_count = 0;
            }
        }

        // --- Overtemperature protection ---
        if (sc->status.temperature > 0.0f &&
            sc->status.temperature > sc->config.fault_temperature) {
            sc->status.fault = SERVO_FAULT_OVERTEMPERATURE;
            sc->status.mode = SERVO_MODE_FAULT;
        }
    }

    // --- Timeout check ---
    if ((sc->status.mode == SERVO_MODE_POSITION) &&
        sc->cmd.timeout_s > 0.0f) {
        // timeout tracking simplified — would need a timer in production
    }

    // --- Clear stale PIDs ---
    clear_stale_pids(sc);

    // --- Clear fault after any non-fault command ---
    if (sc->status.fault != SERVO_FAULT_NONE &&
        sc->cmd.mode != SERVO_MODE_FAULT &&
        sc->cmd.mode != sc->status.mode) {
        // Fault will clear through mode transition
    }

    // --- Set cooldown ---
    if (sc->status.mode != SERVO_MODE_STOPPED &&
        sc->status.mode != SERVO_MODE_FAULT) {
        sc->cooldown_count = sc->config.cooldown_cycles;
    }

    // --- Mode dispatch ---
    switch (sc->status.mode) {
        case SERVO_MODE_STOPPED:
        case SERVO_MODE_FAULT:
            do_stopped(sc, i_d_ref, i_q_ref);
            break;

        case SERVO_MODE_CURRENT:
            do_current(sc, sc->cmd.i_d_A, sc->cmd.i_q_A, i_d_ref, i_q_ref);
            break;

        case SERVO_MODE_POSITION:
            do_position(sc, position, velocity, &sc->cmd, i_d_ref, i_q_ref);
            break;

        case SERVO_MODE_ZERO_VELOCITY:
            do_zero_velocity(sc, position, velocity, i_d_ref, i_q_ref);
            break;

        case SERVO_MODE_BRAKE:
            do_brake(sc, i_d_ref, i_q_ref);
            break;

        case SERVO_MODE_ENABLING:
            do_enabling(sc, position, velocity, theta_elec,
                        i_d_ref, i_q_ref);
            break;

        case SERVO_MODE_CALIBRATING:
            do_calibrating(sc, position, velocity, theta_elec,
                           i_d_ref, i_q_ref);
            break;

        case SERVO_MODE_CALIB_COMPLETE:
            // Calibration done — zero output until commanded to active mode
            *i_d_ref = 0.0f;
            *i_q_ref = 0.0f;
            break;

        default:
            // Unknown mode — safe stop
            do_stopped(sc, i_d_ref, i_q_ref);
            break;
    }

    // --- Store references for telemetry ---
    sc->status.i_d_ref = *i_d_ref;
    sc->status.i_q_ref = *i_q_ref;

    // --- Store PID diagnostics ---
    sc->status.pos_p = sc->pid_pos_state.p;
    sc->status.pos_d = sc->pid_pos_state.d;
    sc->status.pos_i = sc->pid_pos_state.integral;
    sc->status.vel_p = sc->pid_vel_state.p;
    sc->status.vel_i = sc->pid_vel_state.integral;
}
