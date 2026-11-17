/**
 * @file servo_control.h
 * @brief Servo Control State Machine — ported from moteus BldcServoControl
 *
 * Implements the control cascade for BLDC servo:
 *   Position mode:   PID(position_error) → velocity → PID(vel_error) → iq
 *   Velocity mode:   PID(vel_error) → iq
 *   Current mode:    Direct iq setpoint
 *   Torque mode:     iq = torque / torque_constant
 *
 * Control loop architecture (runs at PWM rate, e.g. 20 kHz):
 *   1. Sample phase currents + encoder position
 *   2. Clarke+Park transforms (foc.c)
 *   3. ServoControl_Run → i_d_ref, i_q_ref
 *   4. Current PI control (foc.c)
 *   5. Inverse Park + SVM + PWM write (foc.c)
 *
 * Reference: moteus fw/bldc_servo_control.h, fw/pid.h (Apache 2.0)
 */

#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * Calibration sub-state machine (used during SERVO_MODE_CALIBRATING)
 * =================================================================== */
typedef enum {
    CAL_SUB_NONE             = 0,
    CAL_SUB_RS_MEASURE       = 1,  /**< measure phase resistance        */
    CAL_SUB_THETA_OFFSET     = 2,  /**< align rotor, capture θ offset   */
    CAL_SUB_CURRENT_OFFSET   = 3,  /**< measure ADC current offset      */
    CAL_SUB_COMPLETE         = 4,  /**< calibration done, hold          */
    CAL_SUB_LD_LQ            = 5,  /**< measure d/q inductance (Ld/Lq)  */
} CalibrationSubState;

/* ===================================================================
 * Mode enum — compatible with moteus BldcServoMode values
 * =================================================================== */
typedef enum {
    SERVO_MODE_STOPPED           = 0,
    SERVO_MODE_FAULT             = 1,
    SERVO_MODE_ENABLING          = 2,
    SERVO_MODE_CALIBRATING       = 3,
    SERVO_MODE_CALIB_COMPLETE    = 4,
    SERVO_MODE_PWM               = 5,
    SERVO_MODE_VOLTAGE           = 6,
    SERVO_MODE_VOLTAGE_FOC       = 7,
    SERVO_MODE_VOLTAGE_DQ        = 8,
    SERVO_MODE_CURRENT           = 9,
    SERVO_MODE_POSITION          = 10,
    SERVO_MODE_POSITION_TIMEOUT  = 11,
    SERVO_MODE_ZERO_VELOCITY     = 12,
    SERVO_MODE_STAY_WITHIN_BOUNDS = 13,
    SERVO_MODE_BRAKE             = 15,
    SERVO_MODE_NUM_MODES,
} ServoMode;

/* ===================================================================
 * Fault codes (simplified subset)
 * =================================================================== */
typedef enum {
    SERVO_FAULT_NONE              = 0,
    SERVO_FAULT_OVERCURRENT       = 1,
    SERVO_FAULT_OVERVOLTAGE       = 2,
    SERVO_FAULT_UNDERVOLTAGE      = 3,
    SERVO_FAULT_OVERTEMPERATURE   = 4,
    SERVO_FAULT_ENCODER           = 5,
    SERVO_FAULT_MOTOR_DRIVER      = 6,
    SERVO_FAULT_POSITION_ERROR    = 7,
    SERVO_FAULT_VELOCITY_ERROR    = 8,
    SERVO_FAULT_THETA_INVALID     = 9,
    SERVO_FAULT_MOTOR_NOT_CFG     = 10,
    SERVO_FAULT_START_OUTSIDE_LIMIT = 11,
    SERVO_FAULT_INVALID_LIMITS    = 12,
    SERVO_FAULT_CONFIG_CHANGED    = 13,
    SERVO_FAULT_GENERAL           = 255,
} ServoFault;

/* ===================================================================
 * Limit flags (telemetry)
 * =================================================================== */
typedef enum {
    SERVO_LIMIT_NONE              = 0,
    SERVO_LIMIT_MAX_CURRENT       = (1U << 0),
    SERVO_LIMIT_MAX_VELOCITY      = (1U << 1),
    SERVO_LIMIT_MAX_POWER         = (1U << 2),
    SERVO_LIMIT_MAX_TORQUE        = (1U << 3),
    SERVO_LIMIT_POSITION_BOUNDS   = (1U << 4),
    SERVO_LIMIT_FLUX_BRAKING      = (1U << 5),
    SERVO_LIMIT_FET_TEMP          = (1U << 6),
    SERVO_LIMIT_MOTOR_TEMP        = (1U << 7),
} ServoLimitFlags;

/* ===================================================================
 * Motor parameters (simplified BldcServoMotor)
 * =================================================================== */
typedef struct {
    float resistance_ohm;     /**< phase-to-center resistance [Ω]    */
    float inductance_d_H;     /**< d-axis inductance [H]             */
    float inductance_q_H;     /**< q-axis inductance [H]             */
    float torque_constant;    /**< Kt = 3/2 * poles/2 * flux_linkage [Nm/A] */
    float v_per_hz;           /**< back-EMF constant [V/(rad/s)]     */
    uint8_t poles;            /**< number of motor poles             */
} ServoMotor;

/* ===================================================================
 * Command data (from external source — SPI, CAN, etc.)
 * =================================================================== */
typedef struct {
    ServoMode mode;           /**< requested operating mode          */

    /* Position/velocity mode */
    float position;           /**< target position [rev]             */
    float velocity;           /**< target velocity [rev/s]           */

    /* Current/torque mode */
    float i_d_A;              /**< d-axis current reference [A]      */
    float i_q_A;              /**< q-axis current reference [A]      */
    float torque_Nm;          /**< torque setpoint [Nm]              */

    /* Feedforward + scaling */
    float feedforward_Nm;     /**< torque feedforward [Nm]           */
    float kp_scale;           /**< position PID kp multiplier        */
    float kd_scale;           /**< position PID kd multiplier        */
    float ilimit_scale;       /**< position PID integrator limit multiplier */

    /* Limits */
    float velocity_limit;     /**< velocity limit [rev/s], NaN=unlimited */
    float accel_limit;        /**< acceleration limit [rev/s²], NaN=unlimited */
    float max_torque_Nm;      /**< maximum torque [Nm]               */

    /* Timeout */
    float timeout_s;          /**< position mode timeout [s], 0=none */

    /* Flags */
    bool stop;                /**< request immediate stop             */
} ServoCommandData;

/** @brief Set ServoCommandData to safe defaults. */
static inline void ServoCommandData_Init(ServoCommandData* cmd) {
    cmd->mode          = SERVO_MODE_STOPPED;
    cmd->position      = 0.0f;
    cmd->velocity      = 0.0f;
    cmd->i_d_A         = 0.0f;
    cmd->i_q_A         = 0.0f;
    cmd->torque_Nm     = 0.0f;
    cmd->feedforward_Nm = 0.0f;
    cmd->kp_scale      = 1.0f;
    cmd->kd_scale      = 1.0f;
    cmd->ilimit_scale  = 1.0f;
    cmd->velocity_limit = NAN;
    cmd->accel_limit   = NAN;
    cmd->max_torque_Nm = 100.0f;
    cmd->timeout_s     = 0.0f;
    cmd->stop          = false;
}

/* ===================================================================
 * Servo status (telemetry)
 * =================================================================== */
typedef struct {
    ServoMode  mode;           /**< current operating mode           */
    ServoFault fault;          /**< fault code                       */
    uint8_t    limit_flags;    /**< active limit bitfield            */

    /* Position / velocity */
    float position;            /**< measured position [rev]          */
    float velocity;            /**< measured velocity [rev/s]        */

    /* Current (measured from ADC) */
    float d_A;                 /**< measured d-axis current [A]        */
    float q_A;                 /**< measured q-axis current [A]        */

    /* Current references (after cascade) */
    float i_d_ref;             /**< d-axis current reference [A]      */
    float i_q_ref;             /**< q-axis current reference [A]      */

    /* Control variables */
    float control_position;    /**< filtered/setpoint position [rev] */
    float control_velocity;    /**< filtered/setpoint velocity [rev/s] */
    float control_accel;       /**< control acceleration [rev/s²]    */
    bool  trajectory_done;     /**< trajectory reached target        */

    /* Torque */
    float torque_Nm;           /**< estimated torque [Nm]            */
    float bus_V;               /**< bus voltage [V]                  */
    float temperature;         /**< measured temperature [°C]         */
    float power_W;             /**< electrical power [W]             */

    /* PID diagnostic fields */
    float pos_p, pos_d, pos_i; /**< position PID terms               */
    float vel_p, vel_i;        /**< velocity PI terms                */
    float cur_d_p, cur_d_i;    /**< d-current PI terms               */
    float cur_q_p, cur_q_i;    /**< q-current PI terms               */
} ServoStatus;

/* ===================================================================
 * Servo configuration (gains, limits, motor params)
 * =================================================================== */
typedef struct {
    /* Motor parameters */
    ServoMotor motor;

    /* Current control (SimplePI, d/q axis) */
    float pid_dq_hz;              /**< current loop bandwidth [Hz]       */
    float pid_current_kp;         /**< current PI proportional gain      */
    float pid_current_ki;         /**< current PI integral gain          */
    float current_limit_A;        /**< hard current limit [A]            */
    float current_ilimit_A;       /**< current integrator clamp [A]      */
    float max_current_desired_rate; /**< current setpoint slew [A/s]     */

    /* Velocity control (PID) */
    PID_Config pid_velocity;

    /* Position control (PID) */
    PID_Config pid_position;

    /* Feedforward */
    float current_feedforward;    /**< I·R feedforward scale (0=off, 1=full) */
    float bemf_feedforward;       /**< BEMF feedforward scale (0=off)       */
    float inertia_feedforward;    /**< accel → torque feedforward [kg·m²]   */

    /* Limits */
    float max_velocity;           /**< velocity limit [rev/s]              */
    float max_velocity_derate;    /**< derate range beyond max_velocity    */
    float max_power_W;            /**< power limit [W], NaN=unlimited     */
    float max_voltage;            /**< bus overvoltage trip [V]            */

    /* Fault thresholds */
    float fault_temperature;      /**< FET fault temperature [°C]          */
    float fault_position_error;   /**< position error fault threshold [rev]*/
    float fault_velocity_error;   /**< velocity error fault threshold [rev/s] */

    /* Stall detection */
    float    stall_velocity_threshold;   /**< velocity below which stall is detected [rev/s] */
    float    stall_current_threshold;    /**< q-current above which stall is detected [A] */
    uint16_t stall_timeout_cycles;       /**< stall count threshold before fault */

    /* Timeout */
    float default_timeout_s;      /**< position mode timeout default [s]   */
    uint8_t timeout_mode;         /**< 0=stopped, 10=position, 12=zero_vel, 15=brake */

    /* Cooldown */
    uint16_t cooldown_cycles;     /**< cycles at full current before brake off */
    uint16_t cooldown_brake_cycles; /**< brake cycles before HiZ           */
} ServoConfig;

/** @brief Set ServoConfig to safe defaults for a typical motor. */
void ServoConfig_Init(ServoConfig* cfg);

/* ===================================================================
 * Main servo control state
 * =================================================================== */
typedef struct {
    /* Configuration (read-only during control) */
    ServoConfig config;

    /* Command (latest from external source) */
    ServoCommandData cmd;

    /* Status (telemetry output) */
    ServoStatus status;

    /* --- PID states --- */
    // Current control: SimplePI (matches moteus fw/simple_pi.h, sign=-1)
    SimplePI_Config pid_d_cfg;
    SimplePI_State  pid_d_state;
    SimplePI_Config pid_q_cfg;
    SimplePI_State  pid_q_state;

    // Velocity control: PID
    PID_Config pid_vel_cfg;
    PID_State  pid_vel_state;

    // Position control: PID
    PID_Config pid_pos_cfg;
    PID_State  pid_pos_state;

    /* --- Filters --- */
    ExponentialFilter vel_filter;   /**< velocity low-pass filter         */
    ExponentialFilter bus_v_filter; /**< bus voltage low-pass filter      */

    /* --- Internal state --- */
    float period_s;               /**< control loop period [s]            */
    uint16_t cooldown_count;      /**< cycles remaining in cooldown       */
    uint16_t stall_count;         /**< consecutive stall detection cycles  */
    bool   initialized;           /**< true after ServoControl_Init       */

    /* --- Calibration state ----------------------------------------------- */
    CalibrationSubState cal_substate; /**< current calibration substep      */
    uint16_t cal_step;                /**< step counter within substep       */
    float    cal_theta_elec;          /**< electrical angle during alignment */
    float    theta_offset;            /**< calibrated rotor θ offset [rad]  */
    float    cal_ld_sum_id;           /**< Ld fit: running sum of Id [A]    */
    float    cal_ld_sum_iid;          /**< Ld fit: running sum of i*Id      */

    /* Pre-computed constants */
    float inv_motor_resistance;   /**< 1 / R                              */
    float v_per_hz;               /**< back-EMF constant for feedforward  */
} ServoControl;

/* ===================================================================
 * Public API
 * =================================================================== */

/** @brief Initialise servo control with config defaults.
 *  @param sc        ServoControl state (zeroed before call)
 *  @param period_s  Control loop period [s] (e.g. 1/20000 = 50 µs)
 *  @param poles     Number of motor pole pairs
 *  @param resistance_ohm  Phase-to-center resistance [Ω]
 *  @param torque_constant  Motor torque constant [Nm/A]
 */
void ServoControl_Init(ServoControl* sc, float period_s,
                       uint8_t poles, float resistance_ohm,
                       float torque_constant);

/** @brief Set command from external source (SPI, CAN, etc.).
 *  Must not be called from ISR context (or protect with critical section).
 */
void ServoControl_SetCommand(ServoControl* sc, const ServoCommandData* cmd);

/** @brief Execute one iteration of the servo control cascade.
 *
 *  Called at the PWM/control rate (e.g. 20 kHz inside the FOC ISR).
 *  Dispatches based on current mode (position/velocity/current/torque).
 *
 *  @param sc          ServoControl state
 *  @param position    Measured rotor position [revolutions]
 *  @param velocity    Measured rotor velocity [rev/s]
 *  @param theta_elec  Electrical angle [rad]
 *  @param vbus        Bus voltage [V]
 *  @param d_A         Measured d-axis current [A]
 *  @param q_A         Measured q-axis current [A]
 *  @param[out] i_d_ref  D-axis current reference output [A]
 *  @param[out] i_q_ref  Q-axis current reference output [A]
 */
void ServoControl_Run(ServoControl* sc,
                      float position, float velocity,
                      float theta_elec, float vbus,
                      float d_A, float q_A,
                      float* i_d_ref, float* i_q_ref);

/** @brief Request immediate stop (mode → kStopped).
 *  Safe to call from any context.
 */
void ServoControl_Stop(ServoControl* sc);

/** @brief Set a fault condition. */
void ServoControl_SetFault(ServoControl* sc, ServoFault fault);

/** @brief Update current PI gains from pid_dq_hz + motor params.
 *  Call after changing pid_dq_hz or motor parameters.
 */
void ServoControl_UpdateCurrentGains(ServoControl* sc);

/** @brief Clear latched fault (if fault condition is resolved). */
void ServoControl_ClearFault(ServoControl* sc);

/** @brief Begin calibration sequence.
 *  Transitions mode to SERVO_MODE_CALIBRATING and resets substep.
 *  @param sc         ServoControl state
 *  @param theta_elec Current electrical angle [rad] (for alignment)
 */
void ServoControl_StartCalibration(ServoControl* sc, float theta_elec);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONTROL_H_ */
