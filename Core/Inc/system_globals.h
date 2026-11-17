#ifndef SYSTEM_GLOBALS_H
#define SYSTEM_GLOBALS_H

#include <stdint.h>
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "motor.h"
#include "nrf24.h"
#include "vl6180x.h"

#include "ekf.h"

// ===================== DEFINES DEL SISTEMA =====================
#define ROBOT_RADIO           0.08215f
#define ROBOT_MAX_LINEAR_ACC  3.0f
extern float robot_max_linear_acc;   // runtime linear acceleration limit for vx/vy ramp
#define ROBOT_MAX_LINEAR_VEL 2.5f // En m/s
#define ROBOT_MAX_ANGULAR_ACC 50.0f // rad/s^2, rate limit para vth (rampa interna)
#define ANGULAR_SPEED_FACTOR  30.0f
#define DRIBBLER_CONV(x)      ((x) * (1023.0f / 7.0f))
#define VL6180X_THRESHOLD     65 // en mm
#define VL6180X_SAMPLE_TIME   50
#define nRF24L01_SYSMIC_CHANNEL 0x6B
#define TELEMETRY_CHANNEL       0x6C
#define RADIO_TIMEOUT_MS        500
#define TELEMETRY_PERIOD_MS     10

// Estados del kicker
#define KICKER_DISCHARGED 0x00
#define KICKER_CHARGED    0x01
#define KICKER_START      0x02

// ===================== VARIABLES GLOBALES =====================
extern uint16_t robot_id;

// --- RADIO TASK ---
extern uint8_t txBuffer[32];
extern uint8_t rxBuffer[32];
extern uint8_t rx_len;
extern uint8_t nrf_status;
extern uint8_t nrf_config;
extern nRF24_Handler_t nrf_device;
extern uint32_t radio_last_rx_tick;
extern uint8_t radio_ever_received;
extern uint8_t tx_node_addr[5];
extern uint8_t rx_node_addr[5];
extern uint8_t correction_node_addr[5];
extern osMessageQueueId_t nrf24CheckHandle;
extern osMutexId_t radioMutexHandle;

// --- TELEMETRY TASK ---
extern osThreadId_t telemetryTaskHandle;

// --- ACCEL TASK ---
extern int16_t accel_x_raw;
extern int16_t accel_y_raw;
extern int16_t accel_z_raw;

extern float accel_x_g;
extern float accel_y_g;
extern float accel_z_g;



// --- GYRO TASK ---
extern int16_t gyro_x_raw;
extern int16_t gyro_y_raw;
extern int16_t gyro_z_raw;
extern float gyro_x_dps;
extern float gyro_y_dps;
extern float gyro_z_dps;

// --- DRIVE TASK ---
extern float speed[4];
extern float kinematic[4][3];

extern Motor_Handler_t motor[4];
extern uint8_t direction[4];
extern float dribbler_speed;
extern uint8_t dribbler_sel;
extern const uint16_t Dribbler_SpeedSet[];
extern float v_vel[3];
extern float robot_max_angular_acc;  // runtime angular acceleration limit for vth ramp

// --- ODOMETRY / ROBOT CONTROL ---
extern float vx_enc, vy_enc, vth_enc;
extern float vx_est;
extern float vy_est;
extern float vth_est;
extern float theta_est;    // Kalman-filtered theta estimate
extern float x_target;     // position waypoint target (m, robot frame)
extern float y_target;     // position waypoint target (m, robot frame)
extern float theta_target; // position waypoint heading target (rad)
extern uint8_t position_control_active; // 1 = robot closes position loop toward waypoint
extern float offset_x;      // camera-to-local frame alignment offset (m)
extern float offset_y;      // camera-to-local frame alignment offset (m)
extern float offset_theta;  // camera-to-local frame alignment offset (rad)
extern uint8_t camera_correction_enabled; // 1 = apply camera correction to local EKF
extern PID_Handler_t pid_vx;
extern PID_Handler_t pid_vy;
extern PID_Handler_t pid_vth;
extern uint8_t robot_pid_enable;

// Per-axis PID passthrough gates (open-loop characterization): 1 = closed-loop PID,
// 0 = raw command passthrough for that axis only.
extern uint8_t pid_vx_enable;
extern uint8_t pid_vy_enable;
extern uint8_t pid_vth_enable;

// --- KICK TASK ---
extern uint8_t kick_sel;
extern uint8_t kick_flag;
extern uint16_t kick_delay;
extern uint16_t kick_count;
extern osMutexId_t kickFlagHandle;
extern osMessageQueueId_t kickQueueHandle;

// --- BALL DETECTOR TASK ---
extern VL6180X_Handler_t range_sensor;
extern uint8_t ball_possession;
extern uint16_t ball_range;
extern uint16_t ball_accum;
extern uint8_t ball_meas_set[10];

// ===================== HANDLERS DE PERIFÉRICOS =====================
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_tx;


// ===================== RTOS TASK HANDLES =====================
extern osThreadId_t driveTaskHandle;
extern osThreadId_t accelTaskHandle;
extern osThreadId_t radioTaskHandle;
extern osThreadId_t kickTaskHandle;
extern osThreadId_t telemetryTaskHandle;



// ===================== SENSOR FUSION (KALMAN) =====================
extern float ax_ms2;
extern float ay_ms2;
extern float gyro_z_rads;
extern EKF_t ekf;
extern uint8_t kalman_ready;

// ===================== PHASE A REFACTOR =====================
extern TIM_HandleTypeDef htim7;

// ===================== PER-MOTOR PID PARAMS =====================
extern PID_Params_t wheel_pid_params[4];

// ===================== OPEN-LOOP TEST MODE =====================
extern uint8_t test_enable;
extern uint8_t test_motor;
extern float   test_voltage;

// ===================== CALIBRATION TELEMETRY =====================
extern uint8_t telemetry_calib_mode;  // 1 = calibration format (motor speeds) in packTelemetryBuffer


#endif // SYSTEM_GLOBALS_H
