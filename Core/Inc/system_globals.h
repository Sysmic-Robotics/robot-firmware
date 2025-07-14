#include <stdint.h>
#ifndef SYSTEM_GLOBALS_H
#define SYSTEM_GLOBALS_H

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "motor.h"
#include "nrf24.h"
#include "vl6180x.h"

// ===================== DEFINES DEL SISTEMA =====================
#define ROBOT_RADIO           0.08215f
#define ROBOT_MAX_LINEAR_ACC  1.5f
#define ROBOT_MAX_LINEAR_VEL 2.5f // En m/s
#define ANGULAR_SPEED_FACTOR  30.0f
#define DRIBBLER_CONV(x)      ((x) * (1023.0f / 7.0f))
#define VL6180X_THRESHOLD     65 // en mm
#define VL6180X_SAMPLE_TIME   50
#define nRF24L01_SYSMIC_CHANNEL 0x6B

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
extern uint8_t tx_node_addr[5];
extern uint8_t rx_node_addr[5];
extern osMessageQId nrf24CheckHandle;

// --- DRIVE TASK ---
extern float speed[4];
extern float kinematic[4][3];
extern float a;
extern float b;
extern Motor_Handler_t motor[4];
extern uint8_t direction[4];
extern float dribbler_speed;
extern uint8_t dribbler_sel;
extern const uint16_t Dribbler_SpeedSet[];
extern float v_vel[3];

// --- KICK TASK ---
extern uint8_t kick_sel;
extern uint8_t kick_flag;
extern uint16_t kick_delay;
extern uint16_t kick_count;
extern osMutexId kickFlagHandle;
extern osMessageQId kickQueueHandle;

// --- BALL DETECTOR TASK ---
extern VL6180X_Handler_t range_sensor;
extern uint8_t ball_posession;
extern uint8_t radioTx_counter;
extern uint16_t ball_range;
extern uint16_t ball_accum;
extern uint8_t ball_meas_set[10];

// ===================== HANDLERS DE PERIFÉRICOS =====================
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_tx;


// ===================== RTOS TASK HANDLES =====================
extern osThreadId driveTaskHandle;
extern osThreadId radioTaskHandle;
extern osThreadId kickTaskHandle;
extern osThreadId ballDetectorTaskHandle;

#endif // SYSTEM_GLOBALS_H
