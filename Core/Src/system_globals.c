#include "system_globals.h"

// ===================== RADIO TASK =====================
uint16_t robot_id = 0;
uint8_t txBuffer[32] = {'R','a','d','i','o',' ','O','N','\n'};
uint8_t rxBuffer[32] = {0};
uint8_t rx_len = 0;
uint8_t nrf_status = 0;
uint8_t nrf_config = 0;
nRF24_Handler_t nrf_device = {0};
uint8_t tx_node_addr[5] = {'s', 'y', 's', 't', 'x'};
uint8_t rx_node_addr[5] = {'s', 'y', 's', 'r', 'x'};
osMessageQId nrf24CheckHandle = NULL;

// ===================== DRIVE TASK =====================
uint8_t direction[4] = {0};
float speed[4] = {0};
float kinematic[4][3] = {0};
float a = 0.0f; //-90 y 90  1.08
float b = 0.0f;//-28.0f; //-45 y 45 1.16
Motor_Handler_t motor[4] = {0};
float v_vel[3] = {0};

// ===================== KICK TASK =====================
float dribbler_speed = 0.0f;
uint8_t dribbler_sel = 0;
uint8_t kick_sel = 0;
uint8_t kick_flag = 0;
uint16_t kick_delay = 0;
uint16_t kick_count = 0;
osMessageQId kickQueueHandle = NULL;
osMutexId kickFlagHandle = NULL;
osEvent kicker_side = {0};

// ===================== BALL DETECTOR TASK =====================
VL6180X_Handler_t range_sensor = {0};
uint8_t ball_posession = 0x00;
uint8_t ball_posession_last = 0x00;
uint16_t ball_range = 0;
uint16_t ball_accum = 0;
uint8_t ball_meas_set[10] = {0};
osThreadId ballDetectorTaskHandle = NULL;

// ===================== COMMON =====================

// ===================== RTOS TASK HANDLES =====================
osThreadId driveTaskHandle = NULL;
osThreadId radioTaskHandle = NULL;
osThreadId kickTaskHandle = NULL;

// ===================== HANDLERS DE PERIFÉRICOS =====================
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_tx;

// ===================== DRIBBLER SPEED SET =====================
const uint16_t Dribbler_SpeedSet[] = {0, 200, 450, 700, 900, 1023}; // Ajusta los valores según tu aplicación
