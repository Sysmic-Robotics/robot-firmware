/* main.c - Entry point for STM32 robot firmware (CMSIS-RTOS2) */
#include "main.h"
#include "cmsis_os.h"
#include "system_globals.h"
#include <math.h>
#include <stdint.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "nrf24.h"
#include "board.h"
#include "string.h"
#include "motor.h"
#include "vl6180x.h"
#include "drive_task.h"
#include "radio_task.h"
#include "kick_task.h"
#include "ball_detector_task.h"
#include "FreeRTOS.h"
#include "task.h"

// Prototipos de funciones de inicialización y tareas
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void MX_I2C3_Init(void);
void DriveFunction(void const * argument);
void RadioFunction(void const * argument);
void KickFunction(void const * argument);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM5_Init();
    MX_TIM8_Init();
    MX_I2C3_Init();

    // Inicialización de la matriz cinemática
    kinematic[0][0] = sin(WHEEL_ANGlE_1); kinematic[0][1] = -cos(WHEEL_ANGlE_1); kinematic[0][2] = -ROBOT_RADIO;
    kinematic[1][0] = sin(WHEEL_ANGlE_2); kinematic[1][1] = -cos(WHEEL_ANGlE_2); kinematic[1][2] = -ROBOT_RADIO;
    kinematic[2][0] = sin(WHEEL_ANGlE_3); kinematic[2][1] = -cos(WHEEL_ANGlE_3); kinematic[2][2] = -ROBOT_RADIO;
    kinematic[3][0] = sin(WHEEL_ANGlE_4); kinematic[3][1] = -cos(WHEEL_ANGlE_4); kinematic[3][2] = -ROBOT_RADIO;

    // Parpadeo de LEDs para indicar inicio
    for (uint8_t i = 0; i < 5; i++) {
        Board_LedToggle(BOARD_LED_GPIO, BOARD_LED_PIN_1);
        Board_LedToggle(BOARD_LED_GPIO, BOARD_LED_PIN_2);
        Board_LedToggle(BOARD_LED_GPIO, BOARD_LED_PIN_3);
        HAL_Delay(100);
    }

    // Mutex para el sistema de kick
    osMutexDef(kickFlag);
    kickFlagHandle = osMutexCreate(osMutex(kickFlag));

    // Colas de mensajes
    osMessageQDef(kickQueue, 1, uint16_t);
    kickQueueHandle = osMessageCreate(osMessageQ(kickQueue), NULL);
    osMessageQDef(nrf24Check, 16, uint16_t);
    nrf24CheckHandle = osMessageCreate(osMessageQ(nrf24Check), NULL);

    // Tareas principales del sistema
    osThreadDef(driveTask, DriveFunction, osPriorityAboveNormal, 0, 128);
    driveTaskHandle = osThreadCreate(osThread(driveTask), NULL);
    osThreadDef(radioTask, RadioFunction, osPriorityNormal, 0, 128);
    radioTaskHandle = osThreadCreate(osThread(radioTask), NULL);
    osThreadDef(kickTask, KickFunction, osPriorityLow, 0, 128);
    kickTaskHandle = osThreadCreate(osThread(kickTask), NULL);
    osThreadDef(ballDetectorTask, BallDetectorFunction, osPriorityLow, 0, 128);
    ballDetectorTaskHandle = osThreadCreate(osThread(ballDetectorTask), NULL);

    osKernelStart(); // Inicia el scheduler RTOS

    // Bucle infinito de seguridad (no debería llegar aquí)
    while (1) {}
}

// Callback de periodo de timer (para HAL)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
}

// Handler de error simple
void Error_Handler(void)
{
    // Implementar manejo de error si es necesario
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    // Implementar reporte de error si es necesario
}
#endif
