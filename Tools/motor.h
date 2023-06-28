/**
 * @author  Pablo Reyes Robles
 * @email   pablo.reyesr@alumnos.usm.cl
 * @version v1.0
 * @license UTFSM
@verbatim
   ----------------------------------------------------------------------
    Copyright (C) Sysmic Robotics, 2019
   ----------------------------------------------------------------------
@endverbatim
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

/* Include proper header file */
/* STM32F7xx */
#if defined(STM32F0xx) || defined(STM32F0XX)
#ifndef STM32F0xx
#define STM32F0xx
#endif
#ifndef STM32F0XX
#define STM32F0XX
#endif
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#endif

/* STM32F4xx */
#if defined(STM32F4xx) || defined(STM32F4XX) || defined(STM32F4)
#ifndef STM32F4xx
#define STM32F4xx
#endif
#ifndef STM32F4XX
#define STM32F4XX
#endif
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#endif

/* STM32F7xx */
#if defined(STM32F767xx) || defined(STM32F7XX)
#ifndef STM32F7xx
#define STM32F7xx
#endif
#ifndef STM32F7XX
#define STM32F7XX
#endif
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#endif

#include "encoder.h"
#include "pid.h"
#include "MAX581x.h"

enum {
	WHEEL_P_ROTATION = 0,
	WHEEL_N_ROTATION
};

#define WHEEL_ANGlE_1					60.0f * M_PI / 180.0f
#define WHEEL_ANGlE_2					130.0f * M_PI / 180.0f
#define WHEEL_ANGlE_3					-130.0f * M_PI / 180.0f
#define WHEEL_ANGlE_4					-60.0f * M_PI / 180.0f

/* Open loop */
#define MOTOR_NOMINAL_SPEED		(2.0f * M_PI * (5240.0f / 60.0f))	// rpm -> rad/s
#define MOTOR_SPEED_CONV			4095.0f / MOTOR_NOMINAL_SPEED // remove 0.25f factor to operate on max range 4095.0. Division setted on debug sessions

#define WHEEL_MAX_SPEED_RPS		15.0f
#define WHEEL_MAX_SPEED_RAD		2.0f * M_PI * WHEEL_MAX_SPEED_RPS
#define WHEEL_RADIO						0.02704f // without little wheels: 0.025f
#define WHEEL_GEAR_RATIO			51.0f / 17.0f

#define PID_SAMPLE_TIME				1.0f	// [ms]

//#define SPEED_CNT_RATIO				(WHEEL_GEAR_RATIO * ENCODER_CPR) / ((PID_SAMPLE_TIME * 1000.0f) * 2.0f * M_PI * WHEEL_RADIO) // [m/s] -> [count/(pid_samples * ms)]
#define SPEED_CNT_RATIO       (WHEEL_GEAR_RATIO) / (WHEEL_RADIO) // [m/s] -> [rad/s]

enum {
	MOTOR_BRAKE_DISABLE = 0,
	MOTOR_BRAKE_ENABLE
};

typedef enum
{
	MOTOR_STATUS_DISABLE = 0,
	MOTOR_STATUS_ENABLE
} Motor_Status_t;

typedef struct Motor_GPIO
{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
} Motor_GPIO_t;

typedef struct Motor_Handler
{
	PID_Handler_t pid;
	MAX581x_Handler_t dac;
	Encoder_Handler_t encoder;

	Motor_GPIO_t enablePin;
	Motor_GPIO_t dirPin;
	Motor_GPIO_t brakePin;

	uint8_t outputID;
	volatile float refSpeed;
	volatile float measSpeed;

	uint16_t voltage;
	Motor_Status_t enable;
} Motor_Handler_t;

void Motor_Init(Motor_Handler_t *motorDevice, uint8_t motorID, Motor_Status_t enable);

/**
 * \brief Open loop motor drive
 * \param motorDevice: reference to motor struct
 * \param dacDevice: reference to DAC struct
 * \param speed: reference speed of wheel to convert into motor speed
 */
void Motor_OLDrive(Motor_Handler_t *motorDevice, MAX581x_Handler_t *dacDevice, float speed);
void Motor_CLDrive(Motor_Handler_t *motorDevice, MAX581x_Handler_t *dacDevice, float speed);
void Motor_Enable(Motor_Handler_t *motorDevice, Motor_Status_t enable);
void Motor_SetBrake(Motor_Handler_t *motorDevice, uint8_t brake);
void Motor_SetVoltage(Motor_Handler_t *motorDevice, MAX581x_Handler_t *dacDevice, float speed);

#endif
