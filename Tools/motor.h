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
#if defined(STM32F7xx) || defined(STM32F7XX)
#ifndef STM32F7xx
#define STM32F7xx
#endif
#ifndef STM32F7XX
#define STM32F7XX
#endif
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#endif

#include "stm32f7xx_hal.h"
#include "encoder.h"
#include "PID.h"
#include "MAX5814.h"

#define MOTOR_MAX_SPEED_RPS		10.0
#define MOTOR_SPEED_CONV(x)		x * (MAX5814_MAX_VAL / MOTOR_MAX_SPEED_RPS)

typedef enum
{
	MOTOR_STATUS_DISABLE = 0,
	MOTOR_STATUS_ENABLE
} Motor_Status_t;

typedef struct Motor_Params
{
	uint8_t motorID;
	
	/* Re-think this shietz */
} Motor_Params_t;

typedef struct Motor_GPIO
{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
} Motor_GPIO_t;

typedef struct Motor_Handler
{	
	PID_Handler_t pid;
	MAX5814_Handler_t dac;
	Encoder_Handler_t encoder;

	Motor_GPIO_t enablePin;
	Motor_GPIO_t dirPin;
	Motor_GPIO_t brakePin;

	uint8_t outputID;
	float refSpeed;
	float measSpeed;

	uint16_t voltage;

	Motor_Status_t enable;
} Motor_Handler_t;

void Motor_Init(Motor_Handler_t *motorDevice, uint8_t motorID, Motor_Status_t enable);
void Motor_Drive(Motor_Handler_t *motorDevice, float refSpeed, MAX5814_Handler_t *dacDevice);
void Motor_Enable(Motor_Handler_t *motorDevice, Motor_Status_t enable);
void Motor_SetVoltage(Motor_Handler_t *motorDevice, MAX5814_Handler_t *dacDevice);

#endif
