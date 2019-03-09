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

#ifndef _PID_H_
#define _PID_H_

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

#include "stm32f4xx_hal.h"

typedef enum
{
	PID_STATUS_DISABLE = 0,
	PID_STATUS_ENABLE
} PID_Status_t;

typedef struct PID_Params
{
	double Kp;
	double Ki;
	double Kd;

	double outputMax;
	double outputMin;

	double sampleTime;
} PID_Params_t;

typedef struct PID_Handler
{
	PID_Params_t params;
	
	double ref;
	double error;
	double output;

	double lastMeasure;
	double integral;

	PID_Status_t enable;
} PID_Handler_t;


void PID_Init(PID_Handler_t *pidDevice, PID_Params_t params, uint8_t enable);
void PID_CloseLoop(PID_Handler_t *pidDevice, double measure);
void PID_SetController(PID_Handler_t *pidDevice, double Kp, double Ki, double Kd);
void PID_SetParams(PID_Handler_t *pidDevice, PID_Params_t params);
void PID_Enable(PID_Handler_t *pidDevice, uint8_t enable);

#endif
