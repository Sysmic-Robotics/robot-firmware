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

#include "stm32f7xx_hal.h"

/**
 * \brief Enable status of PID
 */
typedef enum {
	PID_STATUS_DISABLE = 0,
	PID_STATUS_ENABLE
} PID_Status_t;

/**
 * \brief Configuration PID parameters
 */
typedef struct {
	float Kp;
	float Ki;
	float Kd;

	float outputMax;
	float outputMin;
	
	float integralMax;

	float sampleTime;
} PID_Params_t;

/**
 * \brief PID handler for control operation
 */
typedef struct {
	PID_Params_t params;
	
	float ref;
	float error;
	float output;

	float lastMeasure;
	float integral;

	PID_Status_t enable;
} PID_Handler_t;

/**
 * \brief PID initialization
 * \param pid: Struct that store PID operation info
 * \param params: Struct that contains PID configuration parameters
 * \param enable: Enable status of PID
 */
void PID_Init(PID_Handler_t *pid, PID_Params_t params, PID_Status_t enable);

/**
 * \brief Execute a close loop operation
 * \param pid: Struct that store PID operation info
 * \param reference: Reference set to PID controller
 * \param measure: Feedback used to compute error
 */
void PID_CloseLoop(PID_Handler_t *pid, float reference, float measure);

/**
 * \brief Set PID parameters
 * \param pid: Struct that store PID operation info
 * \param Kp: Proportional gain
 * \param Ki: Integral gain
 * \param Kd: Derivative gain
 */
void PID_SetController(PID_Handler_t *pid, float Kp, float Ki, float Kd);

/**
 * \brief Set parameters from struct
 * \param pid: Struct that store PID operation info
 * \param params: Struct with PID configuration parameters
 */
void PID_SetParams(PID_Handler_t *pid, PID_Params_t params);

/**
 * \brief Enable or disable PID
 * \param pid: Struct that store PID operation info
 * \param enable: PID set status
 */
void PID_Enable(PID_Handler_t *pid, PID_Status_t enable);

#endif
