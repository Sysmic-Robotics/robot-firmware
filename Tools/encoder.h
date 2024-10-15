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

#ifndef _ENCODER_H_
#define _ENCODER_H_

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
#include "math.h"

// #define M_PI 					3.14159265358979323846f
#define ENCODER_CPR		4.0f * 2048.0f

typedef enum {
	ENCODER_STATUS_DISABLE = 0,
	ENCODER_STATUS_ENABLE
} Encoder_Status_t;

typedef struct Encoder_Handler {
	volatile uint32_t *count;
	float oldPos;

	uint8_t overflow;	
	int16_t resetVal;
	float minSpeed;

	Encoder_Status_t enable;
} Encoder_Handler_t;

float Encoder_Update(Encoder_Handler_t *encoderDevice, float sampleTime);
float mod(float x, float y);
float AngleNormalize(float a);

#endif
