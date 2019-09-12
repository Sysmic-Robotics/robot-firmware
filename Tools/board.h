/**
 * @author  Pablo Reyes Robles
 * @email   pablo.reyesr@alumnos.usm.cl
 * @version v0.1
 * @license UTFSM
@verbatim
   ----------------------------------------------------------------------
    Copyright (C) Sysmic Robotics, 2019
   ----------------------------------------------------------------------
@endverbatim
 */

#ifndef __SYSMIC_BOARD_H
#define __SYSMIC_BOARD_H

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#define BOARD_LED_GPIO		GPIOI
#define BOARD_LED_PIN_1		GPIO_PIN_12
#define BOARD_LED_PIN_2		GPIO_PIN_13
#define BOARD_LED_PIN_3		GPIO_PIN_14

/**
 * \brief Set led pin on board
 * \param Led_GPIO: Led port
 * \param Led_Pin: Port pin attached to led
 */
void Board_LedSet(GPIO_TypeDef* Led_GPIO, uint16_t Led_Pin);

/**
 * \brief Reset led pin on board
 * \param Led_GPIO: Led port
 * \param Led_Pin: Port pin attached to led
 */
void Board_LedReset(GPIO_TypeDef* Led_GPIO, uint16_t Led_Pin);

/**
 * \brief Toggle led pin on board
 * \param Led_GPIO: Led port
 * \param Led_Pin: Port pin attached to led
 */
void Board_LedToggle(GPIO_TypeDef* Led_GPIO, uint16_t Led_Pin);

#endif
