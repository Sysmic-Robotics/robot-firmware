/**
 * @author  Pablo Reyes Robles
 * @email   pablo.reyesr@alumnos.usm.cl
 * @version v1.0
@verbatim
   ----------------------------------------------------------------------
   Sysmic Robotics, 2019
   ----------------------------------------------------------------------
@endverbatim
 */

#ifndef _MAX5814_H_
#define _MAX5814_H_

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

#define MAX5814_ADDRESS                 0x00
#define MAX5814_WRITE_OPERATION         0x00
#define MAX5814_READ_OPERATION          0x01

#define MAX5814_CMD_CODEn               0x00
#define MAX5814_CMD_LOADn               0x10
#define MAX5814_CMD_CODEn_LOAD_ALL      0x20
#define MAX5814_CMD_CODEn_LOADn         0x30
#define MAX5814_CMD_CODE_ALL            0x80
#define MAX5814_CMD_LOAD_ALL            0x81
#define MAX5814_CMD_CODE_ALL_LOAD_ALL   0x82

#define MAX5814_CMD_POWER               0x40
#define MAX5814_CMD_SW_CLEAR			0x50
#define MAX5814_CMD_SW_RESET			0x51
#define MAX5814_CMD_CONFIG				0x60
#define MAX5814_CMD_REF					0x70

#define MAX5814_REF_EXT					0x00
#define MAX5814_REF_25					0x01
#define MAX5814_REF_20					0x02
#define MAX5814_REF_41					0x03
#define MAX5814_REF_PWR_ON				0x04

#define MAX5814_OUTPUT_A				0x00
#define MAX5814_OUTPUT_B				0x01
#define MAX5814_OUTPUT_C				0x02
#define MAX5814_OUTPUT_D				0x03
#define MAX5814_OUTPUT_ALL				0x04

#define MAX5814_ENABLE_LATCH			0x00
#define MAX5814_DISABLE_LATCH			0x01
#define MAX5814_SEL_ALL					0x40
#define MAX5814_SEL_A					0x01
#define MAX5814_SEL_B					0x02
#define MAX5814_SEL_C					0x04
#define MAX5814_SEL_D					0x08

#define MAX5814_MAX_VAL					1023

typedef struct MAX5814 {
	I2C_HandleTypeDef *i2cHandler;
	uint8_t i2cAddress;
	uint8_t txBuffer[3];
	uint8_t rxBuffer[3];
} MAX5814_Handler_t;

void MAX5814_WriteCommand(MAX5814_Handler_t *dacDevice);
void MAX5814_ReadCommand(MAX5814_Handler_t *dacDevice);
void MAX5814_Init(MAX5814_Handler_t *dacDevice, I2C_HandleTypeDef *hi2c, uint8_t dacRefSelector);
void MAX5814_Config(MAX5814_Handler_t *dacDevice, uint8_t dacLatch, uint8_t dacConfigSelector);
void MAX5814_Reference(MAX5814_Handler_t *dacDevice, uint8_t dacRefConfig);
void MAX5814_Code(MAX5814_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData);
void MAX5814_Load(MAX5814_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData);
void MAX5814_CodeLoadAll(MAX5814_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData);
void MAX5814_CodeLoad(MAX5814_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData);
void MAX5814_CodeAll(MAX5814_Handler_t *dacDevice, uint16_t dacData);
void MAX5814_LoadAll(MAX5814_Handler_t *dacDevice);
void MAX5814_CodeAllLoadAll(MAX5814_Handler_t *dacDevice, uint16_t dacData);

#endif
