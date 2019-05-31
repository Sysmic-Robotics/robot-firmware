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

#include "MAX5814.h"

void MAX5814_WriteCommand(MAX5814_Handler_t *dacDevice)
{
	dacDevice->i2cAddress = 0x10 << 1 | MAX5814_ADDRESS | MAX5814_WRITE_OPERATION;
	HAL_I2C_Master_Transmit(dacDevice->i2cHandler, dacDevice->i2cAddress, dacDevice->txBuffer, 3, 1000);
}

void MAX5814_ReadCommand(MAX5814_Handler_t *dacDevice)
{
	dacDevice->i2cAddress = 0x10 << 1 | MAX5814_ADDRESS | MAX5814_READ_OPERATION;
	HAL_I2C_Master_Transmit(dacDevice->i2cHandler, dacDevice->i2cAddress, dacDevice->txBuffer, 3, 1000);
}

void MAX5814_Init(MAX5814_Handler_t *dacDevice, I2C_HandleTypeDef *hi2c, uint8_t dacRefSelector)
{
	dacDevice->i2cHandler = hi2c;

	MAX5814_Config(dacDevice, MAX5814_DISABLE_LATCH | MAX5814_SEL_ALL, MAX5814_SEL_A | MAX5814_SEL_B | MAX5814_SEL_C | MAX5814_SEL_D);
	MAX5814_Reference(dacDevice, dacRefSelector | MAX5814_REF_PWR_ON);
}

void MAX5814_Config(MAX5814_Handler_t *dacDevice, uint8_t dacLatch, uint8_t dacConfigSelector)
{
	dacDevice->txBuffer[0] = MAX5814_CMD_CONFIG | dacLatch;
	dacDevice->txBuffer[1] = dacConfigSelector;
	dacDevice->txBuffer[2] = 0x00;

	MAX5814_WriteCommand(dacDevice);
}

void MAX5814_Reference(MAX5814_Handler_t *dacDevice, uint8_t dacRefConfig)
{
	dacDevice->txBuffer[0] = MAX5814_CMD_REF | dacRefConfig;
	dacDevice->txBuffer[1] = 0x00;
	dacDevice->txBuffer[2] = 0x00;

	MAX5814_WriteCommand(dacDevice);
}

void MAX5814_Code(MAX5814_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX5814_CMD_CODEn | dacSelector;
	dacDevice->txBuffer[1] = (uint8_t)((dacData >> 2) & 0xFF);
	dacDevice->txBuffer[2] = (uint8_t)((dacData << 6) & 0xF0);

	MAX5814_WriteCommand(dacDevice);
}

void MAX5814_Load(MAX5814_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX5814_CMD_LOADn | dacSelector;
	dacDevice->txBuffer[1] = 0x00;
	dacDevice->txBuffer[2] = 0x00;

	MAX5814_WriteCommand(dacDevice);
}

void MAX5814_CodeLoadAll(MAX5814_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX5814_CMD_CODEn_LOAD_ALL | dacSelector;
	dacDevice->txBuffer[1] = (uint8_t)((dacData >> 2) & 0xFF);
	dacDevice->txBuffer[2] = (uint8_t)((dacData << 6) & 0xF0);

	MAX5814_WriteCommand(dacDevice);
}

void MAX5814_CodeLoad(MAX5814_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX5814_CMD_CODEn_LOADn | dacSelector;
	dacDevice->txBuffer[1] = (uint8_t)((dacData >> 2) & 0xFF);
	dacDevice->txBuffer[2] = (uint8_t)((dacData << 6) & 0xF0);

	MAX5814_WriteCommand(dacDevice);
}

void MAX5814_CodeAll(MAX5814_Handler_t *dacDevice, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX5814_CMD_CODE_ALL;
	dacDevice->txBuffer[1] = (uint8_t)((dacData >> 2) & 0xFF);
	dacDevice->txBuffer[2] = (uint8_t)((dacData << 6) & 0xF0);

	MAX5814_WriteCommand(dacDevice);
}

void MAX5814_LoadAll(MAX5814_Handler_t *dacDevice)
{
	dacDevice->txBuffer[0] = MAX5814_CMD_LOAD_ALL;
	dacDevice->txBuffer[1] = 0x00;
	dacDevice->txBuffer[2] = 0x00;

	MAX5814_WriteCommand(dacDevice);
}

void MAX5814_CodeAllLoadAll(MAX5814_Handler_t *dacDevice, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX5814_CMD_CODE_ALL_LOAD_ALL;
	dacDevice->txBuffer[1] = (uint8_t)((dacData >> 2) & 0xFF);
	dacDevice->txBuffer[2] = (uint8_t)((dacData << 6) & 0xF0);

	MAX5814_WriteCommand(dacDevice);
}
