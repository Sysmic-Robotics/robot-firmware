#include "MAX581x.h"

void MAX581x_WriteCommand(MAX581x_Handler_t *dacDevice)
{
	dacDevice->i2cAddress = (0x10 | MAX581x_ADDRESS) << 1;
	HAL_I2C_Master_Transmit(dacDevice->i2cHandler, dacDevice->i2cAddress, dacDevice->txBuffer, 3, 1000);
}

void MAX581x_ReadCommand(MAX581x_Handler_t *dacDevice)
{
	dacDevice->i2cAddress = (0x10 | MAX581x_ADDRESS) << 1;
	HAL_I2C_Master_Transmit(dacDevice->i2cHandler, dacDevice->i2cAddress, dacDevice->txBuffer, 3, 1000);
}

void MAX581x_Init(MAX581x_Handler_t *dacDevice, I2C_HandleTypeDef *hi2c, uint8_t dacRefSelector)
{
	dacDevice->i2cHandler = hi2c;

	MAX581x_Config(dacDevice, MAX581x_DISABLE_LATCH | MAX581x_SEL_ALL, MAX581x_SEL_A | MAX581x_SEL_B | MAX581x_SEL_C | MAX581x_SEL_D);
	MAX581x_Reference(dacDevice, dacRefSelector | MAX581x_REF_PWR_ON);
}

void MAX581x_Config(MAX581x_Handler_t *dacDevice, uint8_t dacLatch, uint8_t dacConfigSelector)
{
	dacDevice->txBuffer[0] = MAX581x_CMD_CONFIG | dacLatch;
	dacDevice->txBuffer[1] = dacConfigSelector;
	dacDevice->txBuffer[2] = 0x00;

	MAX581x_WriteCommand(dacDevice);
}

void MAX581x_Reference(MAX581x_Handler_t *dacDevice, uint8_t dacRefConfig)
{
	dacDevice->txBuffer[0] = MAX581x_CMD_REF | dacRefConfig;
	dacDevice->txBuffer[1] = 0x00;
	dacDevice->txBuffer[2] = 0x00;

	MAX581x_WriteCommand(dacDevice);
}

void MAX581x_Code(MAX581x_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX581x_CMD_CODEn | dacSelector;
	dacDevice->txBuffer[1] = (uint8_t)((dacData >> MAX581x_RIGHT_SHIFT) & 0xFF);
	dacDevice->txBuffer[2] = (uint8_t)((dacData << MAX581x_LEFT_SHIFT) & 0xF0);

	MAX581x_WriteCommand(dacDevice);
}

void MAX581x_Load(MAX581x_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX581x_CMD_LOADn | dacSelector;
	dacDevice->txBuffer[1] = 0x00;
	dacDevice->txBuffer[2] = 0x00;

	MAX581x_WriteCommand(dacDevice);
}

void MAX581x_CodeLoadAll(MAX581x_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX581x_CMD_CODEn_LOAD_ALL | dacSelector;
	dacDevice->txBuffer[1] = (uint8_t)((dacData >> MAX581x_RIGHT_SHIFT) & 0xFF);
	dacDevice->txBuffer[2] = (uint8_t)((dacData << MAX581x_LEFT_SHIFT) & 0xF0);

	MAX581x_WriteCommand(dacDevice);
}

void MAX581x_CodeLoad(MAX581x_Handler_t *dacDevice, uint8_t dacSelector, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX581x_CMD_CODEn_LOADn | dacSelector;
	dacDevice->txBuffer[1] = (uint8_t)((dacData >> MAX581x_RIGHT_SHIFT) & 0xFF);
	dacDevice->txBuffer[2] = (uint8_t)((dacData << MAX581x_LEFT_SHIFT) & 0xF0);

	MAX581x_WriteCommand(dacDevice);
}

void MAX581x_CodeAll(MAX581x_Handler_t *dacDevice, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX581x_CMD_CODE_ALL;
	dacDevice->txBuffer[1] = (uint8_t)((dacData >> MAX581x_RIGHT_SHIFT) & 0xFF);
	dacDevice->txBuffer[2] = (uint8_t)((dacData << MAX581x_LEFT_SHIFT) & 0xF0);

	MAX581x_WriteCommand(dacDevice);
}

void MAX581x_LoadAll(MAX581x_Handler_t *dacDevice)
{
	dacDevice->txBuffer[0] = MAX581x_CMD_LOAD_ALL;
	dacDevice->txBuffer[1] = 0x00;
	dacDevice->txBuffer[2] = 0x00;

	MAX581x_WriteCommand(dacDevice);
}

void MAX581x_CodeAllLoadAll(MAX581x_Handler_t *dacDevice, uint16_t dacData)
{
	dacDevice->txBuffer[0] = MAX581x_CMD_CODE_ALL_LOAD_ALL;
	dacDevice->txBuffer[1] = (uint8_t)((dacData >> MAX581x_RIGHT_SHIFT) & 0xFF);
	dacDevice->txBuffer[2] = (uint8_t)((dacData << MAX581x_LEFT_SHIFT) & 0xF0);

	MAX581x_WriteCommand(dacDevice);
}
