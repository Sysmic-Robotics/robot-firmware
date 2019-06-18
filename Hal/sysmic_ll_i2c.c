#include "sysmic_ll_spi.h"

uint8_t I2C_Write(I2C_TypeDef *I2Cx, uint8_t txData)
{
    LL_I2C_GenerateStartCondition(I2Cx);

    while (!LL_I2C_IsActiveFlag_BUSY(I2Cx));
    
    
}

void I2C_WriteMultiple(I2C_TypeDef *I2Cx, uint8_t *txData, uint16_t len)
{
    
}

void I2C_ReadMultiple(I2C_TypeDef *I2Cx, uint8_t *rxData, uint8_t dummyData, uint16_t len)
{
    
}