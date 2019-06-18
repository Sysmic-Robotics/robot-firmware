#include "sysmic_ll_i2c.h"

void I2C_Write(I2C_TypeDef *I2Cx, uint8_t address, uint8_t txData)
{
    LL_I2C_GenerateStartCondition(I2Cx);

    while (!LL_I2C_IsActiveFlag_BUSY(I2Cx));
    
    LL_I2C_DisableBitPOS(I2Cx);

    while(!LL_I2C_IsActiveFlag_SB(I2Cx));
    /* TODO: check address shift */
    LL_I2C_TransmitData8(I2Cx, address & I2C_WRITE_OPERATION);

    while(!LL_I2C_IsActiveFlag_ADDR(I2Cx));

    LL_I2C_ClearFlag_ADDR(I2Cx);

    while(!LL_I2C_IsActiveFlag_TXE(I2Cx));

    LL_I2C_TransmitData8(I2Cx, txData);

    while(!LL_I2C_IsActiveFlag_BTF(I2Cx));

    LL_I2C_GenerateStopCondition(I2Cx);
}

void I2C_WriteMultiple(I2C_TypeDef *I2Cx, uint8_t address, uint8_t *txData, uint16_t len)
{
    LL_I2C_GenerateStartCondition(I2Cx);

    while (!LL_I2C_IsActiveFlag_BUSY(I2Cx));
    
    LL_I2C_DisableBitPOS(I2Cx);

    while(!LL_I2C_IsActiveFlag_SB(I2Cx));
    /* TODO: check address shift */
    LL_I2C_TransmitData8(I2Cx, address & I2C_WRITE_OPERATION);

    while(!LL_I2C_IsActiveFlag_ADDR(I2Cx));

    LL_I2C_ClearFlag_ADDR(I2Cx);

    while (len)
    {
        while(!LL_I2C_IsActiveFlag_TXE(I2Cx));

        LL_I2C_TransmitData8(I2Cx, txData);
        len--;

        if((LL_I2C_IsActiveFlag_TXE(I2Cx) && LL_I2C_IsActiveFlag_BTF(I2Cx)) && len > 0)
        {
            LL_I2C_TransmitData8(I2Cx, txData);
            len--;
        }

        while(!LL_I2C_IsActiveFlag_BTF(I2Cx));
    }

    LL_I2C_GenerateStopCondition(I2Cx);
}

uint8_t I2C_Read(I2C_TypeDef *I2Cx, uint8_t address)
{
    uint8_t rxData;

    LL_I2C_GenerateStartCondition(I2Cx);

    while (!LL_I2C_IsActiveFlag_BUSY(I2Cx));
    
    LL_I2C_DisableBitPOS(I2Cx);

    while(!LL_I2C_IsActiveFlag_SB(I2Cx));
    /* TODO: check address shift */
    LL_I2C_TransmitData8(I2Cx, address & I2C_READ_OPERATION);

    while(!LL_I2C_IsActiveFlag_ADDR(I2Cx));

    LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
    LL_I2C_GenerateStopCondition(I2Cx);
    LL_I2C_ClearFlag_ADDR(I2Cx);

    while(!LL_I2C_IsActiveFlag_RXNE(I2Cx));

    rxData = LL_I2C_ReceiveData8(I2Cx);

    while(!LL_I2C_IsActiveFlag_BTF(I2Cx));

    return rxData;
}

void I2C_ReadMultiple(I2C_TypeDef *I2Cx, uint8_t address, uint8_t *rxData, uint16_t len)
{
    LL_I2C_GenerateStartCondition(I2Cx);

    while (!LL_I2C_IsActiveFlag_BUSY(I2Cx));
    
    LL_I2C_DisableBitPOS(I2Cx);

    while(!LL_I2C_IsActiveFlag_SB(I2Cx));
    /* TODO: check address shift */
    LL_I2C_TransmitData8(I2Cx, address & I2C_READ_OPERATION);

    while(!LL_I2C_IsActiveFlag_ADDR(I2Cx));

    if(len == 2)
    {
        LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
        LL_I2C_ClearFlag_ADDR(I2Cx);
        LL_I2C_EnableBitPOS(I2Cx);
    }
    else
    {
        LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
        LL_I2C_ClearFlag_ADDR(I2Cx);
    }

    while (len)
    {
        if(len == 2)
        {
            while(!LL_I2C_IsActiveFlag_BTF(I2Cx));

            rxData++ = LL_I2C_ReceiveData8(I2Cx);
            len--;

            LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
            LL_I2C_GenerateStopCondition(I2Cx);

            rxData++ = LL_I2C_ReceiveData8(I2Cx);
            len--;
        }
        else if(len == 3)
        {
            while(!LL_I2C_IsActiveFlag_BTF(I2Cx));

            LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
            rxData++ = LL_I2C_ReceiveData8(I2Cx);
            len--;

            while(!LL_I2C_IsActiveFlag_BTF(I2Cx));
            LL_I2C_GenerateStopCondition(I2Cx);

            rxData++ = LL_I2C_ReceiveData8(I2Cx);
            len--;

            rxData++ = LL_I2C_ReceiveData8(I2Cx);
            len--;
        }
        else
        {
            while(!LL_I2C_IsActiveFlag_RXNE(I2Cx));

            rxData++ = LL_I2C_ReceiveData8(I2Cx);
            len--;

            if((LL_I2C_IsActiveFlag_RXNE(I2Cx) && LL_I2C_IsActiveFlag_BTF(I2Cx)) && len > 0)
            {
                rxData++ = LL_I2C_ReceiveData8(I2Cx);
                len--;
            }

            while(!LL_I2C_IsActiveFlag_BTF(I2Cx));
        }        
    }
}
