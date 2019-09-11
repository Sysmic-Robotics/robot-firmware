#include "sysmic_ll_spi.h"

uint8_t SPI_Write(SPI_TypeDef *SPIx, uint8_t txData)
{
	if(!LL_SPI_IsEnabled(SPIx))
	{
		return 0;
	}
	while(!LL_SPI_IsActiveFlag_TXE(SPIx));

	LL_SPI_TransmitData8(SPIx, txData);

	while(!LL_SPI_IsActiveFlag_RXNE(SPIx));

	return LL_SPI_ReceiveData8(SPIx);
}

void SPI_WriteMultiple(SPI_TypeDef *SPIx, uint8_t *txData, uint16_t len)
{
	if(!LL_SPI_IsEnabled(SPIx))
	{
		return;
	}

	while(len--)
	{
		while(!LL_SPI_IsActiveFlag_TXE(SPIx));

		LL_SPI_TransmitData8(SPIx, *txData++);

		while(!LL_SPI_IsActiveFlag_RXNE(SPIx));

		LL_SPI_ReceiveData8(SPIx);
	}	
}

void SPI_ReadMultiple(SPI_TypeDef *SPIx, uint8_t *rxData, uint8_t dummyData, uint16_t len)
{
	if(!LL_SPI_IsEnabled(SPIx))
	{
		return;
	}

	while(len--)
	{
		while(!LL_SPI_IsActiveFlag_TXE(SPIx));
		
		LL_SPI_TransmitData8(SPIx, dummyData);

		while(!LL_SPI_IsActiveFlag_RXNE(SPIx));

		*rxData++ = LL_SPI_ReceiveData8(SPIx);
	}	
}
