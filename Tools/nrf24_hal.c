/**
 * @author  Pablo Reyes Robles
 * @email   pablo.reyesr@alumnos.usm.cl
 * @version v1.0
@verbatim
   ----------------------------------------------------------------------
   Created by LonelyWolf. https://github.com/LonelyWolf/stm32
   Mod by Sysmic Robotics, 2019
   ----------------------------------------------------------------------
@endverbatim
 */

#include "nrf24_hal.h"

void nRF24_GPIO_Init(void)
{
	nRF24_CSN_H();
	nRF24_CE_L();
}

uint8_t nRF24_LL_RW(uint8_t data)
{
	return SPI_Write(nRF24_SPI_PORT, data);
}
