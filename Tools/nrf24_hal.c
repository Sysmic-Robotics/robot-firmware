#include "nrf24_hal.h"

// Configure the GPIO lines of the nRF24L01 transceiver
// note: IRQ pin must be configured separately

void nRF24_GPIO_Init(void)
{
    //GPIO_InitTypeDef PORT;

    // Enable the nRF24L01 GPIO peripherals
	//RCC->APB2ENR |= nRF24_GPIO_PERIPHERALS;

    // Configure CSN pin
	//PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	//PORT.GPIO_Speed = GPIO_Speed_2MHz;
	//PORT.GPIO_Pin = nRF24_CSN_PIN;
	//GPIO_Init(nRF24_CSN_PORT, &PORT);
	nRF24_CSN_H();

	// Configure CE pin
	//PORT.GPIO_Pin = nRF24_CE_PIN;
	//GPIO_Init(nRF24_CE_PORT, &PORT);
	nRF24_CE_L();
}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data)
{
	return SPI_Write(nRF24_SPI_PORT, data);
}
