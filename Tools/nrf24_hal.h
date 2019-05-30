#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H


// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


// Peripheral libraries
#include "sysmic_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"

// SPI port peripheral
#define nRF24_SPI_PORT             SPI1

// nRF24 GPIO peripherals
//#define nRF24_GPIO_PERIPHERALS     (RCC_APB2ENR_IOPBEN)

// CE (chip enable) pin (PB11)
#define nRF24_CE_PORT              GPIOC
#define nRF24_CE_PIN               LL_GPIO_PIN_4
#define nRF24_CE_L()               LL_GPIO_ResetOutputPin(nRF24_CE_PORT, nRF24_CE_PIN)
#define nRF24_CE_H()               LL_GPIO_SetOutputPin(nRF24_CE_PORT, nRF24_CE_PIN)

// CSN (chip select negative) pin (PB12)
#define nRF24_CSN_PORT             GPIOA
#define nRF24_CSN_PIN              LL_GPIO_PIN_4
#define nRF24_CSN_L()              LL_GPIO_ResetOutputPin(nRF24_CSN_PORT, nRF24_CSN_PIN)
#define nRF24_CSN_H()              LL_GPIO_SetOutputPin(nRF24_CSN_PORT, nRF24_CSN_PIN)

// IRQ pin (PB10)
#define nRF24_IRQ_PORT             GPIOB
#define nRF24_IRQ_PIN              LL_GPIO_PIN_10


// Function prototypes
void nRF24_GPIO_Init(void);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
