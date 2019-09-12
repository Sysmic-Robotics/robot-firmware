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

#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H

/* Hardware abstraction layer for nRF24L01+ transceiver */

/* Peripheral libraries */
#include "sysmic_ll_spi.h"
#include "stm32f7xx_ll_gpio.h"

/* SPI port peripheral */
#define nRF24_SPI_PORT             SPI1

/* CE (chip enable) pin */
#define nRF24_CE_PORT              GPIOG
#define nRF24_CE_PIN               GPIO_PIN_9
#define nRF24_CE_L()               HAL_GPIO_WritePin(nRF24_CE_PORT, nRF24_CE_PIN, GPIO_PIN_RESET)
#define nRF24_CE_H()               HAL_GPIO_WritePin(nRF24_CE_PORT, nRF24_CE_PIN, GPIO_PIN_SET)

/* CSN (chip select negative) */
#define nRF24_CSN_PORT             GPIOG
#define nRF24_CSN_PIN              GPIO_PIN_10
#define nRF24_CSN_L()              HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN, GPIO_PIN_RESET)
#define nRF24_CSN_H()              HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN, GPIO_PIN_SET)

/* IRQ pin */
#define nRF24_IRQ_PORT             GPIOG
#define nRF24_IRQ_PIN              GPIO_PIN_1

/**
 * \brief Set and reset the GPIO lines of the nRF24L01+ transceiver
 */
void nRF24_GPIO_Init(void);

/**
 * \brief Low level SPI transmit/receive function (hardware depended)
 * \param data: Value to transmit via SPI
 * \return Value received from SPI
 */
uint8_t nRF24_LL_RW(uint8_t data);

#endif
