/**
 * @author  Pablo Reyes Robles
 * @email   pablo.reyesr@alumnos.usm.cl
 * @version v1.0
 * @license UTFSM
@verbatim
   ----------------------------------------------------------------------
    Copyright (C) Sysmic Robotics, 2019
   ----------------------------------------------------------------------
@endverbatim
 */

#include "stm32f4xx_ll_usart.h"

/**
 * \brief Write a single byte through USART
 * \param USARTx: USART instance
 * \param txData: byte to send
 */
uint8_t UART_Write(USART_TypeDef *USARTx, uint8_t txData);

/**
 * \brief Write multiple bytes through USART
 * \param USARTx: USART instance
 * \param txData: data buffer to send
 * \param len: data array length
 */
void USART_WriteMultiple(USART_TypeDef *USARTx, uint8_t *txData, uint16_t len);

/**
 * \brief Write multiple bytes through USART
 * \param USARTx: USART instance
 * \param rxData: data buffer to receive data
 * \param dummyData: dummy byte to receive data from device
 * \param len: data array length
 */
void USART_ReadMultiple(USART_TypeDef *USARTx, uint8_t *rxData, uint8_t dummyData, uint16_t len);