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

#include "stm32f4xx_ll_i2c.h"

enum {
    I2C_WRITE_OPERATION = 0,
    I2C_READ_OPERATION
};

/**
 * \brief Write a single byte through I2C
 * \param I2Cx: I2C instance
 * \param address: device address
 * \param txData: byte to send
 */
void I2C_Write(I2C_TypeDef *I2Cx, uint8_t address, uint8_t txData);

/**
 * \brief Write multiple bytes through I2C
 * \param I2Cx: I2C instance
 * \param address: device address
 * \param txData: data buffer to send
 * \param len: data array length
 */
void I2C_WriteMultiple(I2C_TypeDef *I2Cx, uint8_t address, uint8_t *txData, uint16_t len);

/**
 * \brief Write a single byte through I2C
 * \param I2Cx: I2C instance
 * \param address: device address
 * \return Byte to receive
 */
uint8_t I2C_Read(I2C_TypeDef *I2Cx, uint8_t address);

/**
 * \brief Write multiple bytes through I2C
 * \param I2Cx: I2C instance
 * \param address: device address
 * \param rxData: data buffer to receive data
 * \param len: data array length
 */
void I2C_ReadMultiple(I2C_TypeDef *I2Cx, uint8_t address, uint8_t *rxData, uint16_t len);
