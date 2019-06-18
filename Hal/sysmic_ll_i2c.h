#include "stm32f4xx_ll_i2c.h"

/**
 * @brief Write a single byte through I2C
 * @param I2Cx: I2C instance
 * @param txData: byte to send
 */
uint8_t I2C_Write(I2C_TypeDef *I2Cx, uint8_t txData);

/**
 * @brief Write multiple bytes through I2C
 * @param I2Cx: I2C instance
 * @param txData: data buffer to send
 * @param len: data array length
 */
void I2C_WriteMultiple(I2C_TypeDef *I2Cx, uint8_t *txData, uint16_t len);

/**
 * @brief Write multiple bytes through I2C
 * @param I2Cx: I2C instance
 * @param rxData: data buffer to receive data
 * @param dummyData: dummy byte to receive data from device
 * @param len: data array length
 */
void I2C_ReadMultiple(I2C_TypeDef *I2Cx, uint8_t *rxData, uint8_t dummyData, uint16_t len);