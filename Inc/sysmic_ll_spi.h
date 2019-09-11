#include "stm32f7xx_ll_spi.h"

/**
 * @brief Write a single byte through SPI
 * @param SPIx: SPI instance
 * @param txData: byte to send
 */
uint8_t SPI_Write(SPI_TypeDef *SPIx, uint8_t txData);

/**
 * @brief Write multiple bytes through SPI
 * @param SPIx: SPI instance
 * @param txData: data buffer to send
 * @param len: data array length
 */
void SPI_WriteMultiple(SPI_TypeDef *SPIx, uint8_t *txData, uint16_t len);

/**
 * @brief Write multiple bytes through SPI
 * @param SPIx: SPI instance
 * @param rxData: data buffer to receive data
 * @param dummyData: dummy byte to receive data from device
 * @param len: data array length
 */
void SPI_ReadMultiple(SPI_TypeDef *SPIx, uint8_t *rxData, uint8_t dummyData, uint16_t len);
