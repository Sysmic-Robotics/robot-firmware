#ifndef RADIO_TASK_H
#define RADIO_TASK_H
#include <stdint.h>
#include "nrf24.h"
#include "cmsis_os.h"

void RadioFunction(void const * argument);
void RadioMenu(void const * argument);
void radioInit(void);
void updateBuffer_MotorVels(uint8_t *buffer);
void nRF24_TxPacket(nRF24_Handler_t *device, uint8_t* Buf, uint32_t Len);
void readAndFlush(nRF24_Handler_t *device);
void modeTX(nRF24_Handler_t *device);
void modeRX(nRF24_Handler_t *device);
#endif // RADIO_TASK_H
