#ifndef RADIO_TASK_H
#define RADIO_TASK_H
#include <stdint.h>
#include "nrf24.h"
#include "cmsis_os.h"
void RadioFunction(void *argument);
void TelemetryFunction(void *argument);
void packTelemetryBuffer(uint8_t *buffer);
void nRF24_TxPacket(nRF24_Handler_t *device, uint8_t* Buf, uint32_t Len);
void Radio_SendPacket(nRF24_Handler_t *device, uint8_t *buffer, uint8_t len);
#endif // RADIO_TASK_H
