#include <stdint.h>
#include "radio_task.h"
#include "system_globals.h"
#include "nrf24.h"
#include "board.h"
#include <string.h>
#include "kick_task.h"
#include "drive_task.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int fokk = 0;
void RadioFunction(void const * argument) {
    // --- Inicialización del módulo nRF24 ---
    nRF24_HW_Init(&nrf_device, &hspi1, GPIOG, GPIO_PIN_10, GPIOG, GPIO_PIN_9);
    nRF24_Init(&nrf_device);
    nRF24_SetAddr(&nrf_device, nRF24_PIPE0, rx_node_addr);
    nRF24_SetRFChannel(&nrf_device, nRF24L01_SYSMIC_CHANNEL);
    nRF24_SetRXPipe(&nrf_device, nRF24_PIPE0, nRF24_AA_OFF, 30);
    nRF24_SetPowerMode(&nrf_device, nRF24_PWR_UP);
    nRF24_SetOperationalMode(&nrf_device, nRF24_MODE_RX);
    nRF24_RX_ON(&nrf_device);
    memset(nrf_device.rx_data, 0, 32);
    nRF24_DisableAA(&nrf_device, nRF24_PIPETX);
    tx_node_addr[4] = Board_GetID();
    nRF24_SetAddr(&nrf_device, nRF24_PIPETX, tx_node_addr);
    nrf_config = nRF24_GetConfig(&nrf_device);

    // --- Bucle principal de la tarea ---
    for (;;) {
        // Espera evento de la cola (sin timeout)
        osMessageGet(nrf24CheckHandle, osWaitForever);

        // Actualiza estado del nRF24
        nrf_status = nRF24_GetStatus(&nrf_device);
        nrf_config = nRF24_GetConfig(&nrf_device);


        //memset(txBuffer, 0, 32);
        //updateBuffer(txBuffer);
        //txBuffer[28] = fokk++;
        //HAL_UART_Transmit(&huart5, txBuffer,32,HAL_MAX_DELAY);
        //osDelay(10);
        // Si hay datos recibidos
        if (nrf_status & nRF24_FLAG_RX_DR) {
            // --- Procesamiento de datos recibidos ---



            nRF24_ReadPayload(&nrf_device, nrf_device.rx_data, &rx_len);
            nRF24_FlushRX(&nrf_device);
            nRF24_ClearIRQFlagsRx(&nrf_device);

            setSpeed(nrf_device.rx_data + 5 * robot_id, speed, direction);
            dribbler_sel = getDribbler_speed(nrf_device.rx_data + 5 * robot_id);
            kick_sel = getKickerStatus(nrf_device.rx_data + 5 * robot_id);
            updateBuffer(txBuffer);

            // --- Cambio a modo TX y envío de datos ---
            //nRF24_RX_OFF(&nrf_device);
            //nRF24_SetOperationalMode(&nrf_device, nRF24_MODE_TX);
            //while (nrf_config & nRF24_CONFIG_PRIM_RX) {
            //    nrf_config = nRF24_GetConfig(&nrf_device);
            //}
            //nRF24_TxPacket(&nrf_device, txBuffer, 32);

            // --- Regreso a modo RX ---
            //nRF24_SetOperationalMode(&nrf_device, nRF24_MODE_RX);
            //while (!(nrf_config & nRF24_CONFIG_PRIM_RX)) {
            //    nrf_config = nRF24_GetConfig(&nrf_device);
            //}
            //nRF24_RX_ON(&nrf_device);
            //nRF24_ClearIRQFlags(&nrf_device);
        }
    }
}


void updateBuffer(uint8_t *buffer) {

	// Fill buffer with zeros if necessary
	memset(&buffer[0], 0, 32);
	float m0 = motor[0].measSpeed;
	float r0 = speed[0];
	float m1 = motor[1].measSpeed;
	float r1 = speed[1];
	float m2 = motor[2].measSpeed;
	float r2 = speed[2];
	float m3 = motor[3].measSpeed;
	float r3 = speed[3];



	//buffer[0] = 0xAA;
	memcpy(&buffer[0], &m0, sizeof(float));
	memcpy(&buffer[4], &r0, sizeof(float));
	memcpy(&buffer[8], &m1, sizeof(float));
	memcpy(&buffer[12], &r1, sizeof(float));
	memcpy(&buffer[16], &m2, sizeof(float));
	memcpy(&buffer[20], &r2, sizeof(float));
	memcpy(&buffer[24], &m3, sizeof(float));
	memcpy(&buffer[28], &r3, sizeof(float));
	//buffer[33] = 0x55;

}


void nRF24_TxPacket(nRF24_Handler_t *device, uint8_t* Buf, uint32_t Len)
{
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);

    for (uint32_t i = 0; i < Len; i++) {
        device->tx_data[i] = *Buf++;
    }

    nRF24_WritePayload(device, device->tx_data, Len);
    nRF24_CE_State(device, GPIO_PIN_SET);

    while (!(nrf_status & (nRF24_FLAG_TX_DS))) {
        nrf_status = nRF24_GetStatus(device);
    }

    nRF24_ClearIRQFlagsTx(device);
    nRF24_FlushTX(device);

    nRF24_CE_State(device, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_RESET);
}
