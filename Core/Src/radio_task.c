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

            //

            if (ball_posession == 0x00) {
                if(ball_posession_last == 0x01) {
                    // Si no se pierde la pelota, se manda mensaje
                    ball_posession_last = 0x00;
                    updateBuffer(txBuffer);
                    Radio_SendPacket(&nrf_device, txBuffer, 32);
                }
            } else {
                // si aun se tiene la pelota, no se actualiza el buffer   
            }
        
        } else {
            // Si no se reciben datos no hace nada
        }
    }
}


void updateBuffer(uint8_t *buffer) {

    // Fill buffer with zeros if necessary
    memset(&buffer[0], 0, 32);

    // Set first byte: bits 0-2 = robot_id (3 bits), bit 3 = ball_possession (1 bit), bits 4-7 = 0
    uint8_t id_bits = (robot_id << 3); // 3 bits for robot_id
    uint8_t ball_bit = (ball_posession == 0x01 ? 1 : 0); // 1 bit for ball_posession at bit 3
    buffer[0] = id_bits | ball_bit;

    //float m0 = motor[0].measSpeed;
    //float m1 = motor[1].measSpeed;
    //float m2 = motor[2].measSpeed;
    //float m3 = motor[3].measSpeed;

    //memcpy(&buffer[1+4*0], &m0, sizeof(float));
    //memcpy(&buffer[1+4*1], &m1, sizeof(float));
    //memcpy(&buffer[1+4*2], &m2, sizeof(float));
    //memcpy(&buffer[1+4*3], &m3, sizeof(float));

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

// Cambia a modo TX, envía un paquete y regresa a modo RX
void Radio_SendPacket(nRF24_Handler_t *device, uint8_t *txBuffer, uint32_t len) {
    extern uint8_t nrf_config;
    nRF24_RX_OFF(device);
    nRF24_SetOperationalMode(device, nRF24_MODE_TX);
    while (nrf_config & nRF24_CONFIG_PRIM_RX) {
        nrf_config = nRF24_GetConfig(device);
    }
    nRF24_TxPacket(device, txBuffer, len);

    nRF24_SetOperationalMode(device, nRF24_MODE_RX);
    while (!(nrf_config & nRF24_CONFIG_PRIM_RX)) {
        nrf_config = nRF24_GetConfig(device);
    }
    nRF24_RX_ON(device);
    nRF24_ClearIRQFlags(device);
}
