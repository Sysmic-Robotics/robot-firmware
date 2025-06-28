#include <stdint.h>
#include "radio_task.h"
#include "system_globals.h"
#include "nrf24.h"
#include "board.h"
#include <string.h>
#include "kick_task.h"
#include "drive_task.h"

void RadioFunction(void const * argument) {

    // --- Bucle principal de la tarea ---
    for (;;) {
        // Espera evento de la cola (sin timeout)
        osMessageGet(nrf24CheckHandle, osWaitForever);

        // Actualiza estado del nRF24
        nrf_status = nRF24_GetStatus(&nrf_device);
        nrf_config = nRF24_GetConfig(&nrf_device);

        // Si hay datos recibidos
        if (nrf_status & nRF24_FLAG_RX_DR) {
            // --- Procesamiento de datos recibidos ---
            readAndFlush(&nrf_device);

            setSpeed(nrf_device.rx_data + 5 * robot_id, speed, direction);
            dribbler_sel = getDribbler_speed(nrf_device.rx_data + 5 * robot_id);
            kick_sel = getKickerStatus(nrf_device.rx_data + 5 * robot_id);
            updateBuffer_MotorVels(txBuffer);

            // --- Cambio a modo TX y envío de datos ---
            nRF24_RX_OFF(&nrf_device);
            modeTX(&nrf_device);

            // Envío del paquete de datos
            sendTxPacket(&nrf_device, txBuffer, 32);

            // --- Regreso a modo RX ---

            modeRX(&nrf_device);
        }
    }
}

void RadioMenu(void const * argument) {
    // --- Inicialización del módulo nRF24 ---
    radioInit();
    
    //  Bucle principal del menu
    for (;;) {
        // Actualiza estado del nRF24
        nrf_status = nRF24_GetStatus(&nrf_device);
        nrf_config = nRF24_GetConfig(&nrf_device);

        // Si hay datos recibidos
        if (nrf_status & nRF24_FLAG_RX_DR) {
            //  Procesamiento de datos recibidos 
            readAndFlush(&nrf_device);

            //comprueba si menasje es para este robot
            if (nrf_device.rx_data[0] == robot_id) { 
                //menu de opciones
                switch (nrf_device.rx_data[1]) {
                    case 'P': //PLAY Modo de jugar, inicia el sistema operativo
                        break;
                        break;

                    case 'D': // Modo de dribbler
                        break;

                    case 'K': // Modo de kicker
                        break;

                    default:
                        break;
                }

            }else{
                setSpeed(zeroVector, speed, direction);
            }
            

            // Actualiza el buffer de transmisión
            updateBuffer_MotorVels(txBuffer);

            // --- Cambio a modo TX y envío de datos ---
            
            modeTX(&nrf_device);

            // Envío del paquete de datos
            sendTxPacket(&nrf_device, txBuffer, 32);

            // --- Regreso a modo RX ---

            modeRX(&nrf_device);
        }
    }
}

void radioInit(void) {
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
}

void updateBuffer_MotorVels(uint8_t *buffer) {

	// Fill buffer with zeros if necessary
	memset(&buffer[0], 0, 32);

    // Copy the motor speeds to the txBuffer
    memcpy(&buffer[0], (const void *)&motor[0].measSpeed, sizeof(float));
    buffer[4] = '\n';
    memcpy(&buffer[5], (const void *)&motor[1].measSpeed, sizeof(float));
    buffer[9] = '\n';
    memcpy(&buffer[10], (const void *)&motor[2].measSpeed, sizeof(float));
    buffer[14] = '\n';
    memcpy(&buffer[15], (const void *)&motor[3].measSpeed, sizeof(float));
}


void sendTxPacket(nRF24_Handler_t *device, uint8_t* Buf, uint32_t Len)
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

void readAndFlush(nRF24_Handler_t *device) {
    // Read payload
    nRF24_ReadPayload(device, device->rx_data, &rx_len);
    
    // Flush RX buffer
    nRF24_FlushRX(device);
    
    // Clear IRQ flags
    nRF24_ClearIRQFlagsRx(device);
}

void modeTX(nRF24_Handler_t *device) {
    nRF24_RX_OFF(device);
    nRF24_SetOperationalMode(device, nRF24_MODE_TX);
    while (nrf_config & nRF24_CONFIG_PRIM_RX) {
        nrf_config = nRF24_GetConfig(device);
    }
}

void modeRX(nRF24_Handler_t *device) {
    nRF24_SetOperationalMode(device, nRF24_MODE_RX);
    while (!(nrf_config & nRF24_CONFIG_PRIM_RX)) {
        nrf_config = nRF24_GetConfig(device);
    }
    nRF24_RX_ON(device);
    nRF24_ClearIRQFlags(device);
}

