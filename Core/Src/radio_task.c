#include <stdint.h>
#include <stdbool.h>
#include "radio_task.h"
#include "system_globals.h"
#include "nrf24.h"
#include "board.h"
#include <string.h>
#include "kick_task.h"
#include "drive_task.h"
#include "FreeRTOS.h"
#include "task.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void RadioFunction(void *argument) {
    // --- Inicialización del módulo nRF24 (canal de comandos 0x6B) ---
    nRF24_HW_Init(&nrf_device, &hspi1, GPIOG, GPIO_PIN_10, GPIOG, GPIO_PIN_9);
    nRF24_Init(&nrf_device);
    nRF24_SetAddr(&nrf_device, nRF24_PIPE0, rx_node_addr);
    nRF24_SetRFChannel(&nrf_device, nRF24L01_SYSMIC_CHANNEL);
    nRF24_SetRXPipe(&nrf_device, nRF24_PIPE0, nRF24_AA_OFF, 30);
    // Pipe 1: corrección de cámara del Engine (18 bytes, dirección dedicada).
    nRF24_SetAddr(&nrf_device, nRF24_PIPE1, correction_node_addr);
    nRF24_SetRXPipe(&nrf_device, nRF24_PIPE1, nRF24_AA_OFF, 18);
    nRF24_SetPowerMode(&nrf_device, nRF24_PWR_UP);
    nRF24_SetOperationalMode(&nrf_device, nRF24_MODE_RX);
    nRF24_RX_ON(&nrf_device);
    memset(nrf_device.rx_data, 0, 32);
    nRF24_DisableAA(&nrf_device, nRF24_PIPETX);
    nRF24_SetAddr(&nrf_device, nRF24_PIPETX, tx_node_addr);
    nrf_config = nRF24_GetConfig(&nrf_device);

    // --- Bucle principal de la tarea ---
    for (;;) {
        // Espera evento de la cola (sin timeout)
        uint32_t _mq_msg;
        osMessageQueueGet(nrf24CheckHandle, &_mq_msg, NULL, osWaitForever);

        // Mutex: acceso exclusivo al radio SPI
        osMutexAcquire(radioMutexHandle, osWaitForever);

        // Actualiza estado del nRF24
        nrf_status = nRF24_GetStatus(&nrf_device);
        nrf_config = nRF24_GetConfig(&nrf_device);

        // Si hay datos recibidos
        if (nrf_status & nRF24_FLAG_RX_DR) {
            // --- Procesamiento de datos recibidos ---
            // Drain FIFO completo: lee todos los paquetes encolados, conserva solo el ultimo
            nRF24_RXResult pipe;
            do {
                pipe = nRF24_ReadPayload(&nrf_device, nrf_device.rx_data, &rx_len);
            } while (pipe < 6);
            nRF24_ClearIRQFlagsRx(&nrf_device);

            radio_last_rx_tick = HAL_GetTick();
            radio_ever_received = 1;

            taskENTER_CRITICAL();
            if (pipe == nRF24_PIPE1) {
                // Corrección de cámara del Engine (18 bytes, Pipe 1)
                processCameraCorrection(nrf_device.rx_data);
            } else {
                // Comando de radio (30 bytes, Pipe 0)
                processRadioCommand(nrf_device.rx_data + 5 * robot_id);
                dribbler_sel = getDribblerSpeedIndex(nrf_device.rx_data + 5 * robot_id);
                kick_sel = getKickTriggerFlag(nrf_device.rx_data + 5 * robot_id);
            }
            taskEXIT_CRITICAL();

            // La telemetría ahora la envía TelemetryTask en canal 0x6C
        }

        osMutexRelease(radioMutexHandle);
    }
}


static uint16_t telemetry_seq = 0;

void packTelemetryBuffer(uint8_t *buffer) {

    memset(&buffer[0], 0, 32);

    /* Byte 0: ID(3) + ball_possession(1) + kicker_charged(1) + motor_saturated(1) + reserved(2) */
    uint8_t id_bits = (robot_id << 5);
    uint8_t ball_bit = (ball_possession == 0x01 ? 1 : 0);
    uint8_t kicker_bit = (kick_flag == KICKER_CHARGED ? 1 : 0);
    uint8_t sat_bit = 0;
    for (int i = 0; i < 4; i++) {
        if (motor[i].voltage >= 4095.0f || motor[i].voltage <= -4095.0f) {
            sat_bit |= (1 << i);
        }
    }

    if (telemetry_calib_mode) {
        /* Calibration format (32 bytes):
         * [0]    ID(3)|ball(1)|kicker(1)|sat(4)  — same as normal
         * [1-4]  motor[0].measSpeed (f32)
         * [5-8]  motor[1].measSpeed (f32)
         * [9-12] motor[2].measSpeed (f32)
         * [13-16] motor[3].measSpeed (f32)
         * [17-20] vx_est (f32)
         * [21-24] vy_est (f32)
         * [25-28] omega_est (f32, ekf.x[6])
         * [29]    0xCB magic marker
         * [30-31] sequence counter (u16 LE)
         */
        buffer[0] = id_bits | (ball_bit << 0) | (kicker_bit << 1) | (sat_bit << 2);
        memcpy(&buffer[1],  &motor[0].measSpeed, sizeof(float));
        memcpy(&buffer[5],  &motor[1].measSpeed, sizeof(float));
        memcpy(&buffer[9],  &motor[2].measSpeed, sizeof(float));
        memcpy(&buffer[13], &motor[3].measSpeed, sizeof(float));
        memcpy(&buffer[17], &vx_est, sizeof(float));
        memcpy(&buffer[21], &vy_est, sizeof(float));
        memcpy(&buffer[25], &ekf.x[6], sizeof(float));
        buffer[29] = 0xCB;
        uint16_t seq = telemetry_seq++;
        memcpy(&buffer[30], &seq, sizeof(uint16_t));
        return;
    }

    buffer[0] = id_bits | (ball_bit << 0) | (kicker_bit << 1) | (sat_bit << 2);

    /* Bytes 1-2: ball_range (uint16 LE, mm) */
    memcpy(&buffer[1], &ball_range, sizeof(uint16_t));

    /* Bytes 3-6: x_est (float32, meters) */
    memcpy(&buffer[3], &ekf.x[0], sizeof(float));

    /* Bytes 7-10: y_est (float32, meters) */
    memcpy(&buffer[7], &ekf.x[1], sizeof(float));

    /* Bytes 11-14: vx_est (float32, m/s) */
    memcpy(&buffer[11], &ekf.x[4], sizeof(float));

    /* Bytes 15-18: vy_est (float32, m/s) */
    memcpy(&buffer[15], &ekf.x[5], sizeof(float));

    /* Bytes 19-22: theta_est (float32, rad) */
    memcpy(&buffer[19], &theta_est, sizeof(float));

    /* Bytes 23-26: omega_est (float32, rad/s) */
    memcpy(&buffer[23], &ekf.x[6], sizeof(float));

    /* Bytes 27-30: gyro_z_rads (float32, rad/s) — diagnostic */
    memcpy(&buffer[27], &gyro_z_rads, sizeof(float));

    /* Byte 31: angular acceleration limit (scaled uint8, 0.1 rad/s^2 resolution) */
    buffer[31] = (uint8_t)(robot_max_angular_acc * 10.0f);
}


void nRF24_TxPacket(nRF24_Handler_t *device, uint8_t* Buf, uint32_t Len)
{
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);

    for (uint32_t i = 0; i < Len; i++) {
        device->tx_data[i] = *Buf++;
    }

    nRF24_WritePayload(device, device->tx_data, Len);
    nRF24_CE_State(device, GPIO_PIN_SET);

    uint32_t timeout = HAL_GetTick();
    while (!(nrf_status & (nRF24_FLAG_TX_DS))) {
        nrf_status = nRF24_GetStatus(device);
        if (HAL_GetTick() - timeout > 10) break;
    }

    nRF24_ClearIRQFlagsTx(device);
    nRF24_FlushTX(device);

    nRF24_CE_State(device, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_RESET);
}

void Radio_SendPacket(nRF24_Handler_t *device, uint8_t *buffer, uint8_t len) {
    nRF24_RX_OFF(device);
    nRF24_SetOperationalMode(device, nRF24_MODE_TX);
    uint32_t timeout = HAL_GetTick();
    while (nrf_config & nRF24_CONFIG_PRIM_RX) {
        nrf_config = nRF24_GetConfig(device);
        if (HAL_GetTick() - timeout > 10) break;
    }
    nRF24_TxPacket(device, buffer, len);

    // --- Regreso a modo RX ---
    nRF24_SetOperationalMode(device, nRF24_MODE_RX);
    timeout = HAL_GetTick();
    while (!(nrf_config & nRF24_CONFIG_PRIM_RX)) {
        nrf_config = nRF24_GetConfig(device);
        if (HAL_GetTick() - timeout > 10) break;
    }
    nRF24_RX_ON(device);
    nRF24_ClearIRQFlagsTx(device);
}

static void Radio_SendTelemetryPacket(nRF24_Handler_t *device, uint8_t *buffer, uint8_t len) {
    nRF24_RX_OFF(device);
    nRF24_SetRFChannel(device, TELEMETRY_CHANNEL);
    nRF24_SetOperationalMode(device, nRF24_MODE_TX);
    uint32_t timeout = HAL_GetTick();
    while (nrf_config & nRF24_CONFIG_PRIM_RX) {
        nrf_config = nRF24_GetConfig(device);
        if (HAL_GetTick() - timeout > 10) break;
    }
    nRF24_TxPacket(device, buffer, len);

    nRF24_SetOperationalMode(device, nRF24_MODE_RX);
    nRF24_SetRFChannel(device, nRF24L01_SYSMIC_CHANNEL);
    timeout = HAL_GetTick();
    while (!(nrf_config & nRF24_CONFIG_PRIM_RX)) {
        nrf_config = nRF24_GetConfig(device);
        if (HAL_GetTick() - timeout > 10) break;
    }
    nRF24_RX_ON(device);
    nRF24_ClearIRQFlagsTx(device);
}

void TelemetryFunction(void *argument) {
    osDelay(robot_id * 2 + 1);

    for (;;) {
        osDelay(TELEMETRY_PERIOD_MS);

        osMutexAcquire(radioMutexHandle, osWaitForever);

        packTelemetryBuffer(txBuffer);
        Radio_SendTelemetryPacket(&nrf_device, txBuffer, 32);

        osMutexRelease(radioMutexHandle);
    }
}
