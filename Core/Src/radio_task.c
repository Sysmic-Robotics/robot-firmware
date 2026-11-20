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

int fokk = 0;

#define RADIO_RX_CONFIG (nRF24_CONFIG_PWR_UP | nRF24_CONFIG_PRIM_RX)

static void Radio_Configure(void)
{
    nRF24_RX_OFF(&nrf_device);
    nRF24_Init(&nrf_device);
    nRF24_SetCRCScheme(&nrf_device, nRF24_CRC_2byte); /* Debe coincidir con la base station. */
    nRF24_SetAddr(&nrf_device, nRF24_PIPE0, rx_node_addr);
    nRF24_SetRFChannel(&nrf_device, nRF24L01_SYSMIC_CHANNEL);
    nRF24_SetRXPipe(&nrf_device, nRF24_PIPE0, nRF24_AA_OFF, 30);
    nRF24_SetOperationalMode(&nrf_device, nRF24_MODE_RX);
    nRF24_SetAddr(&nrf_device, nRF24_PIPETX, tx_node_addr);
    nRF24_SetPowerMode(&nrf_device, nRF24_PWR_UP);
    osDelay(3); /* nRF24 power-down to standby: allow at least 1.5 ms. */
    nRF24_RX_ON(&nrf_device);
}

void RadioFunction(void const * argument) {
    // --- Inicialización del módulo nRF24 ---
    nRF24_HW_Init(&nrf_device, &hspi1, GPIOG, GPIO_PIN_10, GPIOG, GPIO_PIN_9);
    memset(nrf_device.rx_data, 0, 32);
    tx_node_addr[4] = Board_GetID();
    Radio_Configure();

    // --- Bucle principal de la tarea ---
    for (;;) {
        // Espera evento de la cola (sin timeout)
        osMessageGet(nrf24CheckHandle, osWaitForever);

        // Actualiza estado del nRF24
        nrf_status = nRF24_GetStatus(&nrf_device);
        nrf_config = nRF24_GetConfig(&nrf_device);

        // Una caida de tension reinicia el nRF24 (queda apagado y sin configuracion)
        if ((nrf_config & RADIO_RX_CONFIG) != RADIO_RX_CONFIG) {
            Radio_Configure();
            continue;
        }

        // Si hay datos recibidos
        if (nrf_status & nRF24_FLAG_RX_DR) {
            // --- Procesamiento de datos recibidos ---

            nRF24_RXResult result = nRF24_ReadPayload(&nrf_device, nrf_device.rx_data, &rx_len);
            nRF24_FlushRX(&nrf_device);
            nRF24_ClearIRQFlagsRx(&nrf_device);
            /* Un ID fuera de 0..5 leeria fuera del paquete: el robot no acepta comandos. */
            if (result != nRF24_RX_PIPE0 || rx_len != 30U || robot_id >= ROBOT_COUNT) continue;

            taskENTER_CRITICAL();
            setSpeed(nrf_device.rx_data + 5 * robot_id, speed, direction);
            dribbler_sel = getDribbler_speed(nrf_device.rx_data + 5 * robot_id);
            kick_sel = getKickerStatus(nrf_device.rx_data + 5 * robot_id);
            radio_last_command_ms = HAL_GetTick();
            radio_command_received = 1;
            taskEXIT_CRITICAL();

            /* Turnos de telemetria separados 2 ms: absorbe el jitter de 1 ms del sondeo
               y el robot 5 termina (~13.5 ms) antes del siguiente comando a 60 Hz. */
            osDelay(RADIO_TELEMETRY_SLOT_MS * (robot_id + 1U));
            updateBuffer(txBuffer); // 0 usa ball_posession
            Radio_SendPacket(&nrf_device, txBuffer, 32);
        
        }
    }
}


/* Telemetria v2 (32 bytes). La base station la reenvia sin cambios.
   [0]     ID<<5 | balon            [1..2] rango del sensor (mm, uint16 LE)
   [3]     0xA5 (marca v2)          [4]    bit0 comando valido, bit1 motores activos
   [5]     muestras de 1 ms promediadas (saturado a 255)
   [6..13]  velocidad medida promedio por rueda, int16 LE, rad/s * 50
   [14..21] referencia por rueda, int16 LE, rad/s * 50
   [22..29] salida aplicada al DAC por rueda, int16 LE (codigo con signo)
   [30]    contador de paquetes     [31]   XOR de los bytes 0..30 */
#define TELEMETRY_V2_MARK   0xA5U
#define TELEMETRY_SPEED_SCALE 50.0f

static void put_i16(uint8_t *dst, float value)
{
    float r = value >= 0.0f ? value + 0.5f : value - 0.5f;
    if (r > 32767.0f) r = 32767.0f;
    else if (r < -32768.0f) r = -32768.0f;
    int16_t v = (int16_t)r;
    dst[0] = (uint8_t)((uint16_t)v & 0xFFU);
    dst[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFU);
}

void updateBuffer(uint8_t *buffer) {
    static uint8_t sequence = 0;
    float meas[4], ref[4], out[4];
    uint16_t samples;
    uint8_t active;

    memset(&buffer[0], 0, 32);

    // Byte 0: bits 5-7 = robot_id, bit 0 = posesion de balon
    uint8_t id_bits = (robot_id << 5);
    uint8_t ball_bit = (ball_posession == 0x01 ? 1 : 0);
    buffer[0] = id_bits | ball_bit;
    memcpy(&buffer[1], &ball_range, sizeof(uint16_t));

    taskENTER_CRITICAL();
    samples = motor_telemetry.samples;
    active = motor_telemetry.active;
    for (uint8_t i = 0; i < 4; ++i) {
        meas[i] = samples ? motor_telemetry.meas_sum[i] / (float)samples : 0.0f;
        motor_telemetry.meas_sum[i] = 0.0f;
        ref[i] = samples ? motor_telemetry.excitation_sum[i] / (float)samples : 0.0f;
        out[i] = samples ? motor_telemetry.dac_sum[i] / (float)samples : 0.0f;
        motor_telemetry.excitation_sum[i] = 0.0f;
        motor_telemetry.dac_sum[i] = 0.0f;
    }
    motor_telemetry.samples = 0;
    taskEXIT_CRITICAL();

    buffer[3] = TELEMETRY_V2_MARK;
    buffer[4] = (radio_command_received ? 0x01U : 0x00U) | (active ? 0x02U : 0x00U);
    buffer[5] = samples > 255U ? 255U : (uint8_t)samples;
    for (uint8_t i = 0; i < 4; ++i) {
        put_i16(&buffer[6 + 2 * i], meas[i] * TELEMETRY_SPEED_SCALE);
        put_i16(&buffer[14 + 2 * i], ref[i] * TELEMETRY_SPEED_SCALE);
        put_i16(&buffer[22 + 2 * i], out[i]);
    }
    buffer[30] = sequence++;
    uint8_t check = 0;
    for (uint8_t i = 0; i < 31; ++i) check ^= buffer[i];
    buffer[31] = check;
}


void nRF24_TxPacket(nRF24_Handler_t *device, uint8_t* Buf, uint32_t Len)
{
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);

    for (uint32_t i = 0; i < Len; i++) {
        device->tx_data[i] = *Buf++;
    }

    nRF24_WritePayload(device, device->tx_data, Len);
    nRF24_CE_State(device, GPIO_PIN_SET);

    /* Espera acotada: un nRF24 reiniciado nunca levanta TX_DS. */
    uint32_t start = HAL_GetTick();
    do {
        nrf_status = nRF24_GetStatus(device);
    } while (!(nrf_status & nRF24_FLAG_TX_DS) && (HAL_GetTick() - start) < RADIO_TX_TIMEOUT_MS);

    nRF24_ClearIRQFlagsTx(device);
    nRF24_FlushTX(device);

    nRF24_CE_State(device, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_RESET);
}

void Radio_SendPacket(nRF24_Handler_t *device, uint8_t *buffer, uint8_t len) {
    nRF24_RX_OFF(device);
    nRF24_SetOperationalMode(device, nRF24_MODE_TX);
    nRF24_TxPacket(device, buffer, len);

    // --- Regreso a modo RX ---
    /* No se limpia RX_DR: un comando recibido durante el turno se procesa en la siguiente vuelta. */
    nRF24_SetOperationalMode(device, nRF24_MODE_RX);
    nRF24_RX_ON(device);
}
