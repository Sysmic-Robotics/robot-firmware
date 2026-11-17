#include "comm_spi.h"
#include "spi.h"          /* hspi2 */
#include "main.h"         /* __disable_irq, __enable_irq */
#include <string.h>

/* ----------------------------------------------------------------------- */
/*  Internal state                                                         */
/* ----------------------------------------------------------------------- */

/* Double-buffers for the ongoing SPI interrupt transfer.
 * tx_buf is filled with telemetry before each transfer starts.
 * rx_buf receives the master command during the transfer. */
static uint16_t tx_buf[COMM_FRAME_SIZE];
static uint16_t rx_buf[COMM_FRAME_SIZE];

/* Parsed setpoints — written by the SPI callback (ISR context),
 * consumed by the main loop.  All fields volatile because they
 * are shared between contexts. */
static volatile uint8_t  cmd_valid;
static volatile uint8_t  cmd_pending_id;
static volatile uint8_t  cmd_pending_mode;
static volatile int16_t  cmd_pending_pos;
static volatile int16_t  cmd_pending_vel;
static volatile int16_t  cmd_pending_trq;

/* Telemetry — written by the main loop, read by the SPI callback
 * to fill tx_buf. */
static volatile int16_t  telem_position;
static volatile int16_t  telem_velocity;
static volatile int16_t  telem_current;
static volatile uint16_t telem_vbus;
static volatile uint8_t  telem_state;
static volatile uint8_t  telem_fault;

static uint8_t comm_initialised;

/* ----------------------------------------------------------------------- */
/*  Helpers                                                                */
/* ----------------------------------------------------------------------- */

static uint16_t calc_checksum(const uint16_t *words, int len)
{
    uint16_t cs = 0;
    for (int i = 0; i < len; i++) {
        cs ^= words[i];
    }
    return cs;
}

/* Start an interrupt-driven full-duplex SPI transfer.  This must
 * always be active in slave mode — the master controls the clock,
 * so we must have a receive buffer ready at all times. */
static void start_transfer(void)
{
    /* Pack telemetry into TX buffer */
    tx_buf[0] = ((uint16_t)telem_state << 8) | telem_fault;
    tx_buf[1] = (uint16_t)telem_position;
    tx_buf[2] = (uint16_t)telem_velocity;
    tx_buf[3] = (uint16_t)telem_current;
    tx_buf[4] = telem_vbus;

    HAL_SPI_TransmitReceive_IT(&hspi2,
                               (uint8_t *)tx_buf,
                               (uint8_t *)rx_buf,
                               COMM_FRAME_SIZE);
}

/* ----------------------------------------------------------------------- */
/*  Public API                                                             */
/* ----------------------------------------------------------------------- */

void COMM_Init(void)
{
    memset((void *)rx_buf, 0, sizeof(rx_buf));
    memset((void *)tx_buf, 0, sizeof(tx_buf));
    cmd_valid = 0;

    start_transfer();
    comm_initialised = 1;
}

uint8_t COMM_GetSetpoint(COMM_Setpoint *sp)
{
    if (!cmd_valid) return 0;

    /* Disable interrupts for atomic multi-field read */
    __disable_irq();
    sp->cmd_id   = cmd_pending_id;
    sp->mode     = cmd_pending_mode;
    sp->position = cmd_pending_pos;
    sp->velocity = cmd_pending_vel;
    sp->torque   = cmd_pending_trq;
    cmd_valid    = 0;
    __enable_irq();

    return 1;
}

void COMM_SetTelemetry(int16_t position, int16_t velocity,
                       int16_t current, uint16_t vbus,
                       uint8_t state, uint8_t fault)
{
    telem_position = position;
    telem_velocity = velocity;
    telem_current  = current;
    telem_vbus     = vbus;
    telem_state    = state;
    telem_fault    = fault;
}

uint8_t COMM_IsActive(void)
{
    return comm_initialised;
}

/* ----------------------------------------------------------------------- */
/*  HAL callback overrides  (called from SPI2_IRQHandler)                  */
/* ----------------------------------------------------------------------- */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI2) return;

    /* Validate checksum */
    uint16_t expected = calc_checksum(rx_buf, COMM_FRAME_SIZE - 1);
    uint16_t received = rx_buf[COMM_FRAME_SIZE - 1];

    if (expected == received) {
        uint16_t hdr = rx_buf[0];
        uint8_t  cmd = (hdr >> COMM_HEADER_CMD_SHIFT) & 0x0F;

        if (cmd == COMM_CMD_EMERGENCY_STOP) {
            cmd_pending_id   = COMM_CMD_SET_SETPOINT;
            cmd_pending_mode = COMM_MODE_STOP;
            cmd_pending_pos  = 0;
            cmd_pending_vel  = 0;
            cmd_pending_trq  = 0;
        } else {
            cmd_pending_id   = cmd;
            cmd_pending_mode = hdr & COMM_HEADER_MODE_MASK;
            cmd_pending_pos  = (int16_t)rx_buf[1];
            cmd_pending_vel  = (int16_t)rx_buf[2];
            cmd_pending_trq  = (int16_t)rx_buf[3];
        }
        cmd_valid = 1;
    } /* else: checksum mismatch — silently drop the frame */

    /* Keep the receive pipeline primed for the next transaction */
    start_transfer();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI2) return;

    /* SPI slave overrun can happen if the master clocks faster than
     * the previous transfer was restarted.  Recover the peripheral
     * and force the HAL state back to READY before restarting. */
    hspi->State = HAL_SPI_STATE_READY;
    __HAL_SPI_DISABLE(hspi);
    (void)hspi->Instance->SR;   /* clear OVR flag by reading SR then DR */
    (void)hspi->Instance->DR;
    __HAL_SPI_ENABLE(hspi);

    start_transfer();
}
