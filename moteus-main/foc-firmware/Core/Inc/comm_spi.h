#ifndef COMM_SPI_H
#define COMM_SPI_H

#include <stdint.h>

/* ======================================================================== *
 *  SPI2 Slave Protocol — Phoenix <-> Master Board                          *
 *                                                                          *
 *  Frame: 5 words of 16-bit each, exchanged in a single SPI transaction   *
 *                                                                          *
 *  Master -> Phoenix (TX):                                                 *
 *    Word 0: [CMD_ID:4][SEQ_NUM:4][MODE:8]                                 *
 *    Word 1: Position setpoint  (int16, scaled units)                      *
 *    Word 2: Velocity setpoint  (int16, scaled units)                      *
 *    Word 3: Torque setpoint    (int16, scaled units)                      *
 *    Word 4: Checksum (XOR of words 0-3)                                   *
 *                                                                          *
 *  Phoenix -> Master (RX, simultaneous):                                   *
 *    Word 0: [STATE:4][FAULT:4][MODE:8]                                    *
 *    Word 1: Actual position    (int16)                                    *
 *    Word 2: Actual velocity    (int16)                                    *
 *    Word 3: Actual Iq current  (int16)                                    *
 *    Word 4: Bus voltage        (uint16, mV)                               *
 * ======================================================================== */

#define COMM_FRAME_SIZE             5

/* Header bitfields */
#define COMM_HEADER_CMD_SHIFT       12
#define COMM_HEADER_CMD_MASK        0xF000
#define COMM_HEADER_SEQ_SHIFT       8
#define COMM_HEADER_SEQ_MASK        0x0F00
#define COMM_HEADER_MODE_MASK       0x00FF

/* Command IDs */
#define COMM_CMD_NOP                0
#define COMM_CMD_SET_SETPOINT       1
#define COMM_CMD_EMERGENCY_STOP     2
#define COMM_CMD_CONFIGURE          3
#define COMM_CMD_CALIBRATE          4

/* Control modes */
#define COMM_MODE_STOP              0
#define COMM_MODE_CURRENT           1
#define COMM_MODE_TORQUE            2
#define COMM_MODE_VELOCITY          3
#define COMM_MODE_POSITION          4
#define COMM_MODE_FAULT             0xFF

/* Status codes */
#define COMM_STATE_INIT             0
#define COMM_STATE_IDLE             1
#define COMM_STATE_READY            2
#define COMM_STATE_RUNNING          3
#define COMM_STATE_FAULT            4

/* Fault flags */
#define COMM_FAULT_OVERCURRENT      (1U << 0)
#define COMM_FAULT_OVERVOLTAGE      (1U << 1)
#define COMM_FAULT_UNDERVOLTAGE     (1U << 2)
#define COMM_FAULT_OVERTEMP         (1U << 3)
#define COMM_FAULT_ENCODER          (1U << 4)
#define COMM_FAULT_STALL            (1U << 5)
#define COMM_FAULT_COMMS            (1U << 6)
#define COMM_FAULT_GENERAL          (1U << 7)

/* Parsed setpoint from SPI frame */
typedef struct {
    uint8_t  cmd_id;
    uint8_t  mode;
    int16_t  position;
    int16_t  velocity;
    int16_t  torque;
} COMM_Setpoint;

/* Public API ------------------------------------------------------------- */

/* Initialise SPI2 slave: start the first interrupt-driven transfer.
 * Must be called once after MX_SPI2_Init(). */
void    COMM_Init(void);

/* Get the latest setpoint received from master (non-blocking).
 * Returns 1 if a new setpoint was available (copied into *sp), else 0. */
uint8_t COMM_GetSetpoint(COMM_Setpoint *sp);

/* Update telemetry data that will be sent to master on the next SPI frame. */
void    COMM_SetTelemetry(int16_t position, int16_t velocity,
                          int16_t current, uint16_t vbus,
                          uint8_t state, uint8_t fault);

/* Returns 1 if the communication module is active. */
uint8_t COMM_IsActive(void);

#endif /* COMM_SPI_H */
