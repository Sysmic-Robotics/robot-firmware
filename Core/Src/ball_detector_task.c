#include "ball_detector_task.h"
#include "system_globals.h"
#include "vl6180x.h"
#include "radio_task.h"
#include <string.h>

void BallDetectorFunction(void const * argument) {
    VL6180X_Init(&range_sensor, &hi2c3, VL6180X_DEFAULT_I2C_ADDR);
    ball_range = VL6180X_ReadRange(&range_sensor);
    memset(ball_meas_set, ball_range, 10);
    for (;;) {
        ball_meas_set[0] = VL6180X_ReadRange(&range_sensor);
        ball_accum = ball_meas_set[0];
        for (uint8_t i = 9; i > 0; i--) {
            ball_accum += ball_meas_set[i];
            ball_meas_set[i] = ball_meas_set[i - 1];
        }
        ball_range = ball_accum / 10;
        if (ball_range < VL6180X_THRESHOLD) {
            ball_posession = 0x01;
            
            updateBuffer(txBuffer);
            Radio_SendPacket(&nrf_device, txBuffer, 32);
        } else {
            ball_posession = 0x00;
        }
        osDelay(1);
    }
}
