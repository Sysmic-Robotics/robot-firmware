#ifndef DRIVE_TASK_H
#define DRIVE_TASK_H
#include "cmsis_os.h"
void DriveFunction(void const * argument);
void setSpeed(uint8_t *buffer, float *velocity, uint8_t *turn);
// Interfaz para buffer de encoder (sólo archivo drive_task.c)
void EncoderBuf_Push(uint8_t motorIdx, float sample);
float EncoderBuf_GetAvg(uint8_t motorIdx);
#endif // DRIVE_TASK_H
