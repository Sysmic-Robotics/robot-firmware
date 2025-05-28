#ifndef DRIVE_TASK_H
#define DRIVE_TASK_H
#include "cmsis_os.h"
void DriveFunction(void const * argument);
void setSpeed(uint8_t *buffer, float *velocity, uint8_t *turn);
#endif // DRIVE_TASK_H
