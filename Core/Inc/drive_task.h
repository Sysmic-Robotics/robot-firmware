#ifndef DRIVE_TASK_H
#define DRIVE_TASK_H
#include "cmsis_os.h"
void DriveFunction(void *argument);
void processRadioCommand(uint8_t *buffer);
void processCameraCorrection(uint8_t *buffer);
#endif // DRIVE_TASK_H
