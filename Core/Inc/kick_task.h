#ifndef KICK_TASK_H
#define KICK_TASK_H
#include "cmsis_os.h"


void KickFunction(void const * argument);
uint8_t getDribbler_speed(uint8_t *buffer);
uint8_t getKickerStatus(uint8_t *buffer);

#endif // KICK_TASK_H
