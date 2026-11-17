#ifndef KICK_TASK_H
#define KICK_TASK_H
#include "cmsis_os.h"


void KickFunction(void *argument);
uint8_t getDribblerSpeedIndex(uint8_t *buffer);
uint8_t getKickTriggerFlag(uint8_t *buffer);

#endif // KICK_TASK_H
