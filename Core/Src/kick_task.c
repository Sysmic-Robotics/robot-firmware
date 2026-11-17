#include "system_globals.h"
#include "kick_task.h"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"

static void charge_capacitor(void)
{
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_4, GPIO_PIN_SET);
    osDelay(4000);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_4, GPIO_PIN_RESET);

    osMutexAcquire(kickFlagHandle, osWaitForever);
    kick_flag = KICKER_CHARGED;
    osMutexRelease(kickFlagHandle);
}

static void fire_kicker(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);
    osDelay(10);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

    osMutexAcquire(kickFlagHandle, osWaitForever);
    kick_flag = KICKER_DISCHARGED;
    osMutexRelease(kickFlagHandle);

    kick_count++;
}

void KickFunction(void *argument)
{
    charge_capacitor();

    for(;;)
    {
        while (!kick_sel) {
            osDelay(10);
        }

        uint32_t _mq_msg;
        osStatus_t status = osMessageQueueGet(kickQueueHandle, &_mq_msg, NULL, 5000);
        if (status != osOK) continue;

        fire_kicker();
        charge_capacitor();
    }
}

uint8_t getDribblerSpeedIndex(uint8_t *buffer)
{
	/* Extract info from data packet (bits [4:3]) */
	uint8_t dribbler_vel = (buffer[0] & 0x18) >> 3;
	if (dribbler_vel > 3) dribbler_vel = 3;

	return dribbler_vel;
}

uint8_t getKickTriggerFlag(uint8_t *buffer)
{
	/* Extract info from data packet */
	uint8_t kick_stat = buffer[0] & 0x02 ? 0x01 : 0x00;

	return kick_stat;
}