#include "system_globals.h"
#include "kick_task.h"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"

void KickFunction(void const * argument)
{
    for(;;)
    {
        // --- Cargar el capacitor del kicker ---
        HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_4, GPIO_PIN_SET);
        osDelay(4000);
        HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_4, GPIO_PIN_RESET);

        // --- Marcar kicker como cargado ---
        osMutexWait(kickFlagHandle, osWaitForever);
        kick_flag = KICKER_CHARGED;
        osMutexRelease(kickFlagHandle);

        // --- Esperar evento de disparo ---
        osEvent kicker_side = osMessageGet(kickQueueHandle, osWaitForever);

        // --- Activar el kicker (disparo) ---
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);
        osDelay(10);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

        // --- Marcar kicker como descargado ---
        osMutexWait(kickFlagHandle, osWaitForever);
        kick_flag = KICKER_DISCHARGED;
        osMutexRelease(kickFlagHandle);

        // --- Contador de disparos ---
        kick_count++;
    }
}

uint8_t getDribbler_speed(uint8_t *buffer)
{
	/* Extract info from data packet */
	uint8_t dribbler_vel = (buffer[0] & 0x1C) >> 2;

	return dribbler_vel;
}

uint8_t getKickerStatus(uint8_t *buffer)
{
	/* Extract info from data packet */
	uint8_t kick_stat = buffer[0] & 0x02 ? 0x01 : 0x00;

	return kick_stat;
}