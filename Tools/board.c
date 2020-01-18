#include "board.h"

void Board_LedSet(GPIO_TypeDef* Led_GPIO, uint16_t Led_Pin)
{
	/* Include HAL or LL function */
	HAL_GPIO_WritePin(Led_GPIO, Led_Pin, GPIO_PIN_SET);
}

void Board_LedReset(GPIO_TypeDef* Led_GPIO, uint16_t Led_Pin)
{
	/* Include HAL or LL function */
	HAL_GPIO_WritePin(Led_GPIO, Led_Pin, GPIO_PIN_RESET);
}

void Board_LedToggle(GPIO_TypeDef* Led_GPIO, uint16_t Led_Pin)
{
	/* Include HAL or LL function */
	HAL_GPIO_TogglePin(Led_GPIO, Led_Pin);
}

uint16_t Board_GetID()
{
	return (HAL_GPIO_ReadPin(DS_GPIO_1, DS_PIN_1)|
	HAL_GPIO_ReadPin(DS_GPIO_2, DS_PIN_2) << 1|
	HAL_GPIO_ReadPin(DS_GPIO_3, DS_PIN_3) << 2|
	HAL_GPIO_ReadPin(DS_GPIO_4, DS_PIN_4) << 3|
	HAL_GPIO_ReadPin(DS_GPIO_5, DS_PIN_5) << 4|
	HAL_GPIO_ReadPin(DS_GPIO_6, DS_PIN_6) << 5);
}
