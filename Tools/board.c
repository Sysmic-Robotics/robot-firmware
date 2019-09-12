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
