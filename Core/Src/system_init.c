#include <stdint.h>
#include "stm32f7xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include <math.h>

// =============================================================================
// MX_Init_Custom()
// Corrige los valores que CubeMX no puede generar directamente:
//   - Timings I2C  (0x6000030D → 0x1FF01428 para I2C1/2/4)
//   - SPI DataSize (4→8 bit), register tweaks (FRXTH, SSI, CRCL, SPE)
//   - Encoder filters (0→8)
//   - SPI2 force-reset + modo SPI 3 (CPOL=1, CPHA=1) para ADXL312
// =============================================================================
void MX_Init_Custom(void)
{
    // ---- I2C Timing Overrides -----------------------------------------------
    // I2C1 → 0x1FF01428 (CubeMX genera 0x6000030D)0x1FF01428
    HAL_I2C_DeInit(&hi2c1);
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x6000030D;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();

    // I2C2 → 0x1FF01428
    HAL_I2C_DeInit(&hi2c2);
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x6000030D;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) Error_Handler();

    // I2C4 → 0x1FF01428
    HAL_I2C_DeInit(&hi2c4);
    hi2c4.Instance = I2C4;
    hi2c4.Init.Timing = 0x6000030D;
    hi2c4.Init.OwnAddress1 = 0;
    hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c4.Init.OwnAddress2 = 0;
    hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c4) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK) Error_Handler();

    // ---- SPI Overrides: DataSize 4→8 bit + register tweaks -----------------
    // SPI1
    HAL_SPI_DeInit(&hspi1);
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_SPI_Init(&hspi1);
    SET_BIT(hspi1.Instance->CR2, SPI_RXFIFO_THRESHOLD_QF);
    SET_BIT(hspi1.Instance->CR1, SPI_CR1_SSI);
    hspi1.Instance->CR1 &= ~SPI_CR1_CRCL;
    hspi1.Instance->CR1 |= SPI_CR1_SPE;

    // SPI2 (force-reset + Modo SPI 3: CPOL=1, CPHA=1 para ADXL312)
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    HAL_SPI_DeInit(&hspi2);
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    HAL_SPI_Init(&hspi2);
    SET_BIT(hspi2.Instance->CR2, SPI_RXFIFO_THRESHOLD_QF);
    SET_BIT(hspi2.Instance->CR1, SPI_CR1_SSI);
    hspi2.Instance->CR1 &= ~SPI_CR1_CRCL;
    hspi2.Instance->CR1 |= SPI_CR1_SPE;

    // SPI4
    HAL_SPI_DeInit(&hspi4);
    hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_SPI_Init(&hspi4);
    SET_BIT(hspi4.Instance->CR2, SPI_RXFIFO_THRESHOLD_QF);
    SET_BIT(hspi4.Instance->CR1, SPI_CR1_SSI);
    hspi4.Instance->CR1 &= ~SPI_CR1_CRCL;
    hspi4.Instance->CR1 |= SPI_CR1_SPE;

    // ---- Encoder Filter Override: 0→8 para todos los encoder TIM ----------
    // TIM2
    HAL_TIM_Encoder_DeInit(&htim2);
    {
        TIM_Encoder_InitTypeDef sConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};
        htim2.Instance = TIM2;
        htim2.Init.Prescaler = 0;
        htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim2.Init.Period = 65535;
        htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 8;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 8;
        if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) Error_Handler();
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
    }

    // TIM3
    HAL_TIM_Encoder_DeInit(&htim3);
    {
        TIM_Encoder_InitTypeDef sConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};
        htim3.Instance = TIM3;
        htim3.Init.Prescaler = 0;
        htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim3.Init.Period = 65535;
        htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 8;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 8;
        if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) Error_Handler();
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
    }

    // TIM5
    HAL_TIM_Encoder_DeInit(&htim5);
    {
        TIM_Encoder_InitTypeDef sConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};
        htim5.Instance = TIM5;
        htim5.Init.Prescaler = 0;
        htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim5.Init.Period = 65535;
        htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 8;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 8;
        if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) Error_Handler();
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) Error_Handler();
    }

    // TIM8
    HAL_TIM_Encoder_DeInit(&htim8);
    {
        TIM_Encoder_InitTypeDef sConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};
        htim8.Instance = TIM8;
        htim8.Init.Prescaler = 0;
        htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim8.Init.Period = 65535;
        htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim8.Init.RepetitionCounter = 0;
        htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 8;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 8;
        if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK) Error_Handler();
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) Error_Handler();
    }
}
