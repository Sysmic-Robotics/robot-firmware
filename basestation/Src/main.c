/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum {
  SEND_FLAG_RESET = 0,
  SEND_FLAG_SET
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_GPIO_PORT           GPIOC
#define LED_GPIO_PIN            GPIO_PIN_13

#define nRF24L01_SYSMIC_CHANNEL 0x6B
#define TELEMETRY_CHANNEL       0x6C

#define TELEMETRY_SIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t serial_data[30];       /* DMA writes here (UART RX) */
uint8_t ready_data[30];        /* ISR copies here while DMA is stopped */
volatile uint32_t serial_len;
volatile uint8_t send_flag = 0;

nRF24_Handler_t tx_device;
nRF24_Handler_t rx_device;
uint8_t rx_len;

uint8_t tx_node_addr[5] = {'s', 'y', 's', 't', 'x'};
uint8_t rx_node_addr[5] = {'s', 'y', 's', 'r', 'x'};
uint8_t correction_node_addr[5] = {'s', 'y', 's', 'r', 'c'};

  uint8_t telemetry_buf[32];
  uint8_t telemetry_len;

  uint8_t uart_tx_buf[32];
  volatile uint8_t uart_tx_busy = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void nRF24_TxPacket(nRF24_Handler_t *device, uint8_t* Buf, uint32_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* --- TX + RX: UART + nRF24 TX + nRF24 RX telemetry --- */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  nRF24_HW_Init(&tx_device, &hspi1, GPIOB, GPIO_PIN_0, GPIOA, GPIO_PIN_4);
  nRF24_Init(&tx_device);
  nRF24_SetAddr(&tx_device, nRF24_PIPETX, rx_node_addr);
  nRF24_SetRFChannel(&tx_device, nRF24L01_SYSMIC_CHANNEL);
  nRF24_SetRXPipe(&tx_device, nRF24_PIPE0, nRF24_AA_OFF, 32);
  nRF24_DisableAA(&tx_device, nRF24_PIPETX);
  nRF24_SetPowerMode(&tx_device, nRF24_PWR_UP);
  nRF24_SetOperationalMode(&tx_device, nRF24_MODE_TX);

  nRF24_HW_Init(&rx_device, &hspi1, GPIOB, GPIO_PIN_1, GPIOA, GPIO_PIN_3);
  nRF24_Init(&rx_device);
  nRF24_SetAddr(&rx_device, nRF24_PIPE1, tx_node_addr);
  nRF24_SetRFChannel(&rx_device, TELEMETRY_CHANNEL);
  nRF24_SetRXPipe(&rx_device, nRF24_PIPE1, nRF24_AA_OFF, 32);
  nRF24_DisableAA(&rx_device, nRF24_PIPETX);
  nRF24_SetPowerMode(&rx_device, nRF24_PWR_UP);
  nRF24_SetOperationalMode(&rx_device, nRF24_MODE_RX);
  nRF24_RX_ON(&rx_device);

  HAL_UART_Receive_DMA(&huart1, serial_data, 30);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT | DMA_IT_TC);
  __HAL_DMA_DISABLE(huart1.hdmarx);
  __HAL_DMA_CLEAR_FLAG(huart1.hdmarx, DMA_FLAG_GL5);
  huart1.hdmarx->Instance->CNDTR = 30;
  __HAL_DMA_ENABLE(huart1.hdmarx);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  send_flag = SEND_FLAG_SET;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      if (send_flag) {
    	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	  send_flag = SEND_FLAG_RESET;

    	  if (ready_data[0] == 0xFE) {
    		  /* Camera-correction packet (18 bytes) → dedicated correction pipe. */
    		  nRF24_SetAddr(&tx_device, nRF24_PIPETX, correction_node_addr);
    		  nRF24_TxPacket(&tx_device, ready_data, 18);
    		  nRF24_SetAddr(&tx_device, nRF24_PIPETX, rx_node_addr);
    	  } else {
    		  nRF24_TxPacket(&tx_device, ready_data, serial_len > 0 ? serial_len : 30);
    	  }
      }

      uint8_t fifo_status = nRF24_GetStatus_RXFIFO(&rx_device);
      if (fifo_status != nRF24_STATUS_RXFIFO_EMPTY && !uart_tx_busy) {
    	  uint8_t rx_payload_len = 32;
    	  nRF24_ReadPayload(&rx_device, telemetry_buf, &rx_payload_len);

    	  memcpy(uart_tx_buf, telemetry_buf, 32);
    	  if (HAL_UART_Transmit_IT(&huart1, uart_tx_buf, 32) == HAL_OK) {
    		  uart_tx_busy = 1;
    	  }
      }
      nRF24_ClearIRQFlags(&rx_device);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void nRF24_TxPacket(nRF24_Handler_t *device, uint8_t* Buf, uint32_t Len) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	for(uint32_t i = 0; i < Len; i++)	{
		device->tx_data[i] = *Buf++;
	}

	if(Len >= 24)	{
		nRF24_WritePayload(device, device->tx_data, Len);
		nRF24_CE_State(device, GPIO_PIN_SET);

		uint8_t tx_status;
		uint32_t timeout = HAL_GetTick();
		do {
			tx_status = nRF24_GetStatus(device);
			if (HAL_GetTick() - timeout > 100) break;
		} while (!(tx_status & nRF24_FLAG_TX_DS));

		nRF24_ClearIRQFlags(device);
		nRF24_FlushTX(device);
		nRF24_CE_State(device, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* DMA is stopped — serial_data is stable. Copy to ready_buffer
		   so the main loop can read it safely after DMA restarts. */
		serial_len = 30 - huart->hdmarx->Instance->CNDTR;
		memcpy(ready_data, serial_data, serial_len);

		__HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAG_GL5);
		huart->hdmarx->Instance->CNDTR = 30;

		__HAL_DMA_ENABLE(huart->hdmarx);
		send_flag = SEND_FLAG_SET;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		uart_tx_busy = 0;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
