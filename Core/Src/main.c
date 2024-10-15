/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : main.c
	* @brief          : Main program body
	******************************************************************************
	* @attention
	*
	* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
	* All rights reserved.</center></h2>
	*
	* This software component is licensed by ST under BSD 3-Clause license,
	* the "License"; You may not use this file except in compliance with the
	* License. You may obtain a copy of the License at:
	*                        opensource.org/licenses/BSD-3-Clause
	*
	******************************************************************************
	*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "nrf24.h"
#include "board.h"
#include "string.h"
#include "motor.h"
#include "vl6180x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROBOT_RADIO					    0.08215 //0.18f / 2.0f
#define ROBOT_MAX_LINEAR_ACC	  0.1f

#define ANGULAR_SPEED_FACTOR	  30.0f
#define DRIBBLER_CONV(x)		    x * (1023.0f / 7.0f)

#define VL6180X_THRESHOLD       65
#define VL6180X_SAMPLE_TIME     50 //[ms]

#define nRF24L01_SYSMIC_CHANNEL 0x6A // CHANNEL

enum {
  KICKER_DISCHARGED = 0x00,
  KICKER_CHARGED,
  KICKER_START
};

const uint16_t Dribbler_SpeedSet[] = {0, 450, 492, 575, 585, 617, 658, 700};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

osThreadId driveTaskHandle;
osThreadId radioTaskHandle;
osThreadId kickTaskHandle;
osMessageQId kickQueueHandle;
osMutexId kickFlagHandle;
/* USER CODE BEGIN PV */
osThreadId ballDetectorTaskHandle;
osMessageQId nrf24CheckHandle;
uint32_t checkSPI = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C3_Init(void);
void DriveFunction(void const * argument);
void RadioFunction(void const * argument);
void KickFunction(void const * argument);

/* USER CODE BEGIN PFP */
/* TODO: make object of wheel/motor in open loop */
void nRF24_TxPacket(nRF24_Handler_t *device, uint8_t* Buf, uint32_t Len);
void PackageTxBuffer(uint8_t *buf);

void BallDetectorFunction(void const * argument);
void setSpeed(uint8_t *buffer, float *velocity, uint8_t *turn);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t robot_id;

uint8_t txBuffer[32] = {
    'R','a','d','i','o',' ','O','N','\n'	//termina en <=60 \n=10
};


uint8_t rxBuffer[32];
uint8_t rx_len;

uint8_t status;
uint8_t config;
uint8_t direction[4];
float speed[4];
float kinematic[4][3];
Motor_Handler_t motor[4];

float dribbler_speed = 0.0f;
uint8_t dribbler_sel = 0;
uint8_t kick_sel = 0;
uint8_t kick_flag = 0;
uint16_t kick_delay = 0;
uint16_t kick_count = 0;

nRF24_Handler_t nrf_device;
uint8_t tx_node_addr[5] = {'s', 'y', 's', 't', 'x'};
uint8_t rx_node_addr[5] = {'s', 'y', 's', 'r', 'x'};

VL6180X_Handler_t range_sensor;
uint8_t ball_posession = 0x00;
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	/* Define wheels angles in motor.h */
	kinematic[0][0] = sin(WHEEL_ANGlE_1); kinematic[0][1] = -cos(WHEEL_ANGlE_1); kinematic[0][2] = -ROBOT_RADIO;
	kinematic[1][0] = sin(WHEEL_ANGlE_2); kinematic[1][1] = -cos(WHEEL_ANGlE_2); kinematic[1][2] = -ROBOT_RADIO;
	kinematic[2][0] = sin(WHEEL_ANGlE_3); kinematic[2][1] = -cos(WHEEL_ANGlE_3); kinematic[2][2] = -ROBOT_RADIO;
	kinematic[3][0] = sin(WHEEL_ANGlE_4); kinematic[3][1] = -cos(WHEEL_ANGlE_4); kinematic[3][2] = -ROBOT_RADIO;

	for (uint8_t i = 0; i < 10; i++)
	{
		Board_LedToggle(BOARD_LED_GPIO, BOARD_LED_PIN_1);
		Board_LedToggle(BOARD_LED_GPIO, BOARD_LED_PIN_2);
		Board_LedToggle(BOARD_LED_GPIO, BOARD_LED_PIN_3);
		HAL_Delay(100);
	}
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of kickFlag */
  osMutexDef(kickFlag);
  kickFlagHandle = osMutexCreate(osMutex(kickFlag));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of kickQueue */
  osMessageQDef(kickQueue, 1, uint16_t);
  kickQueueHandle = osMessageCreate(osMessageQ(kickQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	osMessageQDef(nrf24Check, 16, uint16_t);
  nrf24CheckHandle = osMessageCreate(osMessageQ(nrf24Check), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of driveTask */
  osThreadDef(driveTask, DriveFunction, osPriorityAboveNormal, 0, 128);
  driveTaskHandle = osThreadCreate(osThread(driveTask), NULL);

  /* definition and creation of radioTask */
  osThreadDef(radioTask, RadioFunction, osPriorityNormal, 0, 128);
  radioTaskHandle = osThreadCreate(osThread(radioTask), NULL);

  /* definition and creation of kickTask */
  osThreadDef(kickTask, KickFunction, osPriorityLow, 0, 128);
  kickTaskHandle = osThreadCreate(osThread(kickTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	osThreadDef(ballDetectorTask, BallDetectorFunction, osPriorityLow, 0, 128);
  ballDetectorTaskHandle = osThreadCreate(osThread(ballDetectorTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* TODO: USE FREERTOS */
		
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x6000030D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x6000030D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x6000030D;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
	//LL_SPI_WriteReg(SPI1, CR2, SPI_CR2_FRXTH_Msk);
	WRITE_REG(hspi1.Instance->CR2, SPI_CR2_FRXTH_Msk);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PI12 PI13 PI14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PF3 PF4 PF5 PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PH4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PI15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PJ0 PJ1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : PJ4 PJ6 PJ7 PJ8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG10 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PK6 PK7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Function to fill the txBuffer with motor speeds
void updateBuffer(uint8_t *buffer) {

    // Copy the motor speeds to the txBuffer
    memcpy(&buffer[0], &motor[0].measSpeed, sizeof(float));
    memcpy(&buffer[4], '\n',1);
    memcpy(&buffer[4+1], &motor[1].measSpeed, sizeof(float));
    memcpy(&buffer[8], '\n',1);
    memcpy(&buffer[8+1], &motor[2].measSpeed, sizeof(float));
    memcpy(&buffer[12], '\n',1);
    memcpy(&buffer[12+1], &motor[3].measSpeed, sizeof(float));

    // Fill the remaining part of the buffer with zeros if necessary
    memset(&buffer[16], 0, 16);
}


void nRF24_TxPacket(nRF24_Handler_t *device, uint8_t* Buf, uint32_t Len) {
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);

	for(uint32_t i = 0; i < Len; i++)	{
		device->tx_data[i] = *Buf++;
	}


	nRF24_WritePayload(device, device->tx_data, Len);
	nRF24_CE_State(device, GPIO_PIN_SET);

	while(!(status & (nRF24_FLAG_TX_DS)))	{
		status = nRF24_GetStatus(device);
	}

	nRF24_ClearIRQFlagsTx(device);
	nRF24_FlushTX(device);

	nRF24_CE_State(device, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_RESET);
}

//void PackageTxBuffer(uint8_t *txBuffer){
//
//	txBuffer[0] = (uint8_t)robot_id;
//
//	memcpy(&txBuffer[1], &motor[0].measSpeed, sizeof(motor[0].measSpeed));
//	memcpy(&txBuffer[5], &motor[1].measSpeed, sizeof(motor[1].measSpeed));
//	memcpy(&txBuffer[9], &motor[2].measSpeed, sizeof(motor[2].measSpeed));
//	memcpy(&txBuffer[13], &motor[3].measSpeed, sizeof(motor[3].measSpeed));
//}


float v_vel[3];
void setSpeed(uint8_t *buffer, float *velocity, uint8_t *turn)
{
	/* Last velocities */
	float prv_Vx = v_vel[0], prv_Vy = v_vel[1];
	
	/* Velocities vector: vx, vy and vr respectively */
	v_vel[0] = (buffer[1] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0xC0) << 1 | (uint16_t)(buffer[1] & 0x7F)) / 100.0f : (float)((uint16_t)(buffer[4] & 0xC0) << 1 | (uint16_t)(buffer[1] & 0x7F)) / 100.0f;
	v_vel[1] = (buffer[2] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0x30) << 3 | (uint16_t)(buffer[2] & 0x7F)) / 100.0f : (float)((uint16_t)(buffer[4] & 0x30) << 3 | (uint16_t)(buffer[2] & 0x7F)) / 100.0f;
	v_vel[2] = (buffer[3] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0x0F) << 7 | (uint16_t)(buffer[3] & 0x7F)) / 100.0f : (float)((uint16_t)(buffer[4] & 0x0F) << 7 | (uint16_t)(buffer[3] & 0x7F)) / 100.0f;

	/* Check if acceleration is not too high */
	float Ax = v_vel[0] - prv_Vx, Ay = v_vel[1] - prv_Vy;
	float acc_sum = Ax * Ax + Ay * Ay;
	acc_sum = sqrt(acc_sum);
	
	float norm_Ax = Ax / acc_sum, norm_Ay = Ay / acc_sum;
	
	if(acc_sum > ROBOT_MAX_LINEAR_ACC)
	{
		acc_sum = ROBOT_MAX_LINEAR_ACC;
		Ax = norm_Ax * acc_sum;
		Ay = norm_Ay * acc_sum;
		
		v_vel[0] = prv_Vx + Ax;
		v_vel[1] = prv_Vy + Ay;
	}
	
	for (uint8_t i = 0; i < 4; i++)
	{
		/* Temporal speed variable. Calculate each wheel speed respect to robot kinematic model */
		float t_vel = 0;
		for (uint8_t j = 0; j < 3; j++)
		{
			t_vel += kinematic[i][j] * v_vel[j];
		}
		/* Check velocity direction */
		turn[i] = (t_vel > 0) ? WHEEL_P_ROTATION : WHEEL_N_ROTATION;

		/* Fill speed array. Speed in [m/s] */
		velocity[i] = t_vel;
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

uint16_t ball_range;
uint16_t ball_accum;
uint8_t ball_meas_set[10];

void BallDetectorFunction(void const * argument) {
  //uint32_t timeToWait = osKernelSysTick();
  VL6180X_Init(&range_sensor, &hi2c3, VL6180X_DEFAULT_I2C_ADDR);
  ball_range = VL6180X_ReadRange(&range_sensor);
  memset(ball_meas_set, ball_range, 10);

  for (;;) {
    ball_meas_set[0] = VL6180X_ReadRange(&range_sensor);
    ball_accum = ball_meas_set[0];
    for (uint8_t i = 9; i > 0; i--) {
      ball_accum += ball_meas_set[i];
      ball_meas_set[i] = ball_meas_set[i - 1];
    }
    ball_range = ball_accum / 10;
    if (ball_range < VL6180X_THRESHOLD) {
      ball_posession = 0x01;
    }
    else ball_posession = 0x00;
    osDelay(1);
    //osDelayUntil(&timeToWait, (uint32_t)VL6180X_SAMPLE_TIME);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_DriveFunction */
/**
	* @brief  Function implementing the driveTask thread.
	* @param  argument: Not used 
	* @retval None
	*/

osEvent kick_event;
/* USER CODE END Header_DriveFunction */
void DriveFunction(void const * argument)
{
  /* USER CODE BEGIN 5 */
	/* Init PID sampler */
	uint32_t timeToWait = osKernelSysTick();
	/* Init robot_id */
	robot_id = Board_GetID();
	
	/* Init wheels motors DAC: 2.0[V] ref */
	MAX581x_Handler_t driveDAC;
	MAX581x_Init(&driveDAC, &hi2c1, MAX581x_REF_20);
	MAX581x_Code(&driveDAC, MAX581x_OUTPUT_A, 0.0);
	MAX581x_Code(&driveDAC, MAX581x_OUTPUT_B, 0.0);
	MAX581x_Code(&driveDAC, MAX581x_OUTPUT_C, 0.0);
	MAX581x_Code(&driveDAC, MAX581x_OUTPUT_D, 0.0);
	
	/* Init dribbler motor DAC: 2.0[V] ref */
	MAX581x_Handler_t dribblerDAC;
	MAX581x_Init(&dribblerDAC, &hi2c2, MAX581x_REF_20);
	MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, 0.0);
	HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_6, GPIO_PIN_SET);
	
	/* Config motors GPIO and TIM */
	/* Motor 1 */
	motor[0].enablePin.GPIOx = GPIOA;
	motor[0].enablePin.GPIO_Pin = GPIO_PIN_10;
	motor[0].dirPin.GPIOx = GPIOA;
	motor[0].dirPin.GPIO_Pin = GPIO_PIN_9;
	motor[0].brakePin.GPIOx = GPIOA;
	motor[0].brakePin.GPIO_Pin = GPIO_PIN_8;
	
	motor[0].encoder.count = &TIM3->CNT;
	motor[0].encoder.oldPos = TIM3->CNT / ENCODER_CPR;
	motor[0].encoder.enable = ENCODER_STATUS_ENABLE;
	motor[0].encoder.minSpeed = WHEEL_MAX_SPEED_RAD * 0.01f;
	TIM3->CR1 = TIM_CR1_CEN;
	
	/* Motor 2 */
	motor[1].enablePin.GPIOx = GPIOC;
	motor[1].enablePin.GPIO_Pin = GPIO_PIN_11;
	motor[1].dirPin.GPIOx = GPIOC;
	motor[1].dirPin.GPIO_Pin = GPIO_PIN_12;
	motor[1].brakePin.GPIOx = GPIOD;
	motor[1].brakePin.GPIO_Pin = GPIO_PIN_0;
	
	motor[1].encoder.count = &TIM8->CNT;
	motor[1].encoder.oldPos = TIM8->CNT / ENCODER_CPR;
	motor[1].encoder.enable = ENCODER_STATUS_ENABLE;
	motor[1].encoder.minSpeed = WHEEL_MAX_SPEED_RAD * 0.01f;
	TIM8->CR1 = TIM_CR1_CEN;
	
	/* Motor 3 */
	motor[2].enablePin.GPIOx = GPIOK;
	motor[2].enablePin.GPIO_Pin = GPIO_PIN_7;
	motor[2].dirPin.GPIOx = GPIOG;
	motor[2].dirPin.GPIO_Pin = GPIO_PIN_15;
	motor[2].brakePin.GPIOx = GPIOK;
	motor[2].brakePin.GPIO_Pin = GPIO_PIN_6;
	
	motor[2].encoder.count = &TIM2->CNT;
	motor[2].encoder.oldPos = TIM2->CNT / ENCODER_CPR;
	motor[2].encoder.enable = ENCODER_STATUS_ENABLE;
	motor[2].encoder.minSpeed = WHEEL_MAX_SPEED_RAD * 0.01f;
	TIM2->CR1 = TIM_CR1_CEN;
	
	/* Motor 4 */
	motor[3].enablePin.GPIOx = GPIOF;
	motor[3].enablePin.GPIO_Pin = GPIO_PIN_5;
	motor[3].dirPin.GPIOx = GPIOF;
	motor[3].dirPin.GPIO_Pin = GPIO_PIN_3;
	motor[3].brakePin.GPIOx = GPIOF;
	motor[3].brakePin.GPIO_Pin = GPIO_PIN_4;

	motor[3].encoder.count = &TIM5->CNT;
	motor[3].encoder.oldPos = TIM5->CNT / ENCODER_CPR;
	motor[3].encoder.enable = ENCODER_STATUS_ENABLE;
	motor[3].encoder.minSpeed = WHEEL_MAX_SPEED_RAD * 0.01f;
	TIM5->CR1 = TIM_CR1_CEN;
	
	/* Config PID */
	PID_Params_t pidParams;
	pidParams.Kp = 12.0f;
	pidParams.Ki = 4.5f;
	pidParams.Kd = 0.0f;
	pidParams.outputMax = (float)(/*WHEEL_MAX_SPEED_RAD * 10.0f*/ 4095.0f);
	pidParams.outputMin = (float)(/*-WHEEL_MAX_SPEED_RAD * 10.0f*/ -4095.0f);
	pidParams.integralMax = pidParams.outputMax / 5.0f;
	pidParams.sampleTime = PID_SAMPLE_TIME / 1000.0f;


	/* Enable motors and disable brake */
	for (uint8_t i = 0; i < 4; i++)
	{
		Motor_Init(&motor[i], i, MOTOR_STATUS_ENABLE);
		Motor_SetBrake(&motor[i], MOTOR_BRAKE_DISABLE);
		PID_Init(&motor[i].pid, pidParams, PID_STATUS_ENABLE);
	}

	//Motor_Enable(&motor[0], MOTOR_STATUS_ENABLE);
	/* Infinite loop */
	for(;;)
	{						
		for (uint8_t i = 0; i < 4; i++)
		{
			/* Execute open loop (Motor_OLDrive) or closed loop (Motor_CLDrive) routine */
			Motor_CLDrive(&motor[i], &driveDAC, speed[i]);
			
			/* TODO: make dribbler files, variable speeds */
			MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, Dribbler_SpeedSet[dribbler_sel]);
		}

    if(ball_posession && kick_sel && kick_flag == KICKER_CHARGED) {
      osMutexWait(kickFlagHandle, osWaitForever);
      kick_flag = KICKER_START;
      osMutexRelease(kickFlagHandle);
      osMessagePut(kickQueueHandle, 0, 0);
    }   
		
		osMessagePut(nrf24CheckHandle, 0, 0);
		osDelayUntil(&timeToWait, (uint32_t)PID_SAMPLE_TIME);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_RadioFunction */
/**
* @brief Function implementing the radioTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_RadioFunction */
void RadioFunction(void const * argument) {
	/* USER CODE BEGIN RadioFunction */
	tx_node_addr[4]=Board_GetID();

	nRF24_HW_Init(&nrf_device, &hspi1, GPIOG, GPIO_PIN_10, GPIOG, GPIO_PIN_9);
	nRF24_Init(&nrf_device);

	// Configurar dirección y canal de RF
	nRF24_SetAddr(&nrf_device, nRF24_PIPE0, rx_node_addr);
	nRF24_SetRFChannel(&nrf_device, nRF24L01_SYSMIC_CHANNEL);
	nRF24_SetRXPipe(&nrf_device, nRF24_PIPE0, nRF24_AA_OFF, 30);

	// Encender y configurar en modo RX
	nRF24_SetPowerMode(&nrf_device, nRF24_PWR_UP);
	nRF24_SetOperationalMode(&nrf_device, nRF24_MODE_RX);
	nRF24_RX_ON(&nrf_device);
	
	memset(nrf_device.rx_data, 0, 32);
/*
	// Configurar el canal de transmisión una vez al inicio
	nRF24_DisableAA(&nrf_device, nRF24_PIPETX);
	nRF24_SetAddr(&nrf_device, nRF24_PIPETX, tx_node_addr);
	config = nRF24_GetConfig(&nrf_device);
*/
	/* Infinite loop */
	for(;;) {
	osMessageGet(nrf24CheckHandle, osWaitForever);
	status = nRF24_GetStatus(&nrf_device);
	config = nRF24_GetConfig(&nrf_device);


	if (status & nRF24_FLAG_RX_DR) {
		nRF24_ReadPayload(&nrf_device, nrf_device.rx_data, &rx_len);
		nRF24_FlushRX(&nrf_device);
		nRF24_ClearIRQFlagsRx(&nrf_device);

		// Procesar datos recibidos
		setSpeed(nrf_device.rx_data + 5 * robot_id, speed, direction);
		dribbler_sel = getDribbler_speed(nrf_device.rx_data + 5 * robot_id);
		kick_sel = getKickerStatus(nrf_device.rx_data + 5 * robot_id);





		//PackageTxBuffer(txBuffer); //empaqueeta las velocidades en txbuffer
		//memcpy(txBuffer, nrf_device.rx_data, sizeof(nrf_device.rx_data));
		//txBuffer[31] = '\n';
		//txBuffer[30] = Board_GetID();

		//Actualiza informacion del buffer tx motor[i].measSpeed
		updateBuffer(txBuffer);


		// Cambiar a modo TX y enviar datos

		nRF24_RX_OFF(&nrf_device);
		nRF24_SetOperationalMode(&nrf_device, nRF24_MODE_TX);
		//osDelay(40); // Pequeña demora para asegurar que termine de configurarse, evita mandar paquetes erroneos

		while((config & (nRF24_CONFIG_PRIM_RX)))	{//waits for prim_rx to be 0
				config = nRF24_GetConfig(&nrf_device);
		}

		nRF24_TxPacket(&nrf_device, txBuffer, 32);

		// Volver a modo RX

		nRF24_SetOperationalMode(&nrf_device, nRF24_MODE_RX);
		while(!(config & (nRF24_CONFIG_PRIM_RX)))	{//waits for prim_rx to be 0
					config = nRF24_GetConfig(&nrf_device);
			}

		nRF24_RX_ON(&nrf_device);
		nRF24_ClearIRQFlags(&nrf_device);



	}




	}
	/* USER CODE END RadioFunction */
}
/* USER CODE BEGIN Header_KickFunction */
/**
* @brief Function implementing the kickTask thread.
* @param argument: Not used
* @retval None
*/
osEvent kicker_side;
/* USER CODE END Header_KickFunction */
void KickFunction(void const * argument)
{
  /* USER CODE BEGIN KickFunction */
  /* Infinite loop */
  for(;;)
  {

    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_4, GPIO_PIN_SET);
    osDelay(4000);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_4, GPIO_PIN_RESET);

    osMutexWait(kickFlagHandle, osWaitForever);
    kick_flag = KICKER_CHARGED;
    osMutexRelease(kickFlagHandle);

		kicker_side = osMessageGet(kickQueueHandle, osWaitForever);

		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);
		osDelay(10);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

		osMutexWait(kickFlagHandle, osWaitForever);
    kick_flag = KICKER_DISCHARGED;
    osMutexRelease(kickFlagHandle);

		kick_count++;
  }
  /* USER CODE END KickFunction */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
		 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
