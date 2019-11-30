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
#include "nrf24.h"
#include "board.h"
#include "string.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROBOT_RADIO   0.18f // 0.083648f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

osThreadId driveTaskHandle;
osThreadId radioTaskHandle;
/* USER CODE BEGIN PV */
uint32_t checkSPI = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
void DriveFunction(void const * argument);
void RadioFunction(void const * argument);

/* USER CODE BEGIN PFP */
/* TODO: make object of wheel/motor in open loop */
void setSpeed(uint8_t *buffer, float *velocity, uint8_t *turn);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rxBuffer[32];
uint8_t rxAddr[5];
uint8_t rxLen;

uint8_t status;
uint8_t direction[4];
float speed[4];
float kinematic[4][3];
Motor_Handler_t motor[4];
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
  MX_TIM3_Init();
  MX_TIM5_Init();
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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of driveTask */
  osThreadDef(driveTask, DriveFunction, osPriorityAboveNormal, 0, 128);
  driveTaskHandle = osThreadCreate(osThread(driveTask), NULL);

  /* definition and creation of radioTask */
  osThreadDef(radioTask, RadioFunction, osPriorityNormal, 0, 128);
  radioTaskHandle = osThreadCreate(osThread(radioTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
	LL_SPI_WriteReg(SPI1, CR2, SPI_CR2_FRXTH_Msk);
  /* USER CODE END SPI1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PF3 PF4 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

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
float v_vel[3];
void setSpeed(uint8_t *buffer, float *velocity, uint8_t *turn)
{
	/* Velocities vector: vx, vy and vr respectively */
	
	v_vel[0] = (*(buffer + 1) & 0x80) ? -((float)(*(buffer + 1) & 0x7F)) / 100.0f : ((float)(*(buffer + 1) & 0x7F)) / 100.0f;
	v_vel[1] = (*(buffer + 2) & 0x80) ? -((float)(*(buffer + 2) & 0x7F)) / 100.0f : ((float)(*(buffer + 2) & 0x7F)) / 100.0f;
	v_vel[2] = (*(buffer + 3) & 0x80) ? -((float)(*(buffer + 3) & 0x7F)) / 100.0f : ((float)(*(buffer + 3) & 0x7F)) / 100.0f;

	for (uint8_t i = 0; i < 4; i++)
	{
		/* Temporal speed variable. Calculate each wheel speed respect to robot kinematic model */
		float t_vel = 0;
		for (uint8_t j = 0; j < 3; j++)
		{
			t_vel += kinematic[i][j] * v_vel[j];
		}
		/* Check velocity direction */
		*(turn + i) = (t_vel > 0) ? WHEEL_P_ROTATION : WHEEL_N_ROTATION;

		/* Fill speed array. Speed in [rad/s] */
		*(velocity + i) = t_vel / WHEEL_RADIO;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_DriveFunction */
/**
  * @brief  Function implementing the driveTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_DriveFunction */
void DriveFunction(void const * argument)
{
    
    
    
    

  /* USER CODE BEGIN 5 */
  uint32_t timeToWait = osKernelSysTick();

  MAX581x_Handler_t dacDevice;
  MAX581x_Init(&dacDevice, &hi2c1, MAX581x_REF_20);
  MAX581x_Code(&dacDevice, MAX581x_OUTPUT_A, 0.0);
	MAX581x_Code(&dacDevice, MAX581x_OUTPUT_B, 0.0);
	MAX581x_Code(&dacDevice, MAX581x_OUTPUT_C, 0.0);
	MAX581x_Code(&dacDevice, MAX581x_OUTPUT_D, 0.0);
	
	motor[0].enablePin.GPIOx = GPIOA;
	motor[0].enablePin.GPIO_Pin = GPIO_PIN_10;
	motor[0].dirPin.GPIOx = GPIOA;
	motor[0].dirPin.GPIO_Pin = GPIO_PIN_9;
	motor[0].brakePin.GPIOx = GPIOA;
	motor[0].brakePin.GPIO_Pin = GPIO_PIN_8;
	
	motor[1].enablePin.GPIOx = GPIOC;
	motor[1].enablePin.GPIO_Pin = GPIO_PIN_11;
	motor[1].dirPin.GPIOx = GPIOC;
	motor[1].dirPin.GPIO_Pin = GPIO_PIN_12;
	motor[1].brakePin.GPIOx = GPIOD;
	motor[1].brakePin.GPIO_Pin = GPIO_PIN_0;
	
	motor[2].enablePin.GPIOx = GPIOK;
	motor[2].enablePin.GPIO_Pin = GPIO_PIN_7;
	motor[2].dirPin.GPIOx = GPIOG;
	motor[2].dirPin.GPIO_Pin = GPIO_PIN_15;
	motor[2].brakePin.GPIOx = GPIOK;
	motor[2].brakePin.GPIO_Pin = GPIO_PIN_6;
	
	motor[3].enablePin.GPIOx = GPIOF;
	motor[3].enablePin.GPIO_Pin = GPIO_PIN_5;
	motor[3].dirPin.GPIOx = GPIOF;
	motor[3].dirPin.GPIO_Pin = GPIO_PIN_3;
	motor[3].brakePin.GPIOx = GPIOF;
	motor[3].brakePin.GPIO_Pin = GPIO_PIN_4;
	
	PID_Params_t pidParams;
  pidParams.Kp = 0.1;
  pidParams.Ki = 10.0;
  pidParams.Kd = 0.0;
  pidParams.outputMax = (float)(MOTOR_NOMINAL_SPEED * 2);
  pidParams.outputMin = (float)(-MOTOR_NOMINAL_SPEED * 2);
  pidParams.sampleTime = 0.001;

  motor[0].encoder.count = &TIM3->CNT;
  motor[0].encoder.oldPos = TIM3->CNT / ENCODER_CPR;
	motor[0].encoder.enable = ENCODER_STATUS_ENABLE;
	TIM3->CR1 = TIM_CR1_CEN;
	
	motor[3].encoder.count = &TIM5->CNT;
  motor[3].encoder.oldPos = TIM5->CNT / ENCODER_CPR;
	motor[3].encoder.enable = ENCODER_STATUS_ENABLE;
	TIM5->CR1 = TIM_CR1_CEN;

  for (uint8_t i = 0; i < 4; i++)
  {
    Motor_Init(&motor[i], i, MOTOR_STATUS_ENABLE);
    Motor_SetBrake(&motor[i], MOTOR_BRAKE_DISABLE);
		PID_Init(&motor[i].pid, pidParams, PID_STATUS_ENABLE);
  }
	
	Motor_Enable(&motor[0], MOTOR_STATUS_DISABLE);
	Motor_Enable(&motor[1], MOTOR_STATUS_DISABLE);
	Motor_Enable(&motor[2], MOTOR_STATUS_DISABLE);
	
  /* Infinite loop */
  for(;;)
  {
    setSpeed(rxBuffer, speed, direction);
		speed[3] = 2 * M_PI;
    for (uint8_t i = 0; i < 4; i++)
    {
      Motor_CLDrive(&motor[i], &dacDevice, speed[i]);
    }
    osDelayUntil(&timeToWait, 1);
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
void RadioFunction(void const * argument)
{
  /* USER CODE BEGIN RadioFunction */
	nRF24_GPIO_Init();
	nRF24_Init();
	nRF24_SetRXPipe(nRF24_PIPE0, nRF24_AA_OFF, 24);
  nRF24_DisableAA(nRF24_PIPETX);
	nRF24_SetPowerMode(nRF24_PWR_UP);
	nRF24_SetOperationalMode(nRF24_MODE_RX);
  /*
	checkSPI = nRF24_Check();
	memset(rxAddr, 0xE7, 5);
	nRF24_SetAddr(0, rxAddr);
  */
	nRF24_RX_ON();	
	
	memset(rxBuffer, 0, 24);
  /* Infinite loop */
  for(;;)
  {
		status = nRF24_GetStatus();
		//if(nRF24_GetStatus_RXFIFO() == nRF24_STATUS_RXFIFO_DATA)
		if(nRF24_GetStatus() & nRF24_FLAG_RX_DR)
		{
			nRF24_ReadPayload(rxBuffer, &rxLen);
			nRF24_FlushRX();
			nRF24_ClearIRQFlags();
		}
    osDelay(16);
  }
  /* USER CODE END RadioFunction */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
