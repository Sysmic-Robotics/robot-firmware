/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "cordic.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "as5600.h"
#include "drv8323.h"
#include "comm_spi.h"
#include "foc.h"
#include "servo_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* AS5600 magnetic encoder (absolute reference) */
static AS5600_TypeDef as5600;
static uint16_t as5600_angle;

/* DRV8323S gate driver */
static DRV8323_HandleTypeDef drv8323;

/* TIM1 quadrature encoder (incremental, fast) */
Encoder_TypeDef encoder;  /* non-static for TIM6 ISR extern */

/* Current control mode tracking */
static uint8_t current_mode = COMM_MODE_STOP;

/* Servo control state machine (global, accessed by TIM6 ISR) */
ServoControl g_servo;

/* DRV8323 HW fault flag — set by EXTI15_10 ISR, cleared in main loop after recovery */
volatile uint8_t g_drv8323_hw_fault = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CORDIC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  /* ---- AS5600 (absolute reference, I2C) ---- */
  as5600.i2cHandle = &hi2c2;
  as5600.i2cAddr = AS5600_SLAVE_ADDRESS;

  /* ---- DRV8323S gate driver (SPI3) ---- */
  DRV8323_Init(&drv8323, &hspi3,
               GPIOC, GPIO_PIN_13,   /* CS */
               GPIOC, GPIO_PIN_14,   /* EN */
               GPIOB, GPIO_PIN_13);  /* nFAULT */
  // TODO: verify HiZ pin assignment on PCB — PC15 matches moteus family 1
  // (moteus_hw.cc:324), but confirm the trace connects to DRV8323 HiZ pin.
  DRV8323_SetHiZPin(&drv8323, GPIOC, GPIO_PIN_15);

  DRV8323_PowerOff(&drv8323);
  DRV8323_Enable(&drv8323);
  DRV8323_PowerOn(&drv8323);

  /* ---- TIM1 quadrature encoder (incremental, fast) ---- */
  {
    Encoder_Config ecfg = {
      .cpr                = ENCODER_CPR_DEFAULT,
      .pole_pairs         = ENCODER_POLE_PAIRS_DEFAULT,
      .velocity_lpf_alpha = 0.15f,
      .update_freq        = 20000.0f, /* 20 kHz FOC ISR rate */
    };
    Encoder_Init(&encoder, &ecfg);
  }

  /* ---- Optional: seed encoder absolute position from AS5600 ---- */
  {
    uint16_t raw_angle;
    if (AS5600_GetAngle(&as5600, &raw_angle) == HAL_OK) {
      /* AS5600 gives 12-bit (0-4095) angle over 360°.
       * Convert to encoder counts: ang_counts = raw_angle * CPR / 4096 */
      int32_t abs_pos = (int32_t)raw_angle * (int32_t)encoder.config.cpr / 4096;
      Encoder_SetPosition(&encoder, abs_pos);
    }
  }
  /* ---- Initialise SPI2 slave communication ---- */
  COMM_Init();

  /* ---- Calibrate ADC1 for FOC current sensing (scan mode 2-ch, set in adc.c) ---- */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  /* ---- Initialise FOC (must be after TIM2 and ADC config) ---- */
  FOC_Init();

  /* ---- Initialise ServoControl state machine ---- */
  ServoControl_Init(&g_servo,
                    1.0f / FOC_PWM_FREQ_DEFAULT,   /* 50 µs period */
                    ENCODER_POLE_PAIRS_DEFAULT,     /* poles */
                    0.1f,                           /* motor resistance [Ω] */
                    0.05f);                         /* torque constant [Nm/A] */
  /* Override position PID config for our application */
  g_servo.config.pid_position.kp = 4.0f;
  g_servo.config.pid_position.ki = 1.0f;
  g_servo.config.pid_position.kd = 0.05f;
  g_servo.config.pid_position.sign = -1;
  /* Override velocity PID */
  g_servo.config.pid_velocity.kp = 0.5f;
  g_servo.config.pid_velocity.ki = 2.0f;
  g_servo.config.pid_velocity.sign = -1;
  /* Set current limit */
  g_servo.config.current_limit_A = FOC_MAX_CURRENT_DEFAULT;

  /* Set motor parameters for BEMF feedforward and cross-coupling decoupling */
  FOC_SetMotorParams(
      g_servo.config.motor.inductance_d_H,   /* Ld [H]  */
      g_servo.config.motor.inductance_q_H,   /* Lq [H]  */
      g_servo.config.motor.v_per_hz);        /* BEMF [V/(rad/s)] */

  /* ---- TIM6 basic timer: 20 kHz periodic ISR for FOC loop ---- */
  HAL_TIM_Base_Start_IT(&htim6);

  /* ---- Enable EXTI15_10 for DRV8323 nFAULT (PB13) — prio 0 (highest) ---- */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- 1. Poll SPI2 for new setpoints from master ---- */
    {
      COMM_Setpoint sp;
      if (COMM_GetSetpoint(&sp)) {
        /* SPI scaling factors: position [rev], velocity [rev/s], torque [A] */
        static const float kPosScale  = 10.0f   / 32767.0f;  /* ±32767 → ±10 rev */
        static const float kVelScale  = 50.0f   / 32767.0f;  /* ±32767 → ±50 rev/s */
        static const float kTrqScale  = 0.001f;               /* int16 mA → A        */

        ServoCommandData cmd;
        ServoCommandData_Init(&cmd);

        switch (sp.cmd_id) {
          case COMM_CMD_SET_SETPOINT: {
            switch (sp.mode) {
              case COMM_MODE_STOP:
                cmd.mode = SERVO_MODE_STOPPED;
                break;
              case COMM_MODE_CURRENT:
                cmd.mode = SERVO_MODE_CURRENT;
                cmd.i_q_A = (float)sp.torque * kTrqScale;
                break;
              case COMM_MODE_TORQUE:
                cmd.mode = SERVO_MODE_CURRENT;
                cmd.i_q_A = (float)sp.torque * kTrqScale;
                break;
              case COMM_MODE_VELOCITY:
                cmd.mode = SERVO_MODE_POSITION;
                cmd.position = NAN;          /* hold current position */
                cmd.velocity = (float)sp.velocity * kVelScale;
                cmd.kp_scale = 0.0f;         /* derivative-only */
                cmd.kd_scale = 1.0f;
                cmd.ilimit_scale = 0.0f;
                cmd.max_torque_Nm = (float)sp.torque * kTrqScale;
                break;
              case COMM_MODE_POSITION:
                cmd.mode = SERVO_MODE_POSITION;
                cmd.position = (float)sp.position * kPosScale;
                cmd.velocity = (float)sp.velocity * kVelScale;
                cmd.max_torque_Nm = (float)sp.torque * kTrqScale;
                break;
              default:
                cmd.mode = SERVO_MODE_STOPPED;
                break;
            }
            break;
          }
          case COMM_CMD_EMERGENCY_STOP:
            ServoControl_Stop(&g_servo);
            DRV8323_PowerOff(&drv8323);
            DRV8323_Disable(&drv8323);
            current_mode = COMM_MODE_STOP;
            break;

          case COMM_CMD_CALIBRATE: {
            /* Start calibration sequence using current electrical angle */
            float theta_elec = Encoder_GetElectricalAngle(&encoder);
            ServoControl_StartCalibration(&g_servo, theta_elec);
            if (!g_foc.enabled) FOC_Enable();
            current_mode = COMM_MODE_CURRENT;
            break;
          }

          default:
            break;
        }

        /* Send command to ServoControl */
        ServoControl_SetCommand(&g_servo, &cmd);

        /* Enable FOC if servo enters an active mode */
        if (g_servo.status.mode != SERVO_MODE_STOPPED &&
            g_servo.status.mode != SERVO_MODE_FAULT) {
          if (!g_foc.enabled) FOC_Enable();
        } else {
          if (g_foc.enabled) FOC_Disable();
        }

        current_mode = sp.mode;
      }
    }

    /* ---- 2. Read AS5600 absolute angle (slow I2C, reference) ---- */
    {
      uint16_t raw_angle;
      if (AS5600_GetAngle(&as5600, &raw_angle) == HAL_OK) {
        as5600_angle = raw_angle;
      }
    }

    /* ---- 3. Check DRV8323 fault status (ISR + polling) ---- */
    if (g_drv8323_hw_fault) {
      /* Clear the HW flag that EXTI15_10_IRQHandler set */
      g_drv8323_hw_fault = 0;

      /* Read fault details via SPI */
      DRV8323_ReadStatus(&drv8323);

      /* Disable FOC and set ServoControl fault state */
      FOC_Disable();
      DRV8323_Disable(&drv8323);
      ServoControl_SetFault(&g_servo, SERVO_FAULT_MOTOR_DRIVER);

      /* Wait for driver to be re-enabled (debounce, charge pump) */
      HAL_Delay(200);

      /* Poll the nFAULT pin — if still active, retry after delay.
       * This loop runs in the background main loop (~1 Hz), so
       * this is a slow retry mechanism. For immediate response,
       * the EXTI ISR handles PWM shutdown. */
      if (DRV8323_IsFault(&drv8323)) {
        /* Fault still persisting — do not re-enable */
        continue;  /* skip telemetry, try again next loop */
      }

      /* Fault cleared — re-enable driver and transition to STANDBY */
      DRV8323_Enable(&drv8323);
      HAL_Delay(2);
      DRV8323_PowerOn(&drv8323);
      ServoControl_Stop(&g_servo);

      /* Move to STANDBY so the next SPI command can start fresh */
      current_mode = COMM_MODE_STOP;
    }

    /* ---- 4. Send telemetry back over SPI2 ---- */
    {
      float vbus = 24.0f;  /* nominal VBUS (PA9 conflict) */
      float pos_rev = (float)Encoder_GetPosition(&encoder) /
                      (float)encoder.config.cpr;

      /* Scale velocity [rev/s] → int16 */
      float vel_scaled = g_servo.status.velocity * 100.0f;
      int16_t vel_telem;
      if (vel_scaled > 32767.0f)        vel_telem = 32767;
      else if (vel_scaled < -32768.0f)  vel_telem = -32768;
      else                              vel_telem = (int16_t)vel_scaled;

      /* Scale Iq current [A] → int16 (mA) */
      float iq_telem_f = g_servo.status.q_A * 1000.0f;
      int16_t iq_telem;
      if (iq_telem_f > 32767.0f)        iq_telem = 32767;
      else if (iq_telem_f < -32768.0f)  iq_telem = -32768;
      else                              iq_telem = (int16_t)iq_telem_f;

      /* Map ServoControl state to COMM telemetry */
      uint8_t state;
      if (g_servo.status.mode == SERVO_MODE_FAULT) {
        state = COMM_STATE_FAULT;
      } else if (g_servo.status.mode == SERVO_MODE_STOPPED) {
        state = COMM_STATE_IDLE;
      } else if (g_foc.enabled) {
        state = COMM_STATE_RUNNING;
      } else {
        state = COMM_STATE_READY;
      }

      /* Map ServoFault → COMM fault flags */
      uint8_t fault = 0;
      switch (g_servo.status.fault) {
        case SERVO_FAULT_NONE:            fault = 0;                break;
        case SERVO_FAULT_OVERVOLTAGE:     fault = COMM_FAULT_OVERVOLTAGE;  break;
        case SERVO_FAULT_UNDERVOLTAGE:    fault = COMM_FAULT_UNDERVOLTAGE; break;
        case SERVO_FAULT_OVERTEMPERATURE: fault = COMM_FAULT_OVERTEMP;     break;
        case SERVO_FAULT_ENCODER:         fault = COMM_FAULT_ENCODER;      break;
        default:                          fault = COMM_FAULT_GENERAL;      break;
      }

      COMM_SetTelemetry(
          (int16_t)(encoder.position & 0xFFFF),
          vel_telem,
          iq_telem,
          (uint16_t)(vbus * 1000.0f),  /* Vbus [mV] */
          state,
          fault
      );
    }

    /* Yield to idle — SPI interrupt-driven, FOC runs in TIM6 ISR */
    HAL_Delay(1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
