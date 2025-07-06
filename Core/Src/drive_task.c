#include "system_globals.h"
#include "drive_task.h"
#include "motor.h"
#include "MAX581x.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Definiciones de constantes si no están ya en system_globals.h
#ifndef WHEEL_MAX_SPEED_RAD
#define WHEEL_MAX_SPEED_RAD (M_PI * 40.0f) // Ajusta el valor según tu aplicación
#endif
#ifndef PID_SAMPLE_TIME
#define PID_SAMPLE_TIME 1 // Ajusta el valor según tu aplicación
#endif
#ifndef ENCODER_CPR
#define ENCODER_CPR 4096 // Ajusta el valor según tu aplicación
#endif

// Definiciones de estados del kicker si no están ya en main.h
#ifndef KICKER_CHARGED
#define KICKER_CHARGED 1
#endif
#ifndef KICKER_START
#define KICKER_START 2
#endif

void DriveFunction(void const * argument)
{
    // Init PID sampler
    uint32_t timeToWait = osKernelSysTick();
    // Init robot_id
    robot_id = Board_GetID();

    // Init wheels motors DAC: 2.0[V] ref
    MAX581x_Handler_t driveDAC;
    MAX581x_Init(&driveDAC, &hi2c1, MAX581x_REF_20);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_A, 0.0);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_B, 0.0);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_C, 0.0);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_D, 0.0);

    // Init dribbler motor DAC: 2.0[V] ref
    MAX581x_Handler_t dribblerDAC;
    MAX581x_Init(&dribblerDAC, &hi2c2, MAX581x_REF_20);
    MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, 0.0);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_6, GPIO_PIN_SET);

    // Config motors GPIO and TIM
    // Motor 1
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

    // Motor 2
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

    // Motor 3
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

    // Motor 4
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

    // Config PID
    PID_Params_t pidParams;
    pidParams.Kp = 12.0f;
    pidParams.Ki = 4.5f;
    pidParams.Kd = 0.0f;
    pidParams.outputMax = (float)(4095.0f);
    pidParams.outputMin = (float)(-4095.0f);
    pidParams.integralMax = pidParams.outputMax / 5.0f;
    pidParams.sampleTime = PID_SAMPLE_TIME / 1000.0f;

    // Enable motors and disable brake
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Init(&motor[i], i, MOTOR_STATUS_ENABLE);
        Motor_SetBrake(&motor[i], MOTOR_BRAKE_DISABLE);
        PID_Init(&motor[i].pid, pidParams, PID_STATUS_ENABLE);
    }

    // Infinite loop
    for(;;)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            // Execute open loop (Motor_OLDrive) or closed loop (Motor_CLDrive) routine
            Motor_CLDrive(&motor[i], &driveDAC, speed[i]);
            // TODO: make dribbler files, variable speeds
            MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, Dribbler_SpeedSet[dribbler_sel]);
        }

        if(kick_sel && kick_flag == KICKER_CHARGED) {
            osMutexWait(kickFlagHandle, osWaitForever);
            kick_flag = KICKER_START;
            osMutexRelease(kickFlagHandle);
            osMessagePut(kickQueueHandle, 0, 0);
        }

        osMessagePut(nrf24CheckHandle, 0, 0);
        osDelayUntil(&timeToWait, (uint32_t)PID_SAMPLE_TIME);
    }
}

void setSpeed(uint8_t *buffer, float *velocity, uint8_t *turn)
{

	/* Last velocities */
	float prv_Vx = v_vel[0], prv_Vy = v_vel[1];
	
	/* Velocities vector: vx, vy and vr respectively */
	v_vel[0] = (buffer[1] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0xC0) << 1 | (uint16_t)(buffer[1] & 0x7F))/100.0f  : (float)((uint16_t)(buffer[4] & 0xC0) << 1 | (uint16_t)(buffer[1] & 0x7F))/100.0f ;
	v_vel[1] = (buffer[2] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0x30) << 3 | (uint16_t)(buffer[2] & 0x7F))/100.0f : (float)((uint16_t)(buffer[4] & 0x30) << 3 | (uint16_t)(buffer[2] & 0x7F))/100.0f ;
	v_vel[2] = (buffer[3] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0x0F) << 7 | (uint16_t)(buffer[3] & 0x7F))/100.0f : (float)((uint16_t)(buffer[4] & 0x0F) << 7 | (uint16_t)(buffer[3] & 0x7F))/100.0f ;


	/* Limit linear velocity magnitude */
	float vel_mag = sqrt(v_vel[0]*v_vel[0] + v_vel[1]*v_vel[1]);
	if (vel_mag > ROBOT_MAX_LINEAR_VEL)
	{
	    float scale = ROBOT_MAX_LINEAR_VEL / vel_mag;
	    v_vel[0] *= scale;
	    v_vel[1] *= scale;
	}

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
