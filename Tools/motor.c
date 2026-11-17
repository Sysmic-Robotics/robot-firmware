/**
 * @author  Pablo Reyes Robles
 * @email   pablo.reyesr@alumnos.usm.cl
 * @version v1.0
@verbatim
   ----------------------------------------------------------------------
   Sysmic Robotics, 2019
   ----------------------------------------------------------------------
@endverbatim
 */

#include "motor.h"

void Motor_Init(Motor_Handler_t *motorDevice, uint8_t motorID, Motor_Status_t enable)
{
	motorDevice->enable = enable;
	motorDevice->outputID = motorID;
	motorDevice->refSpeed = 0;
	motorDevice->measSpeed = 0;
	Motor_Enable(motorDevice, enable);
}

void Motor_OLDrive(Motor_Handler_t *motorDevice, MAX581x_Handler_t *dacDevice, float speed)
{
	if(!motorDevice->enable)
	{
		MAX581x_Code(dacDevice, motorDevice->outputID, 0);
		return;
	}

	Motor_SetVoltage(motorDevice, dacDevice, speed  * MOTOR_SPEED_CONV);
}

void Motor_CLDrive(Motor_Handler_t *motorDevice, MAX581x_Handler_t *dacDevice, float speed)
{
	float rawSpeed = Encoder_Update(&motorDevice->encoder, motorDevice->pid.params.sampleTime);
	motorDevice->measSpeed = LowPassFilter_Update(&motorDevice->lpFilt, rawSpeed);

	if (!motorDevice->enable)
	{
		motorDevice->voltage = 0;
		MAX581x_CodeLoad(dacDevice, motorDevice->outputID, 0);
		return;
	}

	motorDevice->refSpeed = speed * SPEED_CNT_RATIO;

	float ff = 0.0f;
	if (fabsf(motorDevice->refSpeed) > motorDevice->speedDeadband)
	{
		float sign = (motorDevice->refSpeed > 0.0f) ? 1.0f : -1.0f;
		ff += motorDevice->ffFrictionGain * sign;
		ff += motorDevice->ffVelocityGain * motorDevice->refSpeed;
	}

	PID_CloseLoop(&motorDevice->pid, motorDevice->refSpeed, motorDevice->measSpeed);

	float out = motorDevice->pid.output + ff;
	float absOut = fabsf(out);
	if (absOut > 4095.0f) absOut = 4095.0f;
	motorDevice->voltage = (uint16_t)(absOut);

	if (out >= 0.0f)
		HAL_GPIO_WritePin(motorDevice->dirPin.GPIOx, motorDevice->dirPin.GPIO_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(motorDevice->dirPin.GPIOx, motorDevice->dirPin.GPIO_Pin, GPIO_PIN_RESET);

	MAX581x_CodeLoad(dacDevice, motorDevice->outputID, motorDevice->voltage);
}

void Motor_Enable(Motor_Handler_t *motorDevice, Motor_Status_t enable)
{
	motorDevice->enable = enable;
	if(enable == MOTOR_STATUS_DISABLE)
	{
		HAL_GPIO_WritePin(motorDevice->enablePin.GPIOx, motorDevice->enablePin.GPIO_Pin, GPIO_PIN_RESET);	
	}		
	else
	{
		HAL_GPIO_WritePin(motorDevice->enablePin.GPIOx, motorDevice->enablePin.GPIO_Pin, GPIO_PIN_SET);
	}		
}

void Motor_SetBrake(Motor_Handler_t *motorDevice, uint8_t brake)
{
	// NOTE: Brake is active LOW on this H-bridge
	if(brake == MOTOR_BRAKE_ENABLE)
	{
		HAL_GPIO_WritePin(motorDevice->brakePin.GPIOx, motorDevice->brakePin.GPIO_Pin, GPIO_PIN_RESET);	
	}		
	else
	{
		HAL_GPIO_WritePin(motorDevice->brakePin.GPIOx, motorDevice->brakePin.GPIO_Pin, GPIO_PIN_SET);
	}	
}

void Motor_SetVoltage(Motor_Handler_t *motorDevice, MAX581x_Handler_t *dacDevice, float speed)
{
	float absSpeed = fabsf(speed);
	if (absSpeed > 4095.0f) absSpeed = 4095.0f;

	if(speed >= 0.0f)
	{
		HAL_GPIO_WritePin(motorDevice->dirPin.GPIOx, motorDevice->dirPin.GPIO_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(motorDevice->dirPin.GPIOx, motorDevice->dirPin.GPIO_Pin, GPIO_PIN_RESET);
	}
	motorDevice->voltage = (uint16_t)(absSpeed);
	MAX581x_CodeLoad(dacDevice, motorDevice->outputID, motorDevice->voltage);
}
