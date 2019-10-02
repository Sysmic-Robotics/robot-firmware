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
}

void Motor_OpenLoop_Drive(Motor_Handler_t *motorDevice, MAX581x_Handler_t *dacDevice, float speed)
{
	if(!motorDevice->enable)
	{
		MAX581x_Code(dacDevice, motorDevice->outputID, 0);
		return;
	}

	Motor_SetVoltage(motorDevice, dacDevice, speed * WHEEL_GEAR_RATIO);
}

void Motor_PID_Drive(Motor_Handler_t *motorDevice, float refSpeed, MAX581x_Handler_t *dacDevice)
{
	if(!motorDevice->enable)
	{
		MAX581x_Code(dacDevice, motorDevice->outputID, 0);
		return;
	}
	/* Apply PID */
	motorDevice->refSpeed = refSpeed;
	motorDevice->measSpeed = Encoder_Update(&motorDevice->encoder, motorDevice->pid.params.sampleTime);
	PID_CloseLoop(&motorDevice->pid, motorDevice->refSpeed, motorDevice->measSpeed);

	Motor_SetVoltage(motorDevice, dacDevice, motorDevice->pid.output);
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
	if(brake == MOTOR_BRAKE_ENABLE)
	{
		HAL_GPIO_WritePin(motorDevice->enablePin.GPIOx, motorDevice->enablePin.GPIO_Pin, GPIO_PIN_RESET);	
	}		
	else
	{
		HAL_GPIO_WritePin(motorDevice->enablePin.GPIOx, motorDevice->enablePin.GPIO_Pin, GPIO_PIN_SET);
	}	
}

void Motor_SetVoltage(Motor_Handler_t *motorDevice, MAX581x_Handler_t *dacDevice, float speed)
{
	if(speed >= (float)0.0)
	{
		HAL_GPIO_WritePin(motorDevice->dirPin.GPIOx, motorDevice->dirPin.GPIO_Pin, GPIO_PIN_SET);
		motorDevice->voltage = (uint16_t)(MOTOR_SPEED_CONV * speed);
		MAX581x_CodeLoad(dacDevice, motorDevice->outputID, motorDevice->voltage);
	}
	else
	{
		HAL_GPIO_WritePin(motorDevice->dirPin.GPIOx, motorDevice->dirPin.GPIO_Pin, GPIO_PIN_RESET);
		motorDevice->voltage = (uint16_t)(MOTOR_SPEED_CONV * fabs(speed));
		MAX581x_CodeLoad(dacDevice, motorDevice->outputID, motorDevice->voltage);
	}	
}
