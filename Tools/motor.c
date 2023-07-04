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

	Motor_SetVoltage(motorDevice, dacDevice, speed * WHEEL_GEAR_RATIO * MOTOR_SPEED_CONV);
}

void Motor_CLDrive(Motor_Handler_t *motorDevice, MAX581x_Handler_t *dacDevice, float speed)
{
  /*
  if (motorDevice->refSpeed == 0.0f) Motor_Enable(motorDevice, MOTOR_STATUS_DISABLE);
  else Motor_Enable(motorDevice, MOTOR_STATUS_ENABLE);
  */
	/* Apply PID */
	motorDevice->refSpeed = speed * SPEED_CNT_RATIO;
	motorDevice->measSpeed = Encoder_Update(&motorDevice->encoder, motorDevice->pid.params.sampleTime);
	PID_CloseLoop(&motorDevice->pid, motorDevice->refSpeed, motorDevice->measSpeed);
	
	if(fabs(motorDevice->pid.output) < 4.0)	{
		motorDevice->pid.output = 0.0;
	}

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
		HAL_GPIO_WritePin(motorDevice->brakePin.GPIOx, motorDevice->brakePin.GPIO_Pin, GPIO_PIN_RESET);	
	}		
	else
	{
		HAL_GPIO_WritePin(motorDevice->brakePin.GPIOx, motorDevice->brakePin.GPIO_Pin, GPIO_PIN_SET);
	}	
}

void Motor_SetVoltage(Motor_Handler_t *motorDevice, MAX581x_Handler_t *dacDevice, float speed)
{
	if(speed >= (float)0.0)
	{
		HAL_GPIO_WritePin(motorDevice->dirPin.GPIOx, motorDevice->dirPin.GPIO_Pin, GPIO_PIN_SET);
		motorDevice->voltage = (uint16_t)(speed);
		MAX581x_CodeLoad(dacDevice, motorDevice->outputID, motorDevice->voltage);
	}
	else
	{
		HAL_GPIO_WritePin(motorDevice->dirPin.GPIOx, motorDevice->dirPin.GPIO_Pin, GPIO_PIN_RESET);
		motorDevice->voltage = (uint16_t)(fabs(speed));
		MAX581x_CodeLoad(dacDevice, motorDevice->outputID, motorDevice->voltage);
	}	
}
