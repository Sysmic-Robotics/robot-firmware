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
  /*
  if (motorDevice->refSpeed == 0.0f) Motor_Enable(motorDevice, MOTOR_STATUS_DISABLE);
  else Motor_Enable(motorDevice, MOTOR_STATUS_ENABLE);
  */
	/* Apply PID */
	motorDevice->refSpeed = speed * SPEED_CNT_RATIO;
	const float measured = Encoder_Update(&motorDevice->encoder, motorDevice->pid.params.sampleTime);
	motorDevice->measSpeed = motorDevice->speedFilterGain < 1.0f ?
		motorDevice->measSpeed + motorDevice->speedFilterGain * (measured - motorDevice->measSpeed) :
		measured;

	const float magnitude = fabsf(motorDevice->refSpeed);
	const float blend = motorDevice->ffSpeedGain > 0.0f ?
		fminf(1.0f, magnitude / 4.0f) : (magnitude > motorDevice->speedDeadband ? 1.0f : 0.0f);
	const float ff = copysignf(motorDevice->ffVoltage * blend +
	                          motorDevice->ffSpeedGain * magnitude, motorDevice->refSpeed);
	PID_CloseLoop(&motorDevice->pid, motorDevice->refSpeed, motorDevice->measSpeed);

	float command = motorDevice->pid.output + ff;
	if (command > MAX581x_MAX_VAL) command = MAX581x_MAX_VAL;
	else if (command < -MAX581x_MAX_VAL) command = -MAX581x_MAX_VAL;
	motorDevice->cmdOutput = command; /* Signed DAC code actually applied (telemetry) */
	Motor_SetVoltage(motorDevice, dacDevice, command);
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
	/* PID + feedforward can exceed the DAC range; larger codes wrap around in MAX581x_CodeLoad */
	if(speed > MAX581x_MAX_VAL) speed = MAX581x_MAX_VAL;
	else if(speed < -MAX581x_MAX_VAL) speed = -MAX581x_MAX_VAL;

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
