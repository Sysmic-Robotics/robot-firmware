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

#include "pid.h"

void PID_Init(PID_Handler_t *pid, PID_Params_t params, PID_Status_t enable)
{
	pid->params = params;
	pid->enable = enable;

	pid->error = 0;
	pid->output = 0;

	pid->lastMeasure = 0;
	pid->integral = 0;

	pid->enable = enable;
}

void PID_CloseLoop(PID_Handler_t *pid, float reference, float measure)
{
	if(pid->enable != PID_STATUS_ENABLE)
	{
		pid->lastMeasure = measure;
		pid->output = reference;
		return;
	}
	pid->ref = reference;
	pid->error = pid->ref - measure;

	pid->integral += pid->error * pid->params.Ki;
	if (pid->integral > pid->params.integralMax)
	{
		pid->integral = pid->params.integralMax;
	}
	else if (pid->integral < -pid->params.integralMax)
	{
		pid->integral = -pid->params.integralMax;
	}
	
	float measDiff = measure - pid->lastMeasure;
		
	pid->output = pid->error * pid->params.Kp + pid->integral + measDiff * pid->params.Kd;
	if (pid->output > pid->params.outputMax)
	{
		pid->output = pid->params.outputMax;
	}		
	else if (pid->output < pid->params.outputMin)
	{
		pid->output = pid->params.outputMin;
	}

	pid->lastMeasure = measure;
}

void PID_UpdateController(PID_Handler_t *pid, float Kp, float Ki, float Kd, float sampleTime)
{
	pid->params.Kp = Kp;
	pid->params.Ki = Ki * sampleTime;
	pid->params.Kd = Kd / sampleTime;

	pid->params.sampleTime = sampleTime;
}

void PID_UpdateLimits(PID_Handler_t *pid, float outputMax, float outputMin) 
{
	pid->params.outputMax = outputMax;
	pid->params.outputMin = outputMin;
}

void PID_Enable(PID_Handler_t *pid, PID_Status_t enable)
{
	pid->enable = enable;
}
