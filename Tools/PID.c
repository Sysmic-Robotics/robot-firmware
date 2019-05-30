#include "PID.h"

void PID_Init(PID_Handler_t *pidDevice, PID_Params_t params, PID_Status_t enable)
{
	pidDevice->params = params;
	pidDevice->enable = enable;

	pidDevice->error = 0;
	pidDevice->output = 0;

	pidDevice->lastMeasure = 0;
	pidDevice->integral = 0;

	pidDevice->enable = enable;
}

void PID_CloseLoop(PID_Handler_t *pidDevice, float reference, float measure)
{
	if(pidDevice->enable != PID_STATUS_ENABLE)
	{
		pidDevice->lastMeasure = measure;
		pidDevice->output = reference;
		return;
	}
	pidDevice->ref = reference;
	pidDevice->error = pidDevice->ref - measure;

	pidDevice->integral += pidDevice->error * pidDevice->params.Ki;
	if (pidDevice->integral > pidDevice->params.outputMax)
		pidDevice->integral = pidDevice->params.outputMax;
	else if (pidDevice->integral < pidDevice->params.outputMin)
		pidDevice->integral = pidDevice->params.outputMin;

	float measDiff = measure - pidDevice->lastMeasure;
		
	pidDevice->output = pidDevice->error * pidDevice->params.Kp + pidDevice->integral + measDiff * pidDevice->params.Kd;
	if (pidDevice->output > pidDevice->params.outputMax)
		pidDevice->output = pidDevice->params.outputMax;
	else if (pidDevice->output < pidDevice->params.outputMin)
		pidDevice->output = pidDevice->params.outputMin;

	pidDevice->lastMeasure = measure;
}

void PID_UpdateController(PID_Handler_t *pidDevice, float Kp, float Ki, float Kd, float sampleTime)
{
	pidDevice->params.Kp = Kp;
	pidDevice->params.Ki = Ki * sampleTime;
	pidDevice->params.Kd = Kd / sampleTime;

	pidDevice->params.sampleTime = sampleTime;
}

void PID_UpdateLimits(PID_Handler_t *pidDevice, float outputMax, float outputMin)
{
	pidDevice->params.outputMax = outputMax;
	pidDevice->params.outputMin = outputMin;
}

void PID_Enable(PID_Handler_t *pidDevice, PID_Status_t enable)
{
	pidDevice->enable = enable;
}
