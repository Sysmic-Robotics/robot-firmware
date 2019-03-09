#include "PID.h"

void PID_Init(PID_Handler_t *pidDevice, PID_Params_t params, PID_Status_t enable)
{
	pidDevice->params = params;
	pidDevice->enable = enable;

	pidDevice->ref = 0;
	pidDevice->error = 0;
	pidDevice->output = 0;

	pidDevice->lastMeasure = 0;
	pidDevice->integral = 0;

	pidDevice->enable = enable;
}

void PID_CloseLoop(PID_Handler_t *pidDevice, double measure)
{
	if(pidDevice->enable != PID_STATUS_ENABLE)
	{
		pidDevice->lastMeasure = measure;
		return;
	}

	double error = pidDevice->ref - measure;

	pidDevice->integral += pidDevice->error * pidDevice->params.Ki;
	if (pidDevice->integral > pidDevice->outputMax)
		pidDevice->integral = pidDevice->outputMax;
	else if (pidDevice->integral < pidDevice->outputMin)
		pidDevice->integral = pidDevice->outputMin;

	double measDiff = measure - pidDevice->lastMeasure;
		
	pidDevice->output = pidDevice->error * pidDevice->params.Kp + pidDevice->integral + measDiff * pidDevice->params.Kd;
	if (pidDevice->output > pidDevice->outputMax)
		pidDevice->output = pidDevice->outputMax;
	else if (pidDevice->output < pidDevice->outputMin)
		pidDevice->output = pidDevice->outputMin;

	pidDevice->lastMeasure = measure;
}

void PID_UpdateController(PID_Handler_t *pidDevice, double Kp, double Ki, double Kd, double sampleTime)
{
	pidDevice->params.Kp = Kp;
	pidDevice->params.Ki = Ki * sampleTime;
	pidDevice->params.Kd = Kd / sampleTime;

	pidDevice->params.sampleTime = sampleTime;
}

void PID_UpdateLimits(PID_Handler_t *pidDevice, double outputMax, double outputMin)
{
	pidDevice->params.outputMax = outputMaxp;
	pidDevice->params.outputMin = outputMin;
}

void PID_Enable(PID_Handler_t *pidDevice, PID_Status_t enable)
{
	pidDevice->enable = enable;
}
