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

#include "PID.h"
#include <math.h>

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

void PID_CloseLoop(PID_Handler_t *pid, float reference, float measure,
                   float feedforward, float dt)
{
	if(pid->enable != PID_STATUS_ENABLE)
	{
		pid->lastMeasure = measure;
		pid->output = reference + feedforward;
		return;
	}
	pid->ref = reference;
	pid->error = pid->ref - measure;

	const float limit = pid->params.integralMax;
	const float previous = fminf(limit, fmaxf(-limit, pid->integral));
	const float samples = dt / pid->params.sampleTime;
	const float candidate = fminf(limit, fmaxf(-limit,
		previous + pid->error * pid->params.Ki * samples));
	const float base = feedforward + pid->error * pid->params.Kp -
		(measure - pid->lastMeasure) * pid->params.Kd / samples;
	const float requested = base + candidate;
	/* The feedforward also consumes DAC headroom. Keep the old integral
	   if the requested output would saturate further in its direction. */
	const int saturated = (requested > pid->params.outputMax && candidate > previous) ||
	                      (requested < pid->params.outputMin && candidate < previous);
	pid->integral = saturated ? previous : candidate;
	pid->output = fminf(pid->params.outputMax,
	                    fmaxf(pid->params.outputMin, base + pid->integral));

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
