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

#include "encoder.h"

float Encoder_Update(Encoder_Handler_t *encoderDevice, float sampleTime)
{
	if(encoderDevice->enable != ENCODER_STATUS_ENABLE)
		return 0;
	uint16_t count = (uint16_t)*encoderDevice->count;
	int16_t delta = (int16_t)(uint16_t)(count - encoderDevice->previousCount);
	encoderDevice->previousCount = count;
	return (float)delta * (2.0f * (float)M_PI) / (ENCODER_CPR * sampleTime);
}

void Encoder_Sync(Encoder_Handler_t *encoderDevice)
{
	encoderDevice->previousCount = (uint16_t)*encoderDevice->count;
}

/* Tigers angle normalizer */
float mod(float x, float y)
{
	if(y == 0.0f)
		return x;

	float m = x - y * floorf(x / y);

	// handle boundary cases resulted from floating-point cut off:
	if(y > 0) // modulo range: [0..y)
	{
		if(m >= y)
			return 0;

		if(m < 0)
		{
			if(y + m == y)
				return 0;
			else
				return y + m;
		}
	}
	else // modulo range: (y..0]
	{
		if(m <= y)
			return 0;

		if(m > 0)
		{
			if(y + m == y)
				return 0;
			else
				return y + m;
		}
	}

	return m;
}

// wrap [rad] angle to [-PI..PI)
float AngleNormalize(float a)
{
	return mod((2 * a + 1) * (float)M_PI, 2 * M_PI) - (float)M_PI;
}
