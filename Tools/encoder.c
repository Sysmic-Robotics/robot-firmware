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
	if (encoderDevice->enable != ENCODER_STATUS_ENABLE)
		return 0;

	// Evita división por cero o sampleTime irreal
	if (sampleTime < 1e-6f)
		return 0;

	uint16_t raw = (uint16_t)*encoderDevice->count;
	uint16_t udiff = raw - encoderDevice->lastRaw;

	// udiff > 4096 y < 61439 → salto mayor a media revolución
	// (detecta correctamente incluso cuando udiff wrappeó por 16-bit)
	if (udiff > 4096 && udiff < 61439) {
		encoderDevice->lastRaw = raw;
		return 0;
	}

	int16_t diff = (int16_t)udiff;
	encoderDevice->lastRaw = raw;

	float speed = (float)diff / (ENCODER_CPR * sampleTime);
	speed *= 2.0f * (float)M_PI;

	if (fabsf(speed) > 500.0f)
		speed = 0.0f;

	return speed;
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
