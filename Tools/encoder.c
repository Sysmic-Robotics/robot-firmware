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
	/* Tigers */
	int32_t encPos = (int32_t)*encoderDevice->count;
	float encPosF = encPos;
	encPosF /= ENCODER_CPR;

	float velocity = AngleNormalize(encPosF - encoderDevice->oldPos) / sampleTime;
	/*
	if(velocity > 1 || velocity < -1)
	{
		velocity = 0;
	}
	velocity *= (60 / sampleTime);
	*/
	encoderDevice->oldPos = encPosF;

	return velocity;
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
