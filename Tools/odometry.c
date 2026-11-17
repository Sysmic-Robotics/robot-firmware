#include "odometry.h"
#include "motor.h"

void ForwardKinematics(float wheel_speeds_rads[4], float *vx, float *vy, float *vth)
{
    *vx = WHEEL_RADIO * (FK_00 * wheel_speeds_rads[0] +
                          FK_01 * wheel_speeds_rads[1] +
                          FK_02 * wheel_speeds_rads[2] +
                          FK_03 * wheel_speeds_rads[3]);

    *vy = WHEEL_RADIO * (FK_10 * wheel_speeds_rads[0] +
                          FK_11 * wheel_speeds_rads[1] +
                          FK_12 * wheel_speeds_rads[2] +
                          FK_13 * wheel_speeds_rads[3]);

    *vth = WHEEL_RADIO * (FK_20 * wheel_speeds_rads[0] +
                           FK_21 * wheel_speeds_rads[1] +
                           FK_22 * wheel_speeds_rads[2] +
                           FK_23 * wheel_speeds_rads[3]);
}
