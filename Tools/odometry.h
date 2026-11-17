#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include <stdint.h>

/* Forward kinematics matrix (pseudoinverse) for wheel angles [60°, 130°, 230°, 300°]
 * Maps wheel speeds [rad/s] to robot velocity [vx, vy, vth]
 * K_FK = (A^T * A)^{-1} * A^T
 * where A[i,:] = [-sin(θ_i), cos(θ_i), ROBOT_RADIO]
 *
 * Nota: Los coeficientes vth NO son iguales porque los ángulos no son simétricos.
 * Ruedas 1,4 (60°, 300°) contribuyen más a vth que ruedas 2,3 (130°, 230°).
 */
#define FK_00  (-0.3239f)
#define FK_01  (-0.2865f)
#define FK_02   0.2865f
#define FK_03   0.3239f

#define FK_10   0.4375f
#define FK_11  (-0.4375f)
#define FK_12  (-0.4375f)
#define FK_13   0.4375f

#define FK_20   3.4235f
#define FK_21   2.6630f
#define FK_22   2.6630f
#define FK_23   3.4235f

/* Forward kinematics: wheel speeds [rad/s] -> robot velocity [m/s, m/s, rad/s]
 * vx_est = WHEEL_RADIO * (FK_00*w0 + FK_01*w1 + FK_02*w2 + FK_03*w3)
 * vy_est = WHEEL_RADIO * (FK_10*w0 + FK_11*w1 + FK_12*w2 + FK_13*w3)
 * vth_est = WHEEL_RADIO * (FK_20*w0 + FK_21*w1 + FK_22*w2 + FK_23*w3)
 */
void ForwardKinematics(float wheel_speeds_rads[4], float *vx, float *vy, float *vth);

#endif
