#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>

/* Position controller gains (tune on hardware) */
#define TRAJ_KP_LINEAR  2.0f   /* (m/s) per meter of position error */
#define TRAJ_KP_ANGULAR 4.0f   /* (rad/s) per radian of heading error */
#define TRAJ_POS_DEADBAND  0.01f  /* m — stop when within this radius */
#define TRAJ_HEAD_DEADBAND 0.02f  /* rad — stop rotating when within this */

/* Compute body-frame velocity commands (vx, vy, vth) that drive the robot
 * from its current pose (x_cur, y_cur, theta_cur) toward the waypoint
 * (x_target, y_target, theta_target). Outputs are saturated to
 * v_max_linear (m/s) and v_max_angular (rad/s). Returns 1 when the waypoint
 * is reached (within deadbands), 0 otherwise. */
uint8_t Trajectory_ComputeVelocity(float x_cur, float y_cur, float theta_cur,
                                   float x_target, float y_target, float theta_target,
                                   float v_max_linear, float v_max_angular,
                                   float *vx_out, float *vy_out, float *vth_out);

#endif
