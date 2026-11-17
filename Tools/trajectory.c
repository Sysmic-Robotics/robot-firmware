#include "trajectory.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float normalize_angle(float a)
{
    while (a > M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

uint8_t Trajectory_ComputeVelocity(float x_cur, float y_cur, float theta_cur,
                                   float x_target, float y_target, float theta_target,
                                   float v_max_linear, float v_max_angular,
                                   float *vx_out, float *vy_out, float *vth_out)
{
    float ex = x_target - x_cur;
    float ey = y_target - y_cur;
    float c = cosf(theta_cur);
    float s = sinf(theta_cur);

    /* Position error in robot body frame */
    float ex_body =  c * ex + s * ey;
    float ey_body = -s * ex + c * ey;

    float dist = sqrtf(ex * ex + ey * ey);
    float ehead = normalize_angle(theta_target - theta_cur);

    float vx = TRAJ_KP_LINEAR * ex_body;
    float vy = TRAJ_KP_LINEAR * ey_body;
    float vth = TRAJ_KP_ANGULAR * ehead;

    /* Saturate linear velocity magnitude */
    float vm = sqrtf(vx * vx + vy * vy);
    if (vm > v_max_linear && vm > 1e-6f)
    {
        float scale = v_max_linear / vm;
        vx *= scale;
        vy *= scale;
    }

    /* Saturate angular velocity */
    if (vth > v_max_angular) vth = v_max_angular;
    else if (vth < -v_max_angular) vth = -v_max_angular;

    *vx_out = vx;
    *vy_out = vy;
    *vth_out = vth;

    /* Waypoint reached when within both deadbands */
    return (dist < TRAJ_POS_DEADBAND && fabsf(ehead) < TRAJ_HEAD_DEADBAND) ? 1 : 0;
}
