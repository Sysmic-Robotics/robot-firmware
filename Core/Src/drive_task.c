#include "system_globals.h"
#include "drive_task.h"
#include "motor.h"
#include "odometry.h"
#include "MAX581x.h"
#include "ekf.h"
#include "trajectory.h"
#include "lowpass_iir.h"
#include <math.h>
#include <string.h>
#include "board.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef WHEEL_MAX_SPEED_RAD
#define WHEEL_MAX_SPEED_RAD (M_PI * 40.0f)
#endif
#ifndef PID_SAMPLE_TIME
#define PID_SAMPLE_TIME 1
#endif

#define ROBOT_PID_SAMPLE_TIME 0.005f  // robot-level loop at 200 Hz

void DriveFunction(void *argument)
{
    // Init robot_id
    robot_id = Board_GetID();

    // Init wheels motors DAC: 2.0[V] ref
    MAX581x_Handler_t driveDAC;
    MAX581x_Init(&driveDAC, &hi2c1, MAX581x_REF_20);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_A, 0.0);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_B, 0.0);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_C, 0.0);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_D, 0.0);

    // Init dribbler motor DAC: 2.0[V] ref
    MAX581x_Handler_t dribblerDAC;
    MAX581x_Init(&dribblerDAC, &hi2c2, MAX581x_REF_20);
    MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, 0.0);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_6, GPIO_PIN_SET);

    // Config motors GPIO and TIM
    // Motor 1
    motor[0].enablePin.GPIOx = GPIOA;
    motor[0].enablePin.GPIO_Pin = GPIO_PIN_10;
    motor[0].dirPin.GPIOx = GPIOA;
    motor[0].dirPin.GPIO_Pin = GPIO_PIN_9;
    motor[0].brakePin.GPIOx = GPIOA;
    motor[0].brakePin.GPIO_Pin = GPIO_PIN_8;
    motor[0].encoder.count = &TIM3->CNT;
    motor[0].encoder.lastRaw = (uint16_t)TIM3->CNT;
    motor[0].encoder.enable = ENCODER_STATUS_ENABLE;
    TIM3->CR1 = TIM_CR1_CEN;

    // Motor 2
    motor[1].enablePin.GPIOx = GPIOC;
    motor[1].enablePin.GPIO_Pin = GPIO_PIN_11;
    motor[1].dirPin.GPIOx = GPIOC;
    motor[1].dirPin.GPIO_Pin = GPIO_PIN_12;
    motor[1].brakePin.GPIOx = GPIOD;
    motor[1].brakePin.GPIO_Pin = GPIO_PIN_0;
    motor[1].encoder.count = &TIM8->CNT;
    motor[1].encoder.lastRaw = (uint16_t)TIM8->CNT;
    motor[1].encoder.enable = ENCODER_STATUS_ENABLE;
    TIM8->CR1 = TIM_CR1_CEN;

    // Motor 3
    motor[2].enablePin.GPIOx = GPIOK;
    motor[2].enablePin.GPIO_Pin = GPIO_PIN_7;
    motor[2].dirPin.GPIOx = GPIOG;
    motor[2].dirPin.GPIO_Pin = GPIO_PIN_15;
    motor[2].brakePin.GPIOx = GPIOK;
    motor[2].brakePin.GPIO_Pin = GPIO_PIN_6;
    motor[2].encoder.count = &TIM2->CNT;
    motor[2].encoder.lastRaw = (uint16_t)TIM2->CNT;
    motor[2].encoder.enable = ENCODER_STATUS_ENABLE;
    TIM2->CR1 = TIM_CR1_CEN;

    // Motor 4
    motor[3].enablePin.GPIOx = GPIOF;
    motor[3].enablePin.GPIO_Pin = GPIO_PIN_5;
    motor[3].dirPin.GPIOx = GPIOF;
    motor[3].dirPin.GPIO_Pin = GPIO_PIN_3;
    motor[3].brakePin.GPIOx = GPIOF;
    motor[3].brakePin.GPIO_Pin = GPIO_PIN_4;
    motor[3].encoder.count = &TIM5->CNT;
    motor[3].encoder.lastRaw = (uint16_t)TIM5->CNT;
    motor[3].encoder.enable = ENCODER_STATUS_ENABLE;
    TIM5->CR1 = TIM_CR1_CEN;

    // Enable motors and disable brake with per-motor PID params
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Init(&motor[i], i, MOTOR_STATUS_ENABLE);
        Motor_SetBrake(&motor[i], MOTOR_BRAKE_DISABLE);
        PID_Init(&motor[i].pid, wheel_pid_params[i], PID_STATUS_DISABLE);

        motor[i].ffVoltage = 50.0f;
        motor[i].ffFrictionGain = 150.0f;
        motor[i].ffVelocityGain = 35.0f;
        motor[i].speedDeadband = WHEEL_MAX_SPEED_RAD * WHEEL_GEAR_RATIO * 0.005f;
    }


    PID_Params_t robotPidParams = {0};
    // Config robot-level PIDs (vx, vy, vth)

    robotPidParams.Kp = 1.5f;
    robotPidParams.Ki = 0.00f;
    robotPidParams.Kd = 0.0f;
    robotPidParams.outputMax = 25.0f;
    robotPidParams.outputMin = -25.0f;
    robotPidParams.integralMax = 1.5f;
    robotPidParams.sampleTime = ROBOT_PID_SAMPLE_TIME;

    PID_Init(&pid_vx, robotPidParams, PID_STATUS_DISABLE);
    PID_Init(&pid_vy, robotPidParams, PID_STATUS_DISABLE);

    robotPidParams.Kp = 0.8f;
    robotPidParams.Ki = 0.0f;
    robotPidParams.outputMax = 10.0f;
    robotPidParams.outputMin = -10.0f;
    robotPidParams.integralMax = 10.0f;
    PID_Init(&pid_vth, robotPidParams, PID_STATUS_DISABLE);

    robot_pid_enable = 0;

    // ── Init IIR filters for encoder noise ──
    for (uint8_t i = 0; i < 4; i++)
        LowPassFilter_Init(&motor[i].lpFilt, 0.25f);

    /* EKF init deferred to first cycle */

    // ── Rate limiter state (function scope so timeout handler can reset) ──
    static float vx_cmd_limited = 0.0f;
    static float vy_cmd_limited = 0.0f;
    static float vth_cmd_limited = 0.0f;

    // ── Velocity reference (written by 200 Hz position loop, consumed by
    //    1 kHz velocity PID) and final velocity command (1 kHz) ──
    static float vx_cmd_ref = 0.0f;
    static float vy_cmd_ref = 0.0f;
    static float vth_cmd_ref = 0.0f;
    static float vx_cmd = 0.0f;
    static float vy_cmd = 0.0f;
    static float vth_cmd = 0.0f;

    // ── Start TIM7 (1 kHz control loop timebase) ──
    HAL_TIM_Base_Start_IT(&htim7);

    // ── Enable DWT cycle counter for actual dt measurement ──
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t prev_cycle = DWT->CYCCNT;

    // Infinite loop
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


        uint32_t now_cycle = DWT->CYCCNT;
        float dt = (float)(now_cycle - prev_cycle) / (float)SystemCoreClock;
        prev_cycle = now_cycle;
        if (dt < 0.0001f) dt = 0.0001f;
        if (dt > 0.005f) dt = 0.005f;

        for (uint8_t i = 0; i < 4; i++)
            motor[i].pid.params.sampleTime = dt;

        // Velocity PIDs now run at 1 kHz (encoder-based); keep their sampleTime
        // in sync with the actual loop dt so integral/slew-rate are correct.
        pid_vx.params.sampleTime = dt;
        pid_vy.params.sampleTime = dt;
        pid_vth.params.sampleTime = dt;

        // ── Radio watchdog: coast motors on timeout (skip in test mode) ──
        uint32_t radio_now = HAL_GetTick();
        if (!test_enable && radio_ever_received && radio_now - radio_last_rx_tick > RADIO_TIMEOUT_MS) {
            uint8_t any_enabled = 0;
            for (uint8_t i = 0; i < 4; i++) {
                if (motor[i].enable == MOTOR_STATUS_ENABLE) any_enabled = 1;
            }
            if (any_enabled) {
                for (uint8_t i = 0; i < 4; i++) {
                    Motor_Enable(&motor[i], MOTOR_STATUS_DISABLE);
                }
                robot_pid_enable = 0;
                v_vel[0] = 0.0f;
                v_vel[1] = 0.0f;
                v_vel[2] = 0.0f;
                vx_cmd_limited = 0.0f;
                vy_cmd_limited = 0.0f;
                vth_cmd_limited = 0.0f;
                vx_cmd_ref = 0.0f;
                vy_cmd_ref = 0.0f;
                vth_cmd_ref = 0.0f;
                vx_cmd = 0.0f;
                vy_cmd = 0.0f;
                vth_cmd = 0.0f;
            }
        } else {
            uint8_t all_disabled = 1;
            for (uint8_t i = 0; i < 4; i++) {
                if (motor[i].enable == MOTOR_STATUS_ENABLE) all_disabled = 0;
            }
            if (all_disabled) {
                for (uint8_t i = 0; i < 4; i++) {
                    Motor_Enable(&motor[i], MOTOR_STATUS_ENABLE);
                }
                robot_pid_enable = 1;
            }
        }

        float wheel_speeds_rads[4];

        /* Pass 1: encoder + PID + GPIO dir (no I2C) */
        for (uint8_t i = 0; i < 4; i++)
        {
            if (test_enable)
            {
                if (i != test_motor)
                {
                    Motor_Enable(&motor[i], MOTOR_STATUS_DISABLE);
                    Motor_SetBrake(&motor[i], MOTOR_BRAKE_ENABLE);
                    motor[i].voltage = 0;
                    motor[i].measSpeed = 0.0f;
                    wheel_speeds_rads[i] = 0.0f;
                }
                else
                {
                    Motor_Enable(&motor[i], MOTOR_STATUS_ENABLE);
                    Motor_SetBrake(&motor[i], MOTOR_BRAKE_DISABLE);
                    if (test_voltage >= 0.0f)
                        HAL_GPIO_WritePin(motor[i].dirPin.GPIOx, motor[i].dirPin.GPIO_Pin, GPIO_PIN_SET);
                    else
                        HAL_GPIO_WritePin(motor[i].dirPin.GPIOx, motor[i].dirPin.GPIO_Pin, GPIO_PIN_RESET);
                    float rawSpeed = Encoder_Update(&motor[i].encoder, motor[i].pid.params.sampleTime);
                    motor[i].measSpeed = LowPassFilter_Update(&motor[i].lpFilt, rawSpeed);
                    motor[i].voltage = (uint16_t)(fabsf(test_voltage));
                    wheel_speeds_rads[i] = motor[i].measSpeed / WHEEL_GEAR_RATIO;
                }
            }
            else
            {
                if (!motor[i].enable)
                {
                    motor[i].voltage = 0;
                    wheel_speeds_rads[i] = 0.0f;
                    continue;
                }
                float rawSpeed = Encoder_Update(&motor[i].encoder, motor[i].pid.params.sampleTime);
                motor[i].measSpeed = LowPassFilter_Update(&motor[i].lpFilt, rawSpeed);
                wheel_speeds_rads[i] = motor[i].measSpeed / WHEEL_GEAR_RATIO;

                motor[i].refSpeed = speed[i] * SPEED_CNT_RATIO;

                float ff = 0.0f;
                if (fabsf(motor[i].refSpeed) > motor[i].speedDeadband)
                {
                    float sign = (motor[i].refSpeed > 0.0f) ? 1.0f : -1.0f;
                    ff += motor[i].ffFrictionGain * sign;
                    ff += motor[i].ffVelocityGain * motor[i].refSpeed;
                }

                PID_CloseLoop(&motor[i].pid, motor[i].refSpeed, motor[i].measSpeed);

                float out = motor[i].pid.output + ff;
                float absOut = fabsf(out);
                if (absOut > 4095.0f) absOut = 4095.0f;
                motor[i].voltage = (uint16_t)(absOut);

                if (out >= 0.0f)
                    HAL_GPIO_WritePin(motor[i].dirPin.GPIOx, motor[i].dirPin.GPIO_Pin, GPIO_PIN_SET);
                else
                    HAL_GPIO_WritePin(motor[i].dirPin.GPIOx, motor[i].dirPin.GPIO_Pin, GPIO_PIN_RESET);
            }
        }

        /* Pass 2: DAC writes via DMA (non-blocking I2C) */
        for (uint8_t i = 0; i < 4; i++)
        {
            MAX581x_CodeLoad(&driveDAC, i, motor[i].voltage);
        }

        // ── Dribbler DAC ──
        // Write only when the speed index changes: MAX581x_Code() is a blocking
        // I2C transaction (~75-100 us) that would add jitter to the 1 kHz loop
        // if executed every cycle. The DAC holds its output until rewritten.
        static uint8_t last_dribbler_sel = 0xFF;
        if (dribbler_sel != last_dribbler_sel)
        {
            MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, Dribbler_SpeedSet[dribbler_sel]);
            last_dribbler_sel = dribbler_sel;
        }

        // ── Robot velocity control (1 kHz, encoder-based) ──
        // Forward kinematics from wheel speeds (encoders) at 1 kHz. The
        // velocity PID closes the loop against encoder-derived velocity, so it
        // is not limited by the 200 Hz MPU6050/EKF rate. The 200 Hz position
        // loop writes the velocity reference (vx_cmd_ref/vy_cmd_ref/vth_cmd_ref).
        ForwardKinematics(wheel_speeds_rads, &vx_enc, &vy_enc, &vth_enc);

        if (robot_pid_enable)
        {
            // Per-axis PID passthrough gates (open-loop characterization):
            // *_enable == 1 -> closed-loop PID; == 0 -> raw reference passthrough
            // for that axis only (other axes stay closed-loop).
            if (pid_vx_enable)
            {
                if (fabsf(vx_cmd_ref) < 0.01f && fabsf(vx_enc) < 0.01f) pid_vx.integral = 0.0f;
                PID_CloseLoop(&pid_vx, vx_cmd_ref, vx_enc);
                vx_cmd = pid_vx.output;
            }
            else
            {
                vx_cmd = vx_cmd_ref;
            }

            if (pid_vy_enable)
            {
                if (fabsf(vy_cmd_ref) < 0.01f && fabsf(vy_enc) < 0.01f) pid_vy.integral = 0.0f;
                PID_CloseLoop(&pid_vy, vy_cmd_ref, vy_enc);
                vy_cmd = pid_vy.output;
            }
            else
            {
                vy_cmd = vy_cmd_ref;
            }

            // ── Vth PID: angular velocity control (omega_cmd → omega) ──
            if (pid_vth_enable)
            {
                if (fabsf(vth_cmd_ref) < 0.01f && fabsf(vth_enc) < 0.01f) pid_vth.integral = 0.0f;
                PID_CloseLoop(&pid_vth, vth_cmd_ref, vth_enc);
                vth_cmd = pid_vth.output;

                // Slew-rate limit on the angular output (1 kHz step)
                float vth_delta = vth_cmd - vth_cmd_limited;
                float max_vth_step = robot_max_angular_acc * dt;
                if (vth_delta > max_vth_step) vth_delta = max_vth_step;
                else if (vth_delta < -max_vth_step) vth_delta = -max_vth_step;
                vth_cmd_limited += vth_delta;
                vth_cmd = vth_cmd_limited;
            }
            else
            {
                vth_cmd = vth_cmd_ref;
            }
        }
        else
        {
            vx_cmd = vx_cmd_ref;
            vy_cmd = vy_cmd_ref;
            vth_cmd = vth_cmd_ref;
        }

        // Inverse kinematics → wheel speed commands
        for (uint8_t i = 0; i < 4; i++)
        {
            speed[i] = kinematic[i][0] * vx_cmd +
                       kinematic[i][1] * vy_cmd +
                       kinematic[i][2] * vth_cmd;
        }

        // ── Robot-level control (200 Hz) ──
        static uint32_t robot_cycle = 0;
        robot_cycle++;
        if (robot_cycle >= 5)
        {
            robot_cycle = 0;

            if (!kalman_ready) {
                EKF_Init(&ekf, ROBOT_PID_SAMPLE_TIME);
                kalman_ready = 1;
            }

            EKF_Predict(&ekf, ax_ms2, ay_ms2, gyro_z_rads);
            EKF_UpdateEncoders(&ekf, vx_enc, vy_enc, vth_enc);
            EKF_SaveHistory(&ekf);

            vx_est  = ekf.x[4];
            vy_est  = ekf.x[5];
            vth_est = ekf.x[6];
            theta_est = atan2f(ekf.x[2], ekf.x[3]);

            /* Position control: drive toward waypoint → velocity reference.
             * The velocity PID (1 kHz, encoder-based) closes the loop against
             * this reference; the position loop only updates the reference at
             * 200 Hz (EKF rate). */
            if (position_control_active)
            {
                float tvx, tvy, tvth;
                uint8_t reached = Trajectory_ComputeVelocity(
                    ekf.x[0], ekf.x[1], theta_est,
                    x_target, y_target, theta_target,
                    ROBOT_MAX_LINEAR_VEL, robot_max_angular_acc,
                    &tvx, &tvy, &tvth);
                vx_cmd_ref = tvx;
                vy_cmd_ref = tvy;
                vth_cmd_ref = tvth;
                if (reached)
                {
                    vx_cmd_ref = 0.0f;
                    vy_cmd_ref = 0.0f;
                    vth_cmd_ref = 0.0f;
                    position_control_active = 0;
                }
            }
            else
            {
                vx_cmd_ref = v_vel[0];
                vy_cmd_ref = v_vel[1];
                vth_cmd_ref = v_vel[2];
            }

            // ── Rate limiters for linear axes (internal ramps) on the reference ──
            {
                float max_lin_step = robot_max_linear_acc * ROBOT_PID_SAMPLE_TIME;

                float vx_delta = vx_cmd_ref - vx_cmd_limited;
                if (vx_delta > max_lin_step) vx_delta = max_lin_step;
                else if (vx_delta < -max_lin_step) vx_delta = -max_lin_step;
                vx_cmd_limited += vx_delta;
                vx_cmd_ref = vx_cmd_limited;

                float vy_delta = vy_cmd_ref - vy_cmd_limited;
                if (vy_delta > max_lin_step) vy_delta = max_lin_step;
                else if (vy_delta < -max_lin_step) vy_delta = -max_lin_step;
                vy_cmd_limited += vy_delta;
                vy_cmd_ref = vy_cmd_limited;
            }
        }

        // ── Kicker ──
        if(kick_sel && kick_flag == KICKER_CHARGED
           /* && ball_possession */) {  // TODO: habilitar detector de pelota
            osMutexAcquire(kickFlagHandle, osWaitForever);
            kick_flag = KICKER_START;
            osMutexRelease(kickFlagHandle);
            uint32_t _m0 = 0;
            osMessageQueuePut(kickQueueHandle, &_m0, 0U, 0U);
            kick_sel = 0;  // Reset para evitar re-kick automático
        }

        uint32_t _m1 = 0;
        osMessageQueuePut(nrf24CheckHandle, &_m1, 0U, 0U);

    }
}

void processRadioCommand(uint8_t *buffer)
{
	/* Config mode: callback bit + mode bit both set → register write over radio.
	 * The engine never sets both simultaneously (test mode = bit0 only,
	 * waypoint = bit2 only), so this combination is a safe config channel. */
	if ((buffer[0] & 0x01) && (buffer[0] & 0x04))
	{
		uint8_t reg = buffer[1];
		uint8_t val = buffer[2];
		switch (reg) {
			case 0x01:  /* telemetry_calib_mode */
				telemetry_calib_mode = val ? 1 : 0;
				break;
			default:
				break;
		}
		return;
	}

	/* Test mode: callback bit = 1 → decode motor_idx + voltage instead of velocities */
	if (buffer[0] & 0x01)
	{
		test_enable = 1;
		test_motor = (buffer[2] >> 6) & 0x03;
		uint16_t abs_v = ((uint16_t)(buffer[1] & 0x7F) << 5) | (uint16_t)(buffer[2] & 0x1F);
		test_voltage = (buffer[1] & 0x80) ? -(float)abs_v : (float)abs_v;
		return;
	}

	/* Normal command: disable test mode and re-enable all motors.
	 * During test mode only test_motor stays enabled; the others are
	 * explicitly disabled.  If we just clear test_enable, the drive-loop
	 * "all_disabled" guard never fires (motor[0] is still enabled), so
	 * motors 1-3 remain disabled and spin at 0 V forever. */
	if (test_enable) {
		for (uint8_t i = 0; i < 4; i++) {
			Motor_Enable(&motor[i], MOTOR_STATUS_ENABLE);
			Motor_SetBrake(&motor[i], MOTOR_BRAKE_DISABLE);
		}
		robot_pid_enable = 1;
	}
	test_enable = 0;

	/* Position waypoint mode (bit 2): decode relative offsets (ΔX, ΔY, Δθ) */
	if (buffer[0] & 0x04)
	{
		int16_t dx_raw = (buffer[1] & 0x80)
		    ? -(int16_t)(((uint16_t)(buffer[4] & 0xC0) << 1) | (uint16_t)(buffer[1] & 0x7F))
		    :  (int16_t)(((uint16_t)(buffer[4] & 0xC0) << 1) | (uint16_t)(buffer[1] & 0x7F));
		int16_t dy_raw = (buffer[2] & 0x80)
		    ? -(int16_t)(((uint16_t)(buffer[4] & 0x30) << 3) | (uint16_t)(buffer[2] & 0x7F))
		    :  (int16_t)(((uint16_t)(buffer[4] & 0x30) << 3) | (uint16_t)(buffer[2] & 0x7F));
		int16_t dtheta_raw = (buffer[3] & 0x80)
		    ? -(int16_t)(((uint16_t)(buffer[4] & 0x0F) << 7) | (uint16_t)(buffer[3] & 0x7F))
		    :  (int16_t)(((uint16_t)(buffer[4] & 0x0F) << 7) | (uint16_t)(buffer[3] & 0x7F));

		/* Waypoint absolute target = current EKF pose + relative offset */
		x_target = ekf.x[0] + (float)dx_raw / 100.0f;
		y_target = ekf.x[1] + (float)dy_raw / 100.0f;
		theta_target = theta_est + (float)dtheta_raw / 100.0f;
		position_control_active = 1;
		return;
	}

	/* Velocity mode: disable position control */
	position_control_active = 0;

	/* Last velocities */
	float prv_Vx = v_vel[0], prv_Vy = v_vel[1];

	/* Decode vx, vy from radio packet (10-bit signed / 100) */
	v_vel[0] = (buffer[1] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0xC0) << 1 | (uint16_t)(buffer[1] & 0x7F))/100.0f  : (float)((uint16_t)(buffer[4] & 0xC0) << 1 | (uint16_t)(buffer[1] & 0x7F))/100.0f ;
	v_vel[1] = (buffer[2] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0x30) << 3 | (uint16_t)(buffer[2] & 0x7F))/100.0f : (float)((uint16_t)(buffer[4] & 0x30) << 3 | (uint16_t)(buffer[2] & 0x7F))/100.0f ;

	/* Decode omega (vth) from radio packet (10-bit signed, scale 100 → rad/s) */
	{
	    int16_t vth_raw = (buffer[3] & 0x80)
	        ? -(int16_t)(((uint16_t)(buffer[4] & 0x0F) << 7) | (uint16_t)(buffer[3] & 0x7F))
	        :  (int16_t)(((uint16_t)(buffer[4] & 0x0F) << 7) | (uint16_t)(buffer[3] & 0x7F));
	    v_vel[2] = (float)vth_raw / 100.0f;  /* omega (rad/s) for angular velocity PID */
	}

	/* Limit linear velocity magnitude */
	float vel_mag = sqrtf(v_vel[0]*v_vel[0] + v_vel[1]*v_vel[1]);
	if (vel_mag > ROBOT_MAX_LINEAR_VEL)
	{
		float scale = ROBOT_MAX_LINEAR_VEL / vel_mag;
		v_vel[0] *= scale;
		v_vel[1] *= scale;
	}

	/* Limit linear acceleration */
	float Ax = v_vel[0] - prv_Vx, Ay = v_vel[1] - prv_Vy;
	float acc_mag = sqrtf(Ax*Ax + Ay*Ay);
	if (acc_mag > ROBOT_MAX_LINEAR_ACC && acc_mag > 1e-6f)
	{
		float scale = ROBOT_MAX_LINEAR_ACC / acc_mag;
		v_vel[0] = prv_Vx + Ax * scale;
		v_vel[1] = prv_Vy + Ay * scale;
	}

	/* speed[] will be computed in DriveFunction loop — do NOT write to velocity here */
}

/* Process a camera-correction packet from the Engine (18 bytes, Pipe 1).
 * Layout: [0]=0xFE header, [1]=robot id, [2..6]=x_cam f32 LE,
 * [6..10]=y_cam f32 LE, [10..14]=theta_cam f32 LE, [14..18]=age f32 LE.
 * The camera pose is in the global field frame; the local EKF is in the
 * odometry frame. We learn an alignment offset while the robot is at rest
 * and apply the correction in the local frame via EKF_UpdateEngineDelayed.
 */
void processCameraCorrection(uint8_t *buffer)
{
	if (buffer[0] != 0xFE || buffer[1] != robot_id)
		return;

	float x_cam, y_cam, theta_cam, age;
	memcpy(&x_cam,     &buffer[2],  sizeof(float));
	memcpy(&y_cam,     &buffer[6],  sizeof(float));
	memcpy(&theta_cam, &buffer[10], sizeof(float));
	memcpy(&age,       &buffer[14], sizeof(float));

	if (!camera_correction_enabled)
		return;

	/* Learn the local<->global alignment offset while the robot is at rest. */
	if (fabsf(vx_est) < 0.05f && fabsf(vy_est) < 0.05f)
	{
		offset_x = x_cam - ekf.x[0];
		offset_y = y_cam - ekf.x[1];
		offset_theta = theta_cam - theta_est;
	}

	float x_local = x_cam - offset_x;
	float y_local = y_cam - offset_y;
	float theta_local = theta_cam - offset_theta;
	float t_cam = ekf.time - age;

	EKF_UpdateEngineDelayed(&ekf, x_local, y_local, theta_local, t_cam,
	                        ax_ms2, ay_ms2, gyro_z_rads);
}
