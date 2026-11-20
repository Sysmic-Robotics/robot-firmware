#include "system_globals.h"
#include "drive_task.h"
#include "board.h"
#include "motor.h"
#include "MAX581x.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Definiciones de constantes si no están ya en system_globals.h
#ifndef WHEEL_MAX_SPEED_RAD
#define WHEEL_MAX_SPEED_RAD (M_PI * 40.0f) // Ajusta el valor según tu aplicación
#endif
#ifndef PID_SAMPLE_TIME
#define PID_SAMPLE_TIME 1 // Ajusta el valor según tu aplicación
#endif
#ifndef ENCODER_CPR
#define ENCODER_CPR 4096 // Ajusta el valor según tu aplicación
#endif

// Definiciones de estados del kicker si no están ya en main.h
#ifndef KICKER_CHARGED
#define KICKER_CHARGED 1
#endif
#ifndef KICKER_START
#define KICKER_START 2
#endif

/* Latest commanded velocity; written by the radio task under a critical section. */
static float target_velocity[3];
static int lateral_direction;
static float lateral_start_time;
static float lateral_tracking_time;

static void calibrateVelocity(float target[3], float dt)
{
    const float forward = target[0];
    const float lateral = target[1];
    /* Identified separately for forward and reverse on the 1 m course. */
    target[0] = 1.16f * forward;
    target[1] += (forward >= 0.0f ? 0.24f : 0.16f) * forward;
    target[2] -= (forward >= 0.0f ? 0.23f : 0.46f) * forward;

    const int direction = fabsf(lateral) >= 0.20f && fabsf(lateral) > fabsf(forward)
                          ? (lateral > 0.0f ? 1 : -1) : 0;
    if (direction != lateral_direction) {
        lateral_direction = direction;
        lateral_start_time = 0.0f;
        lateral_tracking_time = 0.0f;
    }
    if (direction != 0) {
        target[0] += 0.25f * lateral;
        target[2] -= (direction > 0 ? 0.77f : 0.28f) * lateral;
        /* End the breakaway pulse once three encoders track in the expected
           direction, or after the time limit even if the wheels are stalled. */
        if (lateral_start_time < 0.90f && lateral_tracking_time < 0.035f) {
            unsigned int tracking = 0;
            for (uint8_t i = 0; i < 4; ++i) {
                const float sign = (i == 0 || i == 3) ? direction : -direction;
                if (sign * motor[i].measSpeed > 3.0f) ++tracking;
            }
            lateral_start_time += dt;
            lateral_tracking_time = tracking >= 3 ? lateral_tracking_time + dt : 0.0f;
            if (fabsf(target[1]) < 1.0f) target[1] = (float)direction;
        }
    }

    const float magnitude = hypotf(target[0], target[1]);
    if (magnitude > ROBOT_MAX_LINEAR_VEL) {
        const float scale = ROBOT_MAX_LINEAR_VEL / magnitude;
        target[0] *= scale;
        target[1] *= scale;
    }
}

static void updateVelocity(float dt)
{
    float target[3];
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < 3; ++i) target[i] = target_velocity[i];
    taskEXIT_CRITICAL();
    calibrateVelocity(target, dt);
    float dx = target[0] - v_vel[0];
    float dy = target[1] - v_vel[1];
    float delta = sqrtf(dx * dx + dy * dy);
    float max_delta = ROBOT_MAX_LINEAR_ACC * dt;
    if (delta > max_delta && delta > 0.0f) {
        float scale = max_delta / delta;
        dx *= scale;
        dy *= scale;
    }
    v_vel[0] += dx;
    v_vel[1] += dy;
    v_vel[2] = target[2];
    for (uint8_t i = 0; i < 4; ++i) {
        float wheel = 0.0f;
        for (uint8_t j = 0; j < 3; ++j) wheel += kinematic[i][j] * v_vel[j];
        direction[i] = wheel > 0 ? WHEEL_P_ROTATION : WHEEL_N_ROTATION;
        speed[i] = wheel;
    }
}

void DriveFunction(void const * argument)
{
    // Init PID sampler
    uint32_t timeToWait = osKernelSysTick();
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
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_8, GPIO_PIN_RESET);
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
    motor[0].encoder.oldPos = TIM3->CNT / ENCODER_CPR;
    motor[0].encoder.enable = ENCODER_STATUS_ENABLE;
    motor[0].encoder.minSpeed = WHEEL_MAX_SPEED_RAD * 0.001f;
    TIM3->CR1 = TIM_CR1_CEN;

    // Motor 2
    motor[1].enablePin.GPIOx = GPIOC;
    motor[1].enablePin.GPIO_Pin = GPIO_PIN_11;
    motor[1].dirPin.GPIOx = GPIOC;
    motor[1].dirPin.GPIO_Pin = GPIO_PIN_12;
    motor[1].brakePin.GPIOx = GPIOD;
    motor[1].brakePin.GPIO_Pin = GPIO_PIN_0;
    motor[1].encoder.count = &TIM8->CNT;
    motor[1].encoder.oldPos = TIM8->CNT / ENCODER_CPR;
    motor[1].encoder.enable = ENCODER_STATUS_ENABLE;
    motor[1].encoder.minSpeed = WHEEL_MAX_SPEED_RAD * 0.001f;
    TIM8->CR1 = TIM_CR1_CEN;

    // Motor 3
    motor[2].enablePin.GPIOx = GPIOK;
    motor[2].enablePin.GPIO_Pin = GPIO_PIN_7;
    motor[2].dirPin.GPIOx = GPIOG;
    motor[2].dirPin.GPIO_Pin = GPIO_PIN_15;
    motor[2].brakePin.GPIOx = GPIOK;
    motor[2].brakePin.GPIO_Pin = GPIO_PIN_6;
    motor[2].encoder.count = &TIM2->CNT;
    motor[2].encoder.oldPos = TIM2->CNT / ENCODER_CPR;
    motor[2].encoder.enable = ENCODER_STATUS_ENABLE;
    motor[2].encoder.minSpeed = WHEEL_MAX_SPEED_RAD * 0.001f;
    TIM2->CR1 = TIM_CR1_CEN;

    // Motor 4
    motor[3].enablePin.GPIOx = GPIOF;
    motor[3].enablePin.GPIO_Pin = GPIO_PIN_5;
    motor[3].dirPin.GPIOx = GPIOF;
    motor[3].dirPin.GPIO_Pin = GPIO_PIN_3;
    motor[3].brakePin.GPIOx = GPIOF;
    motor[3].brakePin.GPIO_Pin = GPIO_PIN_4;
    motor[3].encoder.count = &TIM5->CNT;
    motor[3].encoder.oldPos = TIM5->CNT / ENCODER_CPR;
    motor[3].encoder.enable = ENCODER_STATUS_ENABLE;
    motor[3].encoder.minSpeed = WHEEL_MAX_SPEED_RAD * 0.001f;
    TIM5->CR1 = TIM_CR1_CEN;

    // Config PID
    PID_Params_t pidParams;
    pidParams.Kp = 12.0f;
    pidParams.Ki = 4.5f;
    pidParams.Kd = 0.0f;
    pidParams.outputMax = (float)(4095.0f);
    pidParams.outputMin = (float)(-4095.0f);
    pidParams.integralMax = pidParams.outputMax / 2.5f; /* experimento: mas autoridad para el lateral (antes /5) */
    pidParams.sampleTime = PID_SAMPLE_TIME / 1000.0f;

    // Enable motors and disable brake
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Init(&motor[i], i, MOTOR_STATUS_DISABLE);
        Motor_SetBrake(&motor[i], MOTOR_BRAKE_DISABLE);
        PID_Init(&motor[i].pid, pidParams, PID_STATUS_ENABLE);

        motor[i].ffVoltage = 300.0f;
        motor[i].speedDeadband = WHEEL_MAX_SPEED_RAD * 0.005f;
    }

    timeToWait = osKernelSysTick();
    uint32_t last_control_ms = HAL_GetTick();
    uint8_t active = 0;
    for (;;)
    {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed = now - last_control_ms;
        last_control_ms = now;
        /* Avoid a large step after a delayed control iteration. */
        if (elapsed > 50U) elapsed = 50U;
        taskENTER_CRITICAL();
        uint32_t check_time = HAL_GetTick();
        uint8_t valid = radio_command_received &&
                        (uint32_t)(check_time - radio_last_command_ms) < RADIO_COMMAND_TIMEOUT_MS;
        if (!valid) {
            lateral_direction = 0;
            radio_command_received = 0;
            dribbler_sel = 0;
            kick_sel = 0;
            for (uint8_t i = 0; i < 3; ++i) target_velocity[i] = 0.0f;
        }
        uint8_t dribbler = dribbler_sel;
        taskEXIT_CRITICAL();

        if (!valid) {
            if (active) {
                for (uint8_t i = 0; i < 4; ++i) Motor_Enable(&motor[i], MOTOR_STATUS_DISABLE);
                HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_8, GPIO_PIN_RESET);
                for (uint8_t i = 0; i < 4; ++i) {
                    speed[i] = 0.0f;
                    motor[i].pid.integral = 0.0f;
                    motor[i].pid.lastMeasure = 0.0f;
                    MAX581x_Code(&driveDAC, i, 0);
                }
                for (uint8_t i = 0; i < 3; ++i) v_vel[i] = 0.0f;
                MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, 0);
                active = 0;
            }
        } else {
            if (!active) {
                for (uint8_t i = 0; i < 4; ++i) {
                    /* Descarta las cuentas acumuladas mientras el robot estaba inactivo. */
                    *motor[i].encoder.count = 0;
                    Motor_Enable(&motor[i], MOTOR_STATUS_ENABLE);
                }
                HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_8, GPIO_PIN_SET);
                active = 1;
            }
            updateVelocity((float)elapsed / 1000.0f);
            for (uint8_t i = 0; i < 4; ++i) {
                motor[i].pid.params.integralMax = pidParams.outputMax /
                    (lateral_direction ? 2.2f : 2.5f);
                Motor_CLDrive(&motor[i], &driveDAC, speed[i]);
            }
            taskENTER_CRITICAL();
            for (uint8_t i = 0; i < 4; ++i) motor_telemetry.meas_sum[i] += motor[i].measSpeed;
            if (motor_telemetry.samples < UINT16_MAX) motor_telemetry.samples++;
            taskEXIT_CRITICAL();
            MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, Dribbler_SpeedSet[dribbler]);
            if (kick_sel && kick_flag == KICKER_CHARGED) {
                osMutexWait(kickFlagHandle, osWaitForever);
                kick_flag = KICKER_START;
                osMutexRelease(kickFlagHandle);
                osMessagePut(kickQueueHandle, 0, 0);
            }
        }
        motor_telemetry.active = active;
        osMessagePut(nrf24CheckHandle, 0, 0);
        osDelayUntil(&timeToWait, (uint32_t)PID_SAMPLE_TIME);
    }
}

void setSpeed(uint8_t *buffer, float *velocity, uint8_t *turn)
{
    (void)velocity;
    (void)turn;
    float vx = (float)(((uint16_t)(buffer[4] & 0xC0) << 1) | (buffer[1] & 0x7F)) / 100.0f;
    float vy = (float)(((uint16_t)(buffer[4] & 0x30) << 3) | (buffer[2] & 0x7F)) / 100.0f;
    float vr = (float)(((uint16_t)(buffer[4] & 0x0F) << 7) | (buffer[3] & 0x7F)) / 100.0f;
    if (buffer[1] & 0x80) vx = -vx;
    if (buffer[2] & 0x80) vy = -vy;
    if (buffer[3] & 0x80) vr = -vr;
    float magnitude = sqrtf(vx * vx + vy * vy);
    if (magnitude > ROBOT_MAX_LINEAR_VEL) {
        float scale = ROBOT_MAX_LINEAR_VEL / magnitude;
        vx *= scale;
        vy *= scale;
    }
    target_velocity[0] = vx;
    target_velocity[1] = vy;
    target_velocity[2] = vr;
}
