#include "system_globals.h"
#include "drive_task.h"
#include "motor.h"
#include "MAX581x.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===== Ajustes (puedes calibrar estos) ===== */
#ifndef WHEEL_MAX_SPEED_RAD
#define WHEEL_MAX_SPEED_RAD (M_PI * 40.0f)
#endif
#ifndef PID_SAMPLE_TIME
#define PID_SAMPLE_TIME 1
#endif
#ifndef ENCODER_CPR
#define ENCODER_CPR 4096
#endif

/* Deadband para considerar “cero” de velocidad de rueda (en m/s, porque speed[] viene en m/s) */
#ifndef WHEEL_ZERO_EPS_MPS
#define WHEEL_ZERO_EPS_MPS (0.02f)  /* ajusta según tu hardware */
#endif

/* ---- Kicker (por compatibilidad con el código original) ---- */
#ifndef KICKER_CHARGED
#define KICKER_CHARGED 1
#endif
#ifndef KICKER_START
#define KICKER_START 2
#endif

/* ===== Helpers ===== */

/* Devuelve el canal DAC asociado a un motor, usando motor[i].outputID si está configurado por Motor_Init().
   Si no está seteado, cae en un mapeo por índice. */
static inline uint8_t Drive_GetDacOutputID(uint8_t i) {
    uint8_t id = motor[i].outputID;
    switch (id) {
        case MAX581x_OUTPUT_A:
        case MAX581x_OUTPUT_B:
        case MAX581x_OUTPUT_C:
        case MAX581x_OUTPUT_D:
            return id;
        default:
            /* fallback razonable por índice */
            switch (i) {
                case 0: return MAX581x_OUTPUT_A;
                case 1: return MAX581x_OUTPUT_B;
                case 2: return MAX581x_OUTPUT_C;
                default: return MAX581x_OUTPUT_D;
            }
    }
}

/* Fuerza el canal DAC del motor i a 0 (evita corriente residual) */
static inline void Drive_DacZeroForMotor(MAX581x_Handler_t *dac, uint8_t i) {
    MAX581x_Code(dac, Drive_GetDacOutputID(i), 0.0);
}

/* Fija un estado de DIR “seguro” cuando está parado (ajústalo si tu hardware requiere HIGH) */
static inline void Drive_SetSafeDir(uint8_t i) {
    HAL_GPIO_WritePin(motor[i].dirPin.GPIOx, motor[i].dirPin.GPIO_Pin, GPIO_PIN_RESET);
}

/* (Opcional) Reset básico del integrador del PID para evitar creep al volver a cero */
static inline void Drive_ResetPidIntegrator(uint8_t i) {
    /* Si tu PID_Handler_t expone estos campos, descomenta; si tienes una función PID_Reset(), úsala en su lugar. */
    #ifdef PID_Reset
        PID_Reset(&motor[i].pid);
    #else
        motor[i].pid.integral = 0.0f;
        motor[i].pid.lastMeasure = 0.0f;
    #endif
}

void DriveFunction(void const * argument)
{
    /* Init PID sampler */
    uint32_t timeToWait = osKernelSysTick();
    /* ID del robot */
    robot_id = Board_GetID();

    /* ===== DAC de ruedas en I2C1 ===== */
    MAX581x_Handler_t driveDAC;
    MAX581x_Init(&driveDAC, &hi2c1, MAX581x_REF_20);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_A, 0.0);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_B, 0.0);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_C, 0.0);
    MAX581x_Code(&driveDAC, MAX581x_OUTPUT_D, 0.0);

    /* ===== DAC del dribbler en I2C2 ===== */
    MAX581x_Handler_t dribblerDAC;
    MAX581x_Init(&dribblerDAC, &hi2c2, MAX581x_REF_20);
    MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, 0.0);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_6, GPIO_PIN_SET);

    /* ===== Config GPIO/ENCODER (igual al original) ===== */
    /* Motor 1 */
    motor[0].enablePin.GPIOx = GPIOA; motor[0].enablePin.GPIO_Pin = GPIO_PIN_10;
    motor[0].dirPin.GPIOx    = GPIOA; motor[0].dirPin.GPIO_Pin    = GPIO_PIN_9;
    motor[0].brakePin.GPIOx  = GPIOA; motor[0].brakePin.GPIO_Pin  = GPIO_PIN_8;
    motor[0].encoder.count   = &TIM3->CNT;
    motor[0].encoder.oldPos  = TIM3->CNT / ENCODER_CPR;
    motor[0].encoder.enable  = ENCODER_STATUS_ENABLE;
    motor[0].encoder.minSpeed= WHEEL_MAX_SPEED_RAD * 0.001f;
    TIM3->CR1 = TIM_CR1_CEN;

    /* Motor 2 */
    motor[1].enablePin.GPIOx = GPIOC; motor[1].enablePin.GPIO_Pin = GPIO_PIN_11;
    motor[1].dirPin.GPIOx    = GPIOC; motor[1].dirPin.GPIO_Pin    = GPIO_PIN_12;
    motor[1].brakePin.GPIOx  = GPIOD; motor[1].brakePin.GPIO_Pin  = GPIO_PIN_0;
    motor[1].encoder.count   = &TIM8->CNT;
    motor[1].encoder.oldPos  = TIM8->CNT / ENCODER_CPR;
    motor[1].encoder.enable  = ENCODER_STATUS_ENABLE;
    motor[1].encoder.minSpeed= WHEEL_MAX_SPEED_RAD * 0.001f;
    TIM8->CR1 = TIM_CR1_CEN;

    /* Motor 3 */
    motor[2].enablePin.GPIOx = GPIOK; motor[2].enablePin.GPIO_Pin = GPIO_PIN_7;
    motor[2].dirPin.GPIOx    = GPIOG; motor[2].dirPin.GPIO_Pin    = GPIO_PIN_15;
    motor[2].brakePin.GPIOx  = GPIOK; motor[2].brakePin.GPIO_Pin  = GPIO_PIN_6;
    motor[2].encoder.count   = &TIM2->CNT;
    motor[2].encoder.oldPos  = TIM2->CNT / ENCODER_CPR;
    motor[2].encoder.enable  = ENCODER_STATUS_ENABLE;
    motor[2].encoder.minSpeed= WHEEL_MAX_SPEED_RAD * 0.001f;
    TIM2->CR1 = TIM_CR1_CEN;

    /* Motor 4 */
    motor[3].enablePin.GPIOx = GPIOF; motor[3].enablePin.GPIO_Pin = GPIO_PIN_5;
    motor[3].dirPin.GPIOx    = GPIOF; motor[3].dirPin.GPIO_Pin    = GPIO_PIN_3;
    motor[3].brakePin.GPIOx  = GPIOF; motor[3].brakePin.GPIO_Pin  = GPIO_PIN_4;
    motor[3].encoder.count   = &TIM5->CNT;
    motor[3].encoder.oldPos  = TIM5->CNT / ENCODER_CPR;
    motor[3].encoder.enable  = ENCODER_STATUS_ENABLE;
    motor[3].encoder.minSpeed= WHEEL_MAX_SPEED_RAD * 0.001f;
    TIM5->CR1 = TIM_CR1_CEN;

    /* ===== PID ===== */
    PID_Params_t pidParams;
    pidParams.Kp          = 12.0f;
    pidParams.Ki          = 4.5f;
    pidParams.Kd          = 0.0f;
    pidParams.outputMax   =  4095.0f;
    pidParams.outputMin   = -4095.0f;
    pidParams.integralMax = pidParams.outputMax / 5.0f;
    pidParams.sampleTime  = PID_SAMPLE_TIME / 1000.0f;

    /* ===== Arranque SEGURO: motores deshabilitados + freno activado ===== */
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_Init(&motor[i], i, MOTOR_STATUS_DISABLE);     /* EN=OFF */
        Motor_SetBrake(&motor[i], MOTOR_BRAKE_ENABLE);      /* BRAKE=ON */
        PID_Init(&motor[i].pid, pidParams, PID_STATUS_ENABLE);
        Drive_DacZeroForMotor(&driveDAC, i);                /* DAC canal = 0 */
        Drive_SetSafeDir(i);                                 /* DIR conocido */
        motor[i].refSpeed  = 0.0f;
        motor[i].measSpeed = 0.0f;
    }

    /* ===== Loop principal ===== */
    for (;;)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            const float ref_mps = speed[i];  /* speed[] viene en m/s, calculado en setSpeed() */

            if (fabsf(ref_mps) < WHEEL_ZERO_EPS_MPS) {
                /* ---- CERO SEGURO: NO habilitar ni liberar freno ---- */
                Motor_Enable(&motor[i], MOTOR_STATUS_DISABLE);   /* EN=OFF */
                Motor_SetBrake(&motor[i], MOTOR_BRAKE_ENABLE);   /* BRAKE=ON */
                Drive_SetSafeDir(i);                             /* DIR fijo */
                Drive_DacZeroForMotor(&driveDAC, i);             /* DAC=0 */
                Drive_ResetPidIntegrator(i);                     /* evita creep */
                motor[i].refSpeed = 0.0f;
            } else {
                /* ---- ACTIVACIÓN: hay demanda real de velocidad ---- */
                Motor_SetBrake(&motor[i], MOTOR_BRAKE_DISABLE);  /* libera freno */
                Motor_Enable(&motor[i], MOTOR_STATUS_ENABLE);    /* habilita driver */

                /* Lazo cerrado como en el original (usa driveDAC global) */
                motor[i].refSpeed = ref_mps;
                Motor_OLDrive(&motor[i], &driveDAC, ref_mps);
            }
        }

        /* Dribbler (igual que antes) */
        MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, Dribbler_SpeedSet[dribbler_sel]);

        /* Kicker (igual que antes) */
        if (kick_sel && kick_flag == KICKER_CHARGED) {
            osMutexWait(kickFlagHandle, osWaitForever);
            kick_flag = KICKER_START;
            osMutexRelease(kickFlagHandle);
            osMessagePut(kickQueueHandle, 0, 0);
        }

        osMessagePut(nrf24CheckHandle, 0, 0);
        osDelayUntil(&timeToWait, (uint32_t)PID_SAMPLE_TIME);
    }
}

/* setSpeed() queda igual */
void setSpeed(uint8_t *buffer, float *velocity, uint8_t *turn)
{
    float prv_Vx = v_vel[0], prv_Vy = v_vel[1];

    v_vel[0] = (buffer[1] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0xC0) << 1 | (uint16_t)(buffer[1] & 0x7F))/100.0f  : (float)((uint16_t)(buffer[4] & 0xC0) << 1 | (uint16_t)(buffer[1] & 0x7F))/100.0f ;
    v_vel[1] = (buffer[2] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0x30) << 3 | (uint16_t)(buffer[2] & 0x7F))/100.0f : (float)((uint16_t)(buffer[4] & 0x30) << 3 | (uint16_t)(buffer[2] & 0x7F))/100.0f ;
    v_vel[2] = (buffer[3] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0x0F) << 7 | (uint16_t)(buffer[3] & 0x7F))/100.0f : (float)((uint16_t)(buffer[4] & 0x0F) << 7 | (uint16_t)(buffer[3] & 0x7F))/100.0f ;

    float vel_mag = sqrtf(v_vel[0]*v_vel[0] + v_vel[1]*v_vel[1]);
    if (vel_mag > ROBOT_MAX_LINEAR_VEL)
    {
        float scale = ROBOT_MAX_LINEAR_VEL / vel_mag;
        v_vel[0] *= scale;
        v_vel[1] *= scale;
    }

    float Ax = v_vel[0] - prv_Vx, Ay = v_vel[1] - prv_Vy;
    float acc_sum = sqrtf(Ax*Ax + Ay*Ay);
    float norm_Ax = (acc_sum > 0.0f) ? Ax / acc_sum : 0.0f;
    float norm_Ay = (acc_sum > 0.0f) ? Ay / acc_sum : 0.0f;

    if(acc_sum > ROBOT_MAX_LINEAR_ACC)
    {
        acc_sum = ROBOT_MAX_LINEAR_ACC;
        Ax = norm_Ax * acc_sum;
        Ay = norm_Ay * acc_sum;

        v_vel[0] = prv_Vx + Ax;
        v_vel[1] = prv_Vy + Ay;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        float t_vel = 0;
        for (uint8_t j = 0; j < 3; j++)
            t_vel += kinematic[i][j] * v_vel[j];

        turn[i] = (t_vel > 0) ? WHEEL_P_ROTATION : WHEEL_N_ROTATION;
        velocity[i] = t_vel;  /* [m/s] */
    }
}
