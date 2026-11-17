#include <math.h>
#include "accel_task.h"
#include "system_globals.h"
#include "MPU6050.h"

#define DEG2RAD 0.01745329252f
#define G_MS2   9.80665f
#define CALIB_SAMPLES 100
void AccelFunction(void *argument)
{
    uint32_t tick = osKernelGetTickCount();
    float ax, ay, az;
    float gx, gy, gz; // Añadidos para lectura local

    float bias_x = 0.0f, bias_y = 0.0f;
    float bias_gyro_z = 0.0f; // Crítico para evitar el drift del giro en Z

    int calib_count = 0;

    // Asegurar que el sensor responda antes de continuar
    while (!MPU6050_init()) {
        osDelay(100);
    }

    // Rutina de calibración (El robot debe estar completamente quieto y plano)
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        if (MPU6050_Read_Accel(&ax, &ay, &az) && MPU6050_Read_Gyro(&gx, &gy, &gz)) {
            bias_x += ax;
            bias_y += ay;
            bias_gyro_z += gz; // Capturamos el error de cero del giroscopio
            calib_count++;
        }
        osDelay(2); // Un pequeño margen para que el sensor actualice su registro interno
    }

    if (calib_count > 0) {
        bias_x /= (float)calib_count;
        bias_y /= (float)calib_count;
        bias_gyro_z /= (float)calib_count;
    }

    for (;;)
    {
        // CONTROL DE TIEMPOS: 5ms da una tasa de 200Hz, ideal para el filtro DLPF de 44Hz
        tick += 5;
        osDelayUntil(tick);
        // Dentro del bucle infinito de tu AccelFunction:
        if (MPU6050_Read_All(&ax, &ay, &az, &gx, &gy, &gz)) {
            accel_x_raw = Accel_X_RAW;
            accel_y_raw = Accel_Y_RAW;
            accel_z_raw = Accel_Z_RAW;
            gyro_x_raw  = Gyro_X_RAW;
            gyro_y_raw  = Gyro_Y_RAW;
            gyro_z_raw  = Gyro_Z_RAW;

            accel_x_g = ax;
            accel_y_g = ay;
            accel_z_g = az;
            gyro_x_dps = gx;
            gyro_y_dps = gy;
            gyro_z_dps = gz;

            ax_ms2 = (ax - bias_x) * G_MS2;
            ay_ms2 = (ay - bias_y) * G_MS2;

            if (fabsf(ax_ms2) > 25.0f) ax_ms2 = 0.0f;
            if (fabsf(ay_ms2) > 25.0f) ay_ms2 = 0.0f;

            gyro_z_rads = (gz - bias_gyro_z) * DEG2RAD;
            if (fabsf(gyro_z_rads) > 10.0f) gyro_z_rads = 0.0f;
        }
    }
}
