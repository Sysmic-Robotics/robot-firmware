#include "stm32f7xx_hal.h"
#include <stdint.h>

extern int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
extern int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

uint8_t MPU6050_init(void);                                   // returns 1 on success
uint8_t MPU6050_Read_Accel(float *Ax, float *Ay, float *Az); // returns 1 on success
uint8_t MPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz);  // returns 1 on success
uint8_t MPU6050_Read_All(float* Ax, float* Ay, float* Az, float* Gx, float* Gy, float* Gz);
