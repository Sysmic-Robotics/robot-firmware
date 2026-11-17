#include "MPU6050.h"
#include "cmsis_os.h"

#define MPU6050_ADDR 0xD0 // 0x68

#define SMPLRT_DIV_REG   0x19
#define CONFIG_REG       0x1A
#define GYRO_CONFIG_REG  0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG   0x41
#define GYRO_XOUT_H_REG  0x43
#define PWR_MGMT_1_REG   0x6B
#define WHO_AM_I_REG     0x75

volatile HAL_StatusTypeDef i2c_debug_status = HAL_OK;
volatile uint8_t mpu_chip_id = 0; // Para ver qué ID está respondiendo

// Mantener estas variables como globales o externas si las usas fuera
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

extern I2C_HandleTypeDef hi2c3;

static uint8_t write_verify(uint8_t reg, uint8_t value)
{
	uint8_t readback;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, reg, 1, &value, 1, 1000) != HAL_OK)
		return 0;

	osDelay(5);
	if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, reg, 1, &readback, 1, 1000) != HAL_OK)
		return 0;
	return readback == value;
}

uint8_t MPU6050_init(void)
{
	uint8_t check;
	HAL_StatusTypeDef status;

	for (int retry = 0; retry < 3; retry++) {
		i2c_debug_status = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
		mpu_chip_id = check;

		if (i2c_debug_status != HAL_OK)
		{
			osDelay(10);
			continue;
		}

		// MODIFICACIÓN: Aceptar 104 (0x68 = MPU6050) o 112 (0x70 = MPU6500/Clon)
		if (check != 104 && check != 112)
		{
			osDelay(10);
			continue;
		}

		// Wake up (clear sleep bit)
		if (!write_verify(PWR_MGMT_1_REG, 0)) continue;
		osDelay(50);

		// DLPF 44 Hz — filtra vibración de los motores
		if (!write_verify(CONFIG_REG, 0x03)) continue;

		// Sample rate = 8 kHz / (1+7) = 1 kHz
		if (!write_verify(SMPLRT_DIV_REG, 0x07)) continue;

		// Accel ±2g
		if (!write_verify(ACCEL_CONFIG_REG, 0x00)) continue;

		// Gyro ±250°/s
		if (!write_verify(GYRO_CONFIG_REG, 0x00)) continue;

		return 1;  // ¡Inicialización exitosa!
	}
	return 0;  // Falló
}

// CAMBIO 1: Corrección de escala matemática (Se eliminó el *100 distorsionador)
uint8_t MPU6050_Read_Accel (float* Ax, float* Ay, float* Az)
{
	uint8_t Rec_Data[6];

	if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000) != HAL_OK)
		return 0;

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
	
	*Ax = Accel_X_RAW / 16384.0f;
	*Ay = Accel_Y_RAW / 16384.0f;
	*Az = Accel_Z_RAW / 16384.0f;
	return 1;
}

uint8_t MPU6050_Read_Gyro(float* Gx, float* Gy, float* Gz)
{
    uint8_t Rec_Data[6];

    if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000) != HAL_OK)
    	return 0;

    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    *Gx = Gyro_X_RAW / 131.0f;
    *Gy = Gyro_Y_RAW / 131.0f;
    *Gz = Gyro_Z_RAW / 131.0f;
    return 1;
}

// CAMBIO 2: NUEVA FUNCIÓN OPTIMIZADA PARA TU CONTROLADOR DE TIEMPOS
// Lee los 14 bytes consecutivos (Accel + Temp + Gyro) de un solo golpe.
// Reduce drásticamente el uso y bloqueo del bus I2C en FreeRTOS.
uint8_t MPU6050_Read_All(float* Ax, float* Ay, float* Az, float* Gx, float* Gy, float* Gz)
{
    uint8_t Rec_Data[14];

    // Leemos en ráfaga continua desde el registro ACCEL_XOUT_H_REG
    if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000) != HAL_OK)
        return 0;

    // Asignación de valores crudos del acelerómetro (0 al 5)
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Los bytes 6 y 7 corresponden a la temperatura (la ignoramos o puedes guardarla si deseas)

    // Asignación de valores crudos del giroscopio (8 al 13)
    Gyro_X_RAW  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // Conversión directa a floats escalados correctamente
    *Ax = Accel_X_RAW / 16384.0f;
    *Ay = Accel_Y_RAW / 16384.0f;
    *Az = Accel_Z_RAW / 16384.0f;

    *Gx = Gyro_X_RAW / 131.0f;
    *Gy = Gyro_Y_RAW / 131.0f;
    *Gz = Gyro_Z_RAW / 131.0f;

    return 1;
}
