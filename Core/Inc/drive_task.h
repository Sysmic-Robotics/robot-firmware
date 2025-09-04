#ifndef DRIVE_TASK_H
#define DRIVE_TASK_H

#include "cmsis_os.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= Configuración (overridable) =================
 * Puedes definir estos macros ANTES de incluir este header
 * para ajustar el comportamiento sin tocar el .c
 */

/* Período del PID [ms] (si no viene de otros headers) */
#ifndef PID_SAMPLE_TIME
#define PID_SAMPLE_TIME   1
#endif

/* Deadband para “cero seguro” por rueda (en m/s).
 * Mientras |speed[i]| < WHEEL_ZERO_EPS_MPS, NO se habilita el driver
 * ni se libera el freno y el DAC queda a 0.
 */
#ifndef WHEEL_ZERO_EPS_MPS
#define WHEEL_ZERO_EPS_MPS  (0.02f)
#endif

/* ============================================================= */

/* Tarea principal de manejo de tracción */
void DriveFunction(void const * argument);

/* Conversión de paquete a velocidades de ruedas (m/s) */
void setSpeed(uint8_t *buffer, float *velocity, uint8_t *turn);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_TASK_H */
