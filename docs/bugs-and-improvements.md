# Bugs Conocidos y Mejoras del Firmware

## Bugs Críticos (Corregidos)

### #1 — Unidades de encoder: rev/s vs rad/s

**Archivo**: `Tools/encoder.c:24`, `Tools/motor.c:46`
**Estado**: CORREGIDO (10-Jun-2026)

**Problema**: `Encoder_Update()` retorna velocidad en rev/s (revoluciones por segundo), pero `Motor_CLDrive()` en `motor.c` calcula `refSpeed = speed * SPEED_CNT_RATIO` donde `SPEED_CNT_RATIO = 1/WHEEL_RADIO`, que convierte velocidad lineal de rueda (m/s) a rad/s. El PID comparaba rad/s (referencia) vs rev/s (medición), con una diferencia de factor 2π (~6.28×). Esto causaba que el controlador PID comandara velocidades mucho mayores a las necesarias.

**Solución**: Se corrigió `Encoder_Update()` para que retorne rad/s en vez de rev/s, multiplicando por `2π`. De esta forma tanto `refSpeed` como `measSpeed` están en rad/s y el PID funciona correctamente.

```c
// Antes (rev/s):
float speed = (encPosRev - encoderDevice->oldPos) / sampleTime;

// Después (rad/s):
float speed = (encPosRev - encoderDevice->oldPos) / sampleTime;
speed *= 2.0f * (float)M_PI;
```

---

### #2 — Data race en variables compartidas (radio_task ↔ drive_task)

**Archivo**: `Core/Src/radio_task.c:49-51`, `Core/Src/drive_task.c:219-221`
**Estado**: CORREGIDO (10-Jun-2026)

**Problema**: Las variables `v_vel[3]`, `dribbler_sel`, `kick_sel` son escritas por `radio_task` (prioridad Normal) y leídas por `drive_task` (prioridad AboveNormal). Como `drive_task` tiene mayor prioridad, puede interrumpir a `radio_task` mientras escribe múltiples variables, resultando en un estado inconsistente (ej: vx del frame nuevo + vy del frame viejo).

**Solución**: Se protegió el bloque de escritura en `radio_task.c` con `taskENTER_CRITICAL()`/`taskEXIT_CRITICAL()` para asegurar atomicidad en la actualización de las variables compartidas.

```c
taskENTER_CRITICAL();
processRadioCommand(nrf_device.rx_data + 5 * robot_id);
dribbler_sel = getDribblerSpeedIndex(nrf_device.rx_data + 5 * robot_id);
kick_sel = getKickTriggerFlag(nrf_device.rx_data + 5 * robot_id);
taskEXIT_CRITICAL();
```

---

### #3 — Dribbler_SpeedSet out-of-bounds

**Archivo**: `Core/Src/kick_task.c:38-43`, `Core/Src/system_globals.c:113`
**Estado**: CORREGIDO (10-Jun-2026)

**Problema**: `Dribbler_SpeedSet[]` tiene 7 elementos (índices 0-6), pero el valor `dribbler_sel` se extrae de los bits [4:2] del paquete de radio, con rango 0-7. Si se enviaba el valor 7, se accedía al índice 7 del array (out-of-bounds), leyendo basura en memoria.

**Solución**: Se agregó clamping en `getDribblerSpeedIndex()` para limitar el valor máximo a 6.

```c
uint8_t getDribblerSpeedIndex(uint8_t *buffer)
{
    uint8_t dribbler_vel = (buffer[0] & 0x1C) >> 2;
    if (dribbler_vel > 6) dribbler_vel = 6;  // clamp to array max index
    return dribbler_vel;
}
```

---

### #4 — LowPassFilter se reiniciaba al converger a 0

**Archivo**: `Tools/lowpass_iir.c:17`, `Tools/lowpass_iir.h`
**Estado**: CORREGIDO (11-Jun-2026)

**Problema**: `LowPassFilter_Update()` usaba `filt->y == 0.0f` para detectar la primera muestra. Cuando el filtro convergía a 0 (rueda detenida), la condición se cumplía nuevamente y el filtro se reiniciaba, dejando pasar la siguiente muestra sin filtrar. Si esa muestra tenía ruido, `measSpeed` se disparaba abruptamente, el PID reaccionaba y el motor vibraba.

**Solución**: Se reemplazó la comparación `y == 0.0f` por un flag `initialized` explícito en el struct. El filtro se inicializa una sola vez en la primera llamada y nunca se reinicia.

```c
// Antes:
if (filt->y == 0.0f && filt->alpha > 0.0f) { ... }

// Después:
if (!filt->initialized) { ... }
```

---

### #5 — Encoder_Update: cálculo de velocidad simplificado y eliminado spike inicial

**Archivo**: `Tools/encoder.c`, `Tools/encoder.h`, `Core/Src/drive_task.c`
**Estado**: CORREGIDO (11-Jun-2026)

**Problema**: `Encoder_Update()` usaba un acumulador de 32-bit y un `oldPos` (posición en revoluciones) para calcular velocidad por diferenciación discreta. Esto introducía dos bugs:
1. **Spike inicial**: `oldPos` se inicializaba en 0, pero el acumulador ya tenía cuentas del timer en la primera lectura, generando una velocidad ficticia enorme (~77 rad/s con solo ruido de 100 cuentas).
2. **Struct hinchado**: El struct `Encoder_Handler_t` tenía campos innecesarios (`accumulator`, `oldPos`, `minSpeed`) que complicaban la inicialización y consumían stack.

Adicionalmente, el struct `Encoder_Handler_t` tenía campos (`minSpeed`) que se inicializaban con `WHEEL_MAX_SPEED_RAD` (definido en `drive_task.c`), creando una dependencia cruzada.

**Solución**: Se simplificó `Encoder_Update()` para calcular velocidad directamente del `diff` entre lecturas consecutivas del timer:

```c
float speed = (float)diff / (ENCODER_CPR * sampleTime);
speed *= 2.0f * (float)M_PI;
```

Se eliminaron los campos `accumulator`, `oldPos`, `minSpeed` del struct. Se agregó un clamp de seguridad (`> 200 rad/s` → 0) para rechazar lecturas físicamente imposibles.

---

## Bugs Conocidos (No corregidos aún)

### #6 — Kicker: carga continua sin verificar si hay kick pendiente

**Archivo**: `Core/Src/kick_task.c`
**Severidad**: Medio
**Estado**: RESUELTO (13-Jul-2026) — Ver Bug #31. El reestructuramiento de `KickFunction` con pre-carga al inicio y recarga solo después de disparar resolvió el problema de carga innecesaria. El capacitor ahora se mantiene cargado mientras espera `kick_sel`, sin ciclos de carga/descarga redundantes.

**Problema original**: La tarea kicker ciclaba continuamente: carga 4s → espera disparo → dispara → vuelve a cargar. Si no había pateo pendiente, el capacitor se cargaba y descargaba en cada ciclo.

---

### #7 — Radio watchdog solo verifica motor[0]

**Archivo**: `Core/Src/drive_task.c:166-183`
**Severidad**: Bajo

**Problema**: La lógica del watchdog de radio verifica únicamente `motor[0].enable` para decidir si todos los motores están deshabilitados/habilitados. Si los estados de los motores se desincronizan (situación posible si un motor falla individualmente), la lógica no detecta correctamente el estado.

**Mejora propuesta**: Verificar los 4 motores o usar una variable global `uint8_t motors_enabled` para el estado colectivo.

---

### #8 — PID_Init no normaliza Kd por sampleTime

**Archivo**: `Tools/PID.c:14-26`, `Tools/PID.c:69-76`
**Severidad**: Bajo
**Estado**: CORREGIDO (11-Jul-2026) — función `PID_UpdateController()` eliminada por completo (código muerto, nunca era llamada). Ver Bug #27.

**Problema**: `PID_Init()` usa Kd directamente sin dividir por sampleTime, mientras que `PID_UpdateController()` sí normaliza `Kd /= sampleTime`. Hay inconsistencia en la interfaz. Actualmente Kd=0 en todos los PIDs, por lo que no hay impacto funcional. Adicionalmente, `PID_UpdateController()` nunca era llamada desde ningún archivo — era código muerto.

**Solución**: Se eliminó `PID_UpdateController()` por completo (declaración en `PID.h` y definición en `PID.c`). La función nunca era invocada.

---

### #9 — nRF24_TxPacket polling con status potencialmente stale

**Archivo**: `Core/Src/radio_task.c:101-104`
**Severidad**: Bajo

**Problema**: El bucle de espera de TX lee `nrf_status` que se actualizó antes de cambiar a modo TX. La primera iteración del while tiene una condición basada en un valor previo.

**Solución**: La primera acción dentro del while es leer `nrf_status` de nuevo, por lo que funcionalmente es correcto, pero ineficiente.

---

### #10 — MPU6050_Read_Accel escala raw ×100/16384

**Archivo**: `Tools/MPU6050.c:51-53`
**Severidad**: Informativo

**Problema**: La función retorna aceleración en centi-g (100×g). En `accel_task.c` se convierte a m/s² dividiendo por 100 y multiplicando por G_MS2. El factor 100 existe solo por la escala de salida y se cancela inmediatamente, añadiendo complejidad innecesaria.

**Mejora propuesta**: Retornar directamente en g (raw/16384) o en m/s² (raw/16384 * 9.80665).

---

### #11 — FreeRTOS heap de 15KB puede ser ajustado

**Archivo**: `Core/Inc/FreeRTOSConfig.h:67`
**Severidad**: Informativo

**Problema**: `configTOTAL_HEAP_SIZE` es 15360 bytes. Con 4 tareas (1KB stacks total), colas, mutex y timers, puede ser suficiente pero ajustado.

---

### #12 — Radio nRF24 no recibe: address width incompatible + TX comentada

**Archivo**: `Tools/nrf24.c:182`, `Tools/nrf24.c:185-271`, `Core/Src/radio_task.c:59-62`
**Severidad**: Crítico
**Estado**: CORREGIDO (13-Jun-2026)

**Problema**: Dos causas independientes rompieron la comunicación de radio:

1. **Address width incorrecto**: `nRF24_SetAddrWidth()` usaba `addr_width - 3` para calcular SETUP_AW, resultando en SETUP_AW=2 (direcciones de 4 bytes). La base station usa SETUP_AW=3 (5 bytes). El hardware del nRF24 filtraba los paquetes entrantes por dirección y nunca coincidían, por lo que el robot no recibía nada.

2. **TX de telemetría comentada**: Las llamadas a `packTelemetryBuffer()` y `Radio_SendPacket()` estaban comentadas. Si la base station espera telemetría del robot para considerar el enlace activo, nunca recibía respuesta.

**Solución**:

```c
// Tools/nrf24.c — SetAddrWidth corregido:
// Antes: nRF24_WriteReg(device, nRF24_REG_SETUP_AW, addr_width - 3);  // SETUP_AW=2
// Después: nRF24_WriteReg(device, nRF24_REG_SETUP_AW, addr_width - 2); // SETUP_AW=3

// Tools/nrf24.c — SetAddr corregido (todos los pipes):
// Antes: addr_width = aw_reg + 3; addr += addr_width - 1; ... } while (--addr_width);
// Después: addr_width = SETUP_AW + 1; addr += addr_width; ... } while (addr_width--);

// Src/radio_task.c — TX reactivada:
// Antes: // packTelemetryBuffer(txBuffer); // Radio_SendPacket(...);
// Después: packTelemetryBuffer(txBuffer); Radio_SendPacket(...);
```

**Diagnóstico**: La variable `aw_reg` se eliminó del stack. El cálculo del address width en `SetAddr` se alineó con el viejo firmware que funcionaba: `SETUP_AW + 1` con post-decrement en el do-while, resultando en 5 bytes de dirección enviados correctamente.

### #13 — Test mode mapping faltante en processRadioCommand

**Archivo**: `Core/Src/drive_task.c:274-306`
**Severidad**: 🔴 Crítico

**Problema**: La documentación (`docs/characterization-guide.md` línea 65 y `docs/motor-characterization.md`) indica que `processRadioCommand` mapea vx→test_motor y vy→test_voltage cuando test_enable==1. Sin embargo, el código de `processRadioCommand()` solo decodifica v_vel[0..2] sin verificar test_enable ni actualizar test_motor/test_voltage. Esto hace que la caracterización de motores (characterize.lua + test_motor_step) no funcione.

**Fix**: Ver `.omo/plans/implementation-plan.md` — Fase 1.1

---

### #14 — kick_sel nunca se resetea (auto-rekick infinito)

**Archivo**: `Core/Src/radio_task.c:56`, `Core/Src/kick_task.c:11`
**Severidad**: 🔴 Crítico

**Problema**: `kick_sel` se escribe desde radio_task pero nunca se resetea a 0. Una vez activado, kick_task queda en ciclo perpetuo: carga 4s → espera → dispara → vuelve a cargar inmediatamente.

**Fix**: Ver `.omo/plans/implementation-plan.md` — Fase 1.2

---

### #15 — Decodificación vth con máscara incorrecta (11-bit vs 10-bit)

**Archivo**: `Core/Src/drive_task.c:284`
**Severidad**: 🟡 Alto

**Problema**: `v_vel[2]` usa `(buffer[4] & 0x0F) << 7` (4 bits = 11-bit total) pero el protocolo especifica bits [3:2] de buffer[4] = 2 bits MSB para 10-bit total. Máscara correcta: `0x0C`. Rango incorrecto: ±10.23 rad/s en vez de ±5.11 rad/s.

**Fix**: Ver `.omo/plans/implementation-plan.md` — Fase 1.3

---

### #16 — vz (dribbler vertical velocity) ignorado en control loop

**Archivo**: `Core/Src/drive_task.c`
**Severidad**: 🟡 Medio

**Problema**: El protocolo de radio envía velocidad vertical (vz) para el dribbler en algunos paquetes, pero el firmware no lo procesa. Dribbler se controla solo por `dribbler_sel` (bits [4:2] del byte 0).

**Estado**: Aceptado (diseño actual usa dribbler_sel como control discreto 0-7).

---

## Auditoría 11-Jul-2026 — Bugs Corregidos

Se realizó una auditoría completa del código fuente (firmware C + engine Rust). Se encontraron 14 problemas adicionales, todos corregidos en esta fecha.

### #17 — Motor DAC overflow: voltaje puede exceder 4095 (CRITICAL)

**Archivo**: `Tools/motor.c:54-55`, `Tools/motor.c:91`
**Severidad**: 🔴 Crítico
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: En `Motor_CLDrive()` (línea 54-55) y `Motor_SetVoltage()` (línea 91), la suma `pid.output + ffVoltage` puede exceder 4095 (rango del DAC 12-bit). Al hacer cast a `uint16_t`, el valor se desborda (wrapping), resultando en un voltaje de DAC muy bajo en vez del máximo esperado. Esto podía causar que un motor瞬间mente recibiera voltaje mínimo durante una maniobra agresiva.

**Solución**: Se agregó `fabsf()` + clamp a `[0.0f, 4095.0f]` antes del cast a `uint16_t` en ambas funciones. También se limpió `Motor_SetVoltage()` para extraer `absSpeed` una sola vez en vez de duplicar la lógica de clamping en ambas ramas.

```c
// Antes (Motor_CLDrive):
*refDuty = (uint16_t)(pid.output + ffVoltage);

// Después:
float voltage = pid.output + ffVoltage;
if (voltage < 0.0f) voltage = 0.0f;
if (voltage > 4095.0f) voltage = 4095.0f;
*refDuty = (uint16_t)voltage;
```

---

### #18 — Rate limiter no se resetea al perder radio → drift al reconectar (CRITICAL)

**Archivo**: `Core/Src/drive_task.c:250-273`
**Severidad**: 🔴 Crítico
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: Las variables estáticas `vx_cmd_limited`, `vy_cmd_limited`, `vth_cmd_limited` (rate limiters) se declaraban como `static` dentro del bloque de control (scope de bloque). Al perder conexión de radio, se usaba el último valor limitado, pero al reconectar, el rate limiter arrancaba desde ese valor residual en vez de 0. Esto causaba que el robot driftara con una velocidad fantasma al reconectar.

**Solución**: Se movieron las variables del scope de bloque al scope de la función `DriveFunction`, y se agregó reset a `0.0f` en la rama de timeout de radio (junto a la limpieza de comandos).

```c
// Antes: static dentro del bloque de control
{
    static float vx_cmd_limited = 0.0f;
    // ...
}

// Después: scope de función, reset en timeout
float vx_cmd_limited = 0.0f;
float vy_cmd_limited = 0.0f;
float vth_cmd_limited = 0.0f;
// En timeout de radio:
vx_cmd_limited = 0.0f;
vy_cmd_limited = 0.0f;
vth_cmd_limited = 0.0f;
```

---

### #19 — EKF: actualización de covarianza P corrompe filas 5,6 (CRITICAL)

**Archivo**: `Tools/ekf.c:315-321` (`EKF_UpdateEncoders`)
**Severidad**: 🔴 Crítico
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: En `EKF_UpdateEncoders`, la actualización de la matriz de covarianza P se hacía in-place: las filas 4, 5 y 6 se leían y escribían secuencialmente en el mismo buffer `Pfull`. Cuando se procesaba la fila 4, las filas 5 y 6 aún contenían valores correctos. Pero al escribir la fila 4 y luego leer la fila 5, esta última ya podía haber sido afectada por la propagación de la fila 4 (dependiendo de la implementación exacta del bucle). Esto causaba correlaciones espurias entre las dimensiones de estado.

**Solución**: Se agregaron arreglos temporales `P4[7]`, `P5[7]`, `P6[7]` para guardar las filas 4, 5 y 6 de `Pfull` antes del bucle de actualización. El bucle ahora lee de los temporales en vez de leer de `Pfull` directamente.

```c
// Antes: lectura directa de Pfull (corrupto después de escritura previa)
for (j = 0; j < 7; j++) { Pfull[4][j] -= K[4] * H_P[j]; }

// Después: lectura de temp guardado antes del bucle
float P4[7], P5[7], P6[7];
for (j = 0; j < 7; j++) { P4[j] = Pfull[4][j]; P5[j] = Pfull[5][j]; P6[j] = Pfull[6][j]; }
for (j = 0; j < 7; j++) { Pfull[4][j] -= K[4] * H_P[j]; }
// ... usa P5[j] y P6[j] en vez de Pfull[5][j], Pfull[6][j]
```

---

### #20 — EKF: sinθ/cosθ se desvían del círculo unitario (CRITICAL)

**Archivo**: `Tools/ekf.c:307-310`
**Severidad**: 🔴 Crítico
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: Después de la actualización del estado `x += K·y`, los valores `x[2]` (sinθ) y `x[3]` (cosθ) se desviaban del círculo unitario debido a errores de precisión de punto flotante y a la actualización asimétrica del filtro. Con el tiempo, `sin²θ + cos²θ` se alejaba de 1.0, causando que la dirección reportada por el EKF se distorsionara progresivamente.

**Solución**: Se agregó renormalización después de la actualización del estado:

```c
// Después de x += K * y:
float s = ekf->x[2];
float c = ekf->x[3];
float mag = sqrtf(s * s + c * c);
if (mag > 1e-6f) {
    ekf->x[2] = s / mag;
    ekf->x[3] = c / mag;
}
```

---

### #21 — PID integral se resetea con comando≈0 en vez de error≈0 (MODERATE)

**Archivo**: `Core/Src/drive_task.c:281-282`
**Severidad**: 🟡 Moderado
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: El congelamiento del integral PID se activaba cuando `fabs(vx_cmd) < 0.01` (comando de velocidad ≈ 0), no cuando el error de seguimiento era pequeño. Esto significaba que si el robot estaba siguiendo un comando constante de 0.5 m/s con error pequeño, el integral NO se congelaba. Pero si el comando era 0 (detenido) y el robot estaba casi detenido, el integral sí se congelaba. La condición correcta es congelar cuando el ERROR es pequeño, no cuando el comando es pequeño.

**Solución**: Se cambió la condición de `fabs(vx_cmd)` → `fabsf(vx_cmd - vx_meas)` (error de seguimiento):

```c
// Antes:
if (fabs(vx_cmd) < 0.01f) { pid_vx.integral = 0.0f; }

// Después:
if (fabsf(vx_cmd - vx_meas) < 0.01f) { pid_vx.integral = 0.0f; }
```

---

### #22 — Rate limiter de theta aplica a ángulo absoluto (MODERATE)

**Archivo**: `Core/Src/drive_task.c:269-273`
**Severidad**: 🟡 Moderado
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: El rate limiter de theta se aplicaba ANTES del PID, limitando la tasa de cambio del ángulo objetivo absoluto (`vth_cmd`). Esto es incorrecto semánticamente — el rate limiter debe limitar la velocidad angular de salida, no la demanda de ángulo. Un robot que necesita girar 180° rápidamente veía su ángulo objetivo limitado gradualmente, resultando en una rotación lenta y temblorosa.

**Solución**: Se removió el rate limiter de theta del bloque pre-PID y se agregó un limitador de velocidad angular DESPUÉS de `PID_CloseLoop(&pid_theta, ...)`, consistente con cómo funcionan los limitadores de vx/vy.

---

### #23 — kick_task: osMessageQueueGet con osWaitForever puede colgar (MODERATE)

**Archivo**: `Core/Src/kick_task.c:27`
**Severidad**: 🟡 Moderado
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: `osMessageQueueGet` en `kick_task.c` usaba `osWaitForever` como timeout. Si el capacitor estaba cargado y no llegaba ningún comando de kick (radio perdida, drive_task bloqueado), la tarea quedaba colgada indefinidamente en esta llamada, sin poder responder a ningún evento nuevo.

**Solución**: Se cambió el timeout a 5000ms (5 segundos). Al expirar, se verifica el estado y se reinicia la secuencia de carga si es necesario:

```c
// Antes:
osMessageQueueGet(kickQueueHandle, &kick_cmd, NULL, osWaitForever);

// Después:
osStatus_t status = osMessageQueueGet(kickQueueHandle, &kick_cmd, NULL, 5000);
if (status == osOK) { /* procesar kick */ }
else { continue; /* timeout → reiniciar ciclo de carga */ }
```

---

### #24 — Logger flush en cada frame desperdicia BufWriter (MODERATE)

**Archivo**: `engine/engine/src/logger/logger.rs:122`
**Severidad**: 🟡 Moderado
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: `log_frame()` llamaba `writer.flush()` en cada frame (60/s), anulando completamente el beneficio de `BufWriter`. Cada flush fuerza una syscall de escritura al disco, incrementando I/O innecesariamente.

**Solución**: Se agregó un contador `frame_count` al logger. Ahora solo se hace flush cada 60 frames (~1 segundo), restaurando el batching del `BufWriter`:

```c
// Antes:
self.writer.flush()?;

// Después:
self.frame_count += 1;
if self.frame_count >= 60 {
    self.writer.flush()?;
    self.frame_count = 0;
}
```

---

### #25 — processRadioCommand: parámetros unused `velocity`/`turn` (LOW)

**Archivo**: `Core/Src/drive_task.c`, `Core/Src/radio_task.c`, `Core/Inc/drive_task.h`
**Severidad**: 🔵 Bajo
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: `processRadioCommand()` recibía parámetros `velocity` y `turn` que nunca eran usados dentro de la función. Esto creaba una API confusa y violaba el principio de que los parámetros deben tener un propósito claro.

**Solución**: Se eliminaron los parámetros `velocity` y `turn` de la firma de `processRadioCommand()`. Ahora solo recibe `buffer`. Se actualizaron todas las llamadas en `radio_task.c` y la declaración en `drive_task.h`.

---

### #26 — PID_UpdateController: código muerto eliminado (LOW)

**Archivo**: `Tools/PID.c:69-76`, `Tools/PID.h`
**Severidad**: 🔵 Bajo
**Estado**: CORREGIDO (11-Jul-2026) — Ver Bug #8 para contexto original.

**Problema**: `PID_UpdateController()` era una función que normalizaba Kd por sampleTime, pero nunca era llamada desde ningún archivo del proyecto. Era código muerto que generaba confusión con `PID_CloseLoop()`.

**Solución**: Se eliminó la definición de `PID.c` y la declaración de `PID.h`.

---

### #27 — system_globals.h: `#include` antes del header guard (LOW)

**Archivo**: `Core/Inc/system_globals.h:1-2`
**Severidad**: 🔵 Bajo
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: `#include <stdint.h>` aparecía ANTES de `#ifndef SYSTEM_GLOBALS_H` (el header guard). Esto causaba que `stdint.h` se procesara en cada inclusión, incrementando tiempo de compilación innecesariamente. Aunque funcionalmente no causaba bugs (stdint.h tiene su propio guard), viola la convención estándar de C.

**Solución**: Se movió `#include <stdint.h>` debajo del header guard:

```c
// Antes:
#include <stdint.h>
#ifndef SYSTEM_GLOBALS_H
#define SYSTEM_GLOBALS_H

// Después:
#ifndef SYSTEM_GLOBALS_H
#define SYSTEM_GLOBALS_H
#include <stdint.h>
```

---

### #28 — Tracker: `set_noise_parameters()` llamado cada frame innecesariamente (LOW)

**Archivo**: `engine/engine/src/receiver/tracker.rs:216`
**Severidad**: 🔵 Bajo
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: `Tracker::track()` llamaba `filter.set_noise_parameters(...)` en cada invocación (~60/s). Los valores de ruido (`process_noise_p/v`, `measurement_noise`) solo cambian cuando se llama `update_config()`, que es raro. La llamada repetida ejecutaba 3 asignaciones y potencialmente recálculos internos del filtro por nada.

**Solución**: Se eliminó la llamada `set_noise_parameters()` del bucle `track()`. Los parámetros se establecen una sola vez al crear el filtro y se actualizan solo cuando `update_config()` los cambia.

---

### #29 — Logger: timestamp sin precisión sub-segundo (LOW)

**Archivo**: `engine/engine/src/logger/logger.rs` (`chrono_like_timestamp()`)
**Severidad**: 🔵 Bajo
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: `chrono_like_timestamp()` usaba `SystemTime::now().as_secs()` que solo tiene precisión de 1 segundo. Si se creaban múltiples sesiones de logging en menos de 1 segundo, los nombres de archivo colisionaban.

**Solución**: Se agregó precisión de milisegundos al timestamp:

```c
// Antes: "20260711_143025"
// Después: "20260711_143025.123"
```

---

### #30 — nRF24_Init: Auto-Acknowledge habilitado en pipes 1-5 innecesarios (LOW)

**Archivo**: `Tools/nrf24.c:90`
**Severidad**: 🔵 Bajo
**Estado**: CORREGIDO (11-Jul-2026)

**Problema**: `nRF24_Init()` configuraba `EN_AA = 0x3F` (Auto-Acknowledge habilitado en los 6 pipes), pero solo se usa pipe 0 para RX/TX. Los pipes 1-5 no se usaban en ese momento, por lo que tener AA habilitado en ellos generaba configuración innecesaria y potencialmente interferencia si algún pipe coincidiera con una dirección residual.

**Solución**: Se cambió `EN_AA` de `0x3F` (todos los pipes) a `0x01` (solo pipe 0).

> **Nota (27-Ago-2026, B2)**: Con la introducción del canal de corrección de cámara, la **Pipe 1 ahora sí se usa** (dirección `"sysrc"`, payload 18 bytes). Sin embargo, la Pipe 1 se configura explícitamente con `nRF24_AA_OFF` (ver `radio_task.c`), por lo que `EN_AA = 0x01` sigue siendo correcto — el AA solo está habilitado en pipe 0. La justificación original ("pipes 1-5 nunca se usan") quedó desactualizada, pero la configuración de AA no necesita cambio.

```c
// Antes:
nRF24_WriteReg(device, nRF24_REG_EN_AA, 0x3F);

// Después:
nRF24_WriteReg(device, nRF24_REG_EN_AA, 0x01);
```

---

### #31 — kick_task: retardo de 5s al primer kick por carga post-等待 (CRITICAL)

**Archivo**: `Core/Src/kick_task.c`
**Severidad**: 🔴 Crítico
**Estado**: CORREGIDO (13-Jul-2026)

**Problema**: El flujo original de `KickFunction` era: esperar `kick_sel` → cargar capacitor (4s) → esperar comando de kick → disparar. Cuando el driver enviaba `kick_sel=1` por primera vez, el robot tardaba ~5 segundos en patear (4s de carga + ~1s de latencia). En un partido, esto hacía que el pase fuera inútil — el oponente interceptaba antes de que el robot pudiera patear.

**Solución**: Se reestructuró `KickFunction` para pre-cargar el capacitor al inicio (antes del loop principal). Se extrajeron helpers estáticos `charge_capacitor()` y `fire_kicker()`. El flujo nuevo es: startup → charge → wait kick_sel → wait kick_cmd → fire → re-charge → wait kick_sel → ...

```c
// Flujo nuevo:
static void charge_capacitor(void) {
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_4, GPIO_PIN_SET);
    osDelay(4000);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_4, GPIO_PIN_RESET);
    osMutexAcquire(kickFlagHandle, osWaitForever);
    kick_flag = KICKER_CHARGED;
    osMutexRelease(kickFlagHandle);
}

void KickFunction(void *argument) {
    charge_capacitor();  // Pre-carga al startup
    for (;;) {
        while (!kick_sel) { osDelay(10); }  // Espera orden del driver
        uint32_t _mq_msg;
        osStatus_t status = osMessageQueueGet(kickQueueHandle, &_mq_msg, NULL, 5000);
        if (status != osOK) continue;
        fire_kicker();
        charge_capacitor();  // Recarga para el siguiente
    }
}
```

Adicionalmente, se agregó `kick_sel = 0` en `drive_task.c` después de enviar el comando de kick para evitar re-kick automático.

---

### #32 — WHEEL_GEAR_RATIO: reducción de engranaje configurable (MODERATE)

**Archivo**: `Tools/motor.h`, `Core/Src/drive_task.c`, `Core/Src/radio_task.c`
**Severidad**: 🟡 Moderado
**Estado**: CORREGIDO (13-Jul-2026)

**Problema**: El firmware asumía ruedas de tracción directa (motor = rueda). Con la introducción de un mecanismo de reducción por engranajes (ratio 3:1), el encoder mide velocidad del MOTOR, no de la rueda. Sin ajustar, la odometría reportaría 3× la velocidad real, el PID de wheel compararía velocidades de motor vs referencia de rueda, y la telemetría enviaría datos incorrectos al engine.

**Solución**: Se agregó `WHEEL_GEAR_RATIO` (= 3.0f) en `motor.h` como el único parámetro a cambiar entre versiones de robot (direct drive = 1.0f, geared = 3.0f). Se ajustaron todos los puntos que mezclan espacios de velocidad:

```c
// motor.h:
#define WHEEL_GEAR_RATIO  3.0f
#define SPEED_CNT_RATIO   (WHEEL_GEAR_RATIO / WHEEL_RADIO)  // m/s → motor rad/s

// drive_task.c — odometría (ambos caminos: test y closed-loop):
wheel_speeds_rads[i] = motor[i].measSpeed / WHEEL_GEAR_RATIO;

// drive_task.c — deadband escalado:
motor[i].speedDeadband = WHEEL_MAX_SPEED_RAD * WHEEL_GEAR_RATIO * 0.005f;

// radio_task.c — telemetría:
float m0 = motor[0].measSpeed / WHEEL_GEAR_RATIO;  // wheel rad/s
```

**Flujo de señales con gear ratio:**
```
speed[i] (m/s) × SPEED_CNT_RATIO = refSpeed (motor rad/s)
encoder → measSpeed (motor rad/s)
PID: refSpeed vs measSpeed ✓
odometry: measSpeed / WHEEL_GEAR_RATIO = wheel rad/s ✓
telemetry: EKF state (x/y/vx/vy/θ/ω) → engine ✓
```

Para cambiar de robot: solo tocar `WHEEL_GEAR_RATIO` en `motor.h` (1.0f o 3.0f).

---

### #33 — Address Mismatch: BS RX pipe1 escucha dirección incorrecta (CRITICAL)

**Archivo**: `basestation/Src/main.c`
**Estado**: CORREGIDO (23-Jul-2026)

**Problema**: El módulo `rx_device` de la basestation escuchaba en `rx_node_addr` ("sysrx") en pipe1, pero el robot transmite su telemetría a la dirección `tx_node_addr` ("systx"). El hardware NRF24L01+ filtra paquetes por dirección de pipe, por lo que la basestation nunca recibía telemetría del robot. `rx_status` quedaba en 14 (RX_DR=0).

**Solución**: Cambiado `nRF24_SetAddr(&rx_device, nRF24_PIPE1, rx_node_addr)` → `nRF24_SetAddr(&rx_device, nRF24_PIPE1, tx_node_addr)`. El rx_device ahora escucha en "systx" (la dirección TX del robot), permitiendo recibir telemetría correctamente.

---

### #34 — UART Blocking TX causa pérdida de paquetes (CRITICAL)

**Archivo**: `basestation/Src/main.c`
**Estado**: CORREGIDO (23-Jul-2026)

**Problema**: `HAL_UART_Transmit()` bloquea ~2.8ms por paquete de 32 bytes a 115200 baud. Con 6 robots transmitiendo telemetría a 100Hz con stagger de ~1.67ms, la ventana de transmisión es insuficiente y se pierden paquetes. El engine no recibe datos correctos.

**Solución**: Cambiado a `HAL_UART_Transmit_IT()` (interrupt-driven, no bloqueante). Se agregó variable `volatile uint8_t uart_tx_busy` como guard. El main loop verifica `!uart_tx_busy` antes de transmitir. `HAL_UART_TxCpltCallback()` limpia el flag al completar.

---

### #35 — USART1_IRQHandler procesa interrupciones TX (CRITICAL)

**Archivo**: `basestation/Src/stm32f1xx_it.c`
**Estado**: CORREGIDO (23-Jul-2026)

**Problema**: El handler original llamaba `HAL_UART_RxCpltCallback()` en TODA interrupción de USART1, incluyendo TX complete. Con `HAL_UART_Transmit_IT`, esto generaba 33 llamadas por paquete (32 TXE + 1 TC), cada una deshabilitando/rehabilitando DMA RX y reseteando CNDTR, corrompiendo los datos entrantes.

**Solución**: El handler ahora solo llama `HAL_UART_RxCpltCallback()` cuando el flag IDLE está activo:

```c
void USART1_IRQHandler(void) {
  uint32_t sr = huart1.Instance->SR;
  if (sr & USART_SR_IDLE) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    HAL_UART_RxCpltCallback(&huart1);
  }
  HAL_UART_IRQHandler(&huart1);
}
```

---

### #36 — UART TX FIFO drain por while-loop (MODERATE)

**Archivo**: `basestation/Src/main.c`
**Estado**: CORREGIDO (23-Jul-2026)

**Problema**: El main loop original usaba un `while` loop para drenar el FIFO del NRF24 y forward cada paquete a UART. Si `uart_tx_busy` era true (TX anterior aún en curso), el while extraía el paquete del NRF24 FIFO y lo descartaba sin enviarlo, causando pérdida de telemetría.

**Solución**: Cambiado `while` a `if` — solo extrae un paquete del FIFO si hay espacio para enviarlo. Los paquetes restantes se procesan en la siguiente iteración del loop.

---

## Mejoras Generales Pendientes

| # | Mejora | Impacto |
|---|--------|---------|
| 1 | Mover handles de DAC (driveDAC, dribblerDAC) a variables static en vez de stack | Bajo |
| 2 | Agregar `.clang-format` y unificar estilo (mezcla tabs/spaces) | Bajo |
| 3 | Agregar timeouts configurables en todas las operaciones I2C/SPI | Medio |
| 4 | Unificar include guards (mezcla `_PID_H_`, `_MOTOR_H_`, `__NRF24_H__`) | Bajo |
| 5 | Agregar `static` a funciones y variables que no necesitan ser globales | Bajo |
| 6 | Usar `HAL_I2C_Mem_Read/Write` en VL6180X en vez de `Master_Transmit+Receive` manual | Medio |
| 7 | ~~Cachear dribbler_sel para evitar 1000 I2C writes/s (solo escribir si cambia)~~ | Bajo |
| 8 | Separado en pid_vx_enable, pid_vy_enable, pid_vth_enable | Medio |

> **Nota (27-Ago-2026)**: La mejora #7 ya está implementada — el cache de `dribbler_sel` vive en `drive_task.c:272-276` (`last_dribbler_sel`, solo escribe al DAC cuando el valor cambia). La mejora #8 también está implementada (`pid_vx_enable`, `pid_vy_enable`, `pid_vth_enable` en `system_globals`).

### Plan de Implementación

Ver `.omo/plans/implementation-plan.md` para el plan completo.

**Estado de implementación (30-Jun-2026):**

| Bug | Archivo | Severidad | Estado |
|-----|---------|-----------|--------|
| #13 — Test mode mapping | `drive_task.c` | 🔴 Crítico | ✅ Corregido |
| #14 — kick_sel auto-reset | `radio_task.c`, `kick_task.c` | 🔴 Crítico | ✅ Corregido |
| #15 — vth mask incorrecta | `drive_task.c:284` | 🟡 Alto | ✅ Corregido |
| #16 — vz ignorado | `drive_task.c` | 🟡 Medio | ✅ Aceptado (diseño actual) |

**Code smells resueltos:**
- M5 — DAC handles movidos a `static` (~200 bytes stack liberados)
- M9 — Dribbler I2C cacheado (1000/s → solo en cambio)
- M4 — Includes legacy F0/F4/F7 eliminados (~175 líneas)
- M8 — ball_detector_task comentada con nota

**Mejoras implementadas:**
- `pid_vx_enable`, `pid_vy_enable`, `pid_vth_enable` (control por eje desde stm32monitor)
- Parámetros `velocity`/`turn` eliminados de `processRadioCommand` (Bug #25)

**Auditoría 11-Jul-2026 — Correcciones:**

| Bug | Archivo | Severidad | Estado |
|-----|---------|-----------|--------|
| #17 — Motor DAC overflow | `Tools/motor.c` | 🔴 Crítico | ✅ Corregido |
| #18 — Rate limiter drift en radio timeout | `Core/Src/drive_task.c` | 🔴 Crítico | ✅ Corregido |
| #19 — EKF P matrix in-place corruption | `Tools/ekf.c` | 🔴 Crítico | ✅ Corregido |
| #20 — EKF sin/cos unit circle drift | `Tools/ekf.c` | 🔴 Crítico | ✅ Corregido |
| #21 — PID integral zeroing por comando≠error | `Core/Src/drive_task.c` | 🟡 Moderado | ✅ Corregido |
| #22 — Rate limiter theta en ángulo absoluto | `Core/Src/drive_task.c` | 🟡 Moderado | ✅ Corregido |
| #23 — kick_task osWaitForever hang | `Core/Src/kick_task.c` | 🟡 Moderado | ✅ Corregido |
| #24 — Logger flush cada frame | `engine/.../logger.rs` | 🟡 Moderado | ✅ Corregido |
| #25 — processRadioCommand params unused | `drive_task.c`, `radio_task.c` | 🔵 Bajo | ✅ Corregido |
| #26 — PID_UpdateController dead code | `Tools/PID.c`, `PID.h` | 🔵 Bajo | ✅ Corregido |
| #27 — system_globals.h include antes de guard | `Core/Inc/system_globals.h` | 🔵 Bajo | ✅ Corregido |
| #28 — Tracker noise params cada frame | `engine/.../tracker.rs` | 🔵 Bajo | ✅ Corregido |
| #29 — Logger timestamp sin milisegundos | `engine/.../logger.rs` | 🔵 Bajo | ✅ Corregido |
| #30 — nRF24 AA en pipes innecesarios | `Tools/nrf24.c` | 🔵 Bajo | ✅ Corregido |

**Correcciones 13-Jul-2026 — Kick + Gear Ratio:**

| Bug | Archivo | Severidad | Estado |
|-----|---------|-----------|--------|
| #31 — kick_task retardo 5s primer kick | `Core/Src/kick_task.c`, `Core/Src/drive_task.c` | 🔴 Crítico | ✅ Corregido |
| #32 — WHEEL_GEAR_RATIO para reducción configurable | `Tools/motor.h`, `Core/Src/drive_task.c`, `Core/Src/radio_task.c` | 🟡 Moderado | ✅ Corregido |

**Correcciones 23-Jul-2026 — Dual Channel + UART Fixes:**

| Bug | Archivo | Severidad | Estado |
|-----|---------|-----------|--------|
| #33 — Address Mismatch BS RX pipe1 | `basestation/Src/main.c` | 🔴 Crítico | ✅ Corregido |
| #34 — UART Blocking TX | `basestation/Src/main.c` | 🔴 Crítico | ✅ Corregido |
| #35 — USART1 IRQ handler | `basestation/Src/stm32f1xx_it.c` | 🔴 Crítico | ✅ Corregido |
| #36 — UART TX FIFO drain | `basestation/Src/main.c` | 🟡 Moderado | ✅ Corregido |

## Historial de Cambios

| Fecha | Archivo | Cambio |
|-------|---------|--------|
| 10-Jun-2026 | `Tools/encoder.c` | Corregido Encoder_Update para retornar rad/s (× 2π) — Bug #1 |
| 10-Jun-2026 | `Core/Src/radio_task.c` | Agregada sección crítica para escritura de vars compartidas — Bug #2 |
| 10-Jun-2026 | `Tools/kick_task.c` | Agregado clamping a getDribblerSpeedIndex (max 6) — Bug #3 |
| 10-Jun-2026 | `Tools/lowpass_iir.c` | Agregada función LowPassFilter_InitValue; Update detecta primer sample — Mejora #5 |
| 10-Jun-2026 | `Tools/lowpass_iir.h` | Declarada LowPassFilter_InitValue |
| 10-Jun-2026 | `Core/Src/drive_task.c` | Mejorado watchdog para verificar los 4 motores en vez de solo motor[0] — Mejora #6 |
| 10-Jun-2026 | `Core/Src/kick_task.c` | Kicker espera kick_sel activo antes de cargar — Mejora #4 |
| 10-Jun-2026 | `docs/bugs-and-improvements.md` | Documento creado con bugs, mejoras e historial de cambios |
| 11-Jun-2026 | `Tools/lowpass_iir.c` | Reemplazada comparación `y == 0.0f` por flag `initialized` — Bug #4 |
| 11-Jun-2026 | `Tools/lowpass_iir.h` | Agregado campo `initialized` a LowPassFilter_t — Bug #4 |
| 11-Jun-2026 | `Tools/encoder.c` | Eliminados accumulator/oldPos; speed calculado directo del diff — Bug #5 |
| 11-Jun-2026 | `Tools/encoder.h` | Struct reducido a count/lastRaw/enable — Bug #5 |
| 11-Jun-2026 | `Core/Src/drive_task.c` | Removidas inicializaciones de accumulator/oldPos/minSpeed para 4 motores — Bug #5 |
| 13-Jun-2026 | `Tools/nrf24.c` | Corregido address width del nRF24 (SETUP_AW, SetAddr) — Bug #12 |
| 13-Jun-2026 | `Core/Src/radio_task.c` | Reactivada TX de telemetría (packTelemetryBuffer + Radio_SendPacket) — Bug #12 |
| 13-Jun-2026 | `docs/bugs-and-improvements.md` | Documentado Bug #12 (radio nRF24 no recibe) |
| 14-Jun-2026 | `Core/Inc/system_globals.h` | Agregadas declaraciones: wheel_pid_params, test_enable, test_motor, test_voltage |
| 14-Jun-2026 | `Core/Src/system_globals.c` | Agregadas definiciones: wheel_pid_params per-motor, test_enable/motor/voltage |
| 14-Jun-2026 | `Core/Src/drive_task.c` | Init con wheel_pid_params[i], watchdog salta en test mode, loop con bypass open-loop |
| 14-Jun-2026 | `engine/.../api/control.rs` | Agregada función Lua test_motor_step(id, motor_idx, voltage) |
| 14-Jun-2026 | `engine/engine/lua/api_globals.lua` | Documentada test_motor_step |
| 14-Jun-2026 | `lua/api_globals.lua` | Documentada test_motor_step |
| 14-Jun-2026 | `engine/engine/lua/characterize.lua` | Script de caracterización multi-step creado |
| 14-Jun-2026 | `docs/motor-characterization.md` | Documento de caracterización de motores y ajuste PID |
| 30-Jun-2026 | — | Auditoría: Bug #13 (test mode no implementado), Bug #14 (kick_sel auto-reset), Bug #15 (vth mask), Bug #16 (vz ignorado) |
| 30-Jun-2026 | `.omo/plans/implementation-plan.md` | Plan de implementación creado — bugs + smells + caracterización |
| 09-Jul-2026 | `Core/Src/main.c` | Migracion CMSISv1->v2: osThreadDef->osThreadNew+osThreadAttr_t, osMessageQ->osMessageQueue |
| 09-Jul-2026 | `Core/Src/accel_task.c` | CMSISv2: osKernelSysTick->osKernelGetTickCount, osDelayUntil v1->v2, void const*->void* |
| 09-Jul-2026 | `Core/Src/drive_task.c` | CMSISv2: osMutexWait->osMutexAcquire, osMessagePut->osMessageQueuePut |
| 09-Jul-2026 | `Core/Src/radio_task.c` | CMSISv2: void const*->void*, osMessageGet->osMessageQueueGet |
| 09-Jul-2026 | `Core/Src/kick_task.c` | CMSISv2: void const*->void*, osMessageGet->osMessageQueueGet, osMutexWait->osMutexAcquire |
| 09-Jul-2026 | `Core/Src/system_globals.c` | Eliminados HAL handles duplicados con main.c (fix linker error) |
| 09-Jul-2026 | `Core/Inc/system_globals.h` | CMSISv2: tipos actualizados (osThreadId_t, osMessageQueueId_t, osMutexId_t) |
| 09-Jul-2026 | `Core/Src/system_init.c` | Refactor: eliminados MX_*_Init duplicados -> solo MX_Init_Custom() con overrides |
| 09-Jul-2026 | `Core/Src/main.c` | TIM7->vTaskNotifyGiveFromISR(driveTaskHandle), radioTask+kickTask creation, kickQueue, Error_Handler restaurado, cinematica trig, Motor_Init DISABLE |
| 09-Jul-2026 | `Core/Inc/main.h` | Added `#include "system_globals.h"` |
| 11-Jul-2026 | `Tools/motor.c` | Clamped voltage to [0, 4095] before DAC write — Bug #17 |
| 11-Jul-2026 | `Core/Src/drive_task.c` | Rate limiter statics reset on radio timeout — Bug #18 |
| 11-Jul-2026 | `Tools/ekf.c` | EKF P matrix updated with temp arrays to prevent in-place corruption — Bug #19 |
| 11-Jul-2026 | `Tools/ekf.c` | sin/cos renormalized after state update — Bug #20 |
| 11-Jul-2026 | `Core/Src/drive_task.c` | PID integral zeroing condition changed to error-based — Bug #21 |
| 11-Jul-2026 | `Core/Src/drive_task.c` | Theta rate limiter moved to angular velocity output — Bug #22 |
| 11-Jul-2026 | `Core/Src/kick_task.c` | osMessageQueueGet timeout changed from osWaitForever to 5000ms — Bug #23 |
| 11-Jul-2026 | `engine/.../logger.rs` | BufWriter flush every 60 frames instead of every frame — Bug #24 |
| 11-Jul-2026 | `Core/Src/drive_task.c` + `radio_task.c` + `drive_task.h` | Removed unused velocity/turn params from processRadioCommand — Bug #25 |
| 11-Jul-2026 | `Tools/PID.c` + `PID.h` | Removed dead code PID_UpdateController — Bug #26/#8 |
| 11-Jul-2026 | `Core/Inc/system_globals.h` | Moved #include below header guard — Bug #27 |
| 11-Jul-2026 | `engine/.../tracker.rs` | Removed per-frame set_noise_parameters() call — Bug #28 |
| 11-Jul-2026 | `engine/.../logger.rs` | Added millisecond precision to timestamp — Bug #29 |
| 11-Jul-2026 | `Tools/nrf24.c` | EN_AA changed from 0x3F to 0x01 (pipe 0 only) — Bug #30 |
| 13-Jul-2026 | `Core/Src/kick_task.c` | Reestructurado: pre-carga capacitor al inicio, helpers charge_capacitor/fire_kicker — Bug #31 |
| 13-Jul-2026 | `Core/Src/drive_task.c` | kick_sel=0 post-disparo, /*ball_possession*/ comentado — Bug #31 |
| 13-Jul-2026 | `Tools/motor.h` | WHEEL_GEAR_RATIO=3.0f, SPEED_CNT_RATIO usa gear ratio — Bug #32 |
| 13-Jul-2026 | `Core/Src/drive_task.c` | Odometría: measSpeed / WHEEL_GEAR_RATIO, deadband escalado — Bug #32 |
| 13-Jul-2026 | `Core/Src/radio_task.c` | Telemetría: estado EKF (x/y/vx/vy/θ/ω) reemplaza wheel speeds — Bug #32 |
| 09-Jul-2026 | - | Revision completa de consistencia contra backup: 6 issues corregidos |
| 23-Jul-2026 | `basestation/Src/main.c` | Dual RF channel: rx_device on 0x6C, tx_node_addr for pipe1, non-blocking UART TX — Bugs #33, #34, #36 |
| 23-Jul-2026 | `basestation/Src/stm32f1xx_it.c` | USART1 IRQ handler: only call RxCpltCallback on IDLE flag — Bug #35 |
| 23-Jul-2026 | `basestation/Src/stm32f1xx_hal_msp.c` | USART1 AFIO remap to PB6/PB7 (PA9/PA10 burned) |
| 23-Jul-2026 | `Core/Src/radio_task.c` | Added TelemetryFunction for async telemetry on channel 0x6C with radioMutex |
| 23-Jul-2026 | `Core/Inc/system_globals.h` | Added TELEMETRY_CHANNEL (0x6C), TELEMETRY_PERIOD_MS (10) |
| 23-Jul-2026 | `engine/.../sender/radio.rs` | Changed read() to read_exact() + added break after successful parse |
