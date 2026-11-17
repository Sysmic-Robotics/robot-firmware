# Plan Integral: Bugs + Code Smells + Caracterización

**Fecha**: 2026-06-30
**Estado**: ✅ Aprobado (Pendiente de ejecución)
**Contexto**: stm32monitor disponible para leer/escribir variables en caliente sin reflashear

---

## Resumen

Plan completo para resolver bugs críticos, sincronizar documentación, limpiar code smells, y habilitar el pipeline de caracterización de controladores en el firmware STM32F767ZI del robot SysmicProyect.

---

## Fase 1: 🐛 Bugs Críticos

### 1.1 Bug #1 — Test mode mapping faltante en processRadioCommand

**Dónde**: `Core/Src/drive_task.c` — función `processRadioCommand()`
**Problema**: La documentación dice que `vx→test_motor` y `vy→test_voltage` cuando `test_enable==1`, pero el código no implementa ese mapeo. La caracterización de motores no funciona.
**Fix**: Agregar al inicio de `processRadioCommand()`:

```c
if (test_enable) {
    // Map radio vel fields to test variables for open-loop motor characterization
    test_motor = (uint8_t)(fabsf(v_vel[0]) * 10.0f);  // vx → motor index (0-3)
    if (test_motor > 3) test_motor = 3;
    test_voltage = v_vel[1] * 1000.0f;                  // vy → voltage command
    return;  // skip normal control when in test mode
}
```

**Verificación con stm32monitor**:
```bash
# Escribir test_enable = 1
# Escribir vx, vy vía radio (o simular buffer)
# Leer test_motor, test_voltage para confirmar mapeo
```

---

### 1.2 Bug #2 — Auto-rekick infinito (kick_sel nunca se resetea)

**Dónde**: `Core/Src/radio_task.c` (escritura) + `Core/Src/kick_task.c` (lectura)
**Problema**: `kick_sel` se escribe desde radio pero NUNCA se resetea a 0, causando un ciclo perpetuo de carga/disparo.
**Fix**: Implementar detección de flanco (edge-triggered). Opción recomendada:

En `radio_task.c:RadioFunction()`, cambiar:
```c
kick_sel = getKickTriggerFlag(nrf_device.rx_data + 5 * robot_id);
```
Por detección de flanco:
```c
static uint8_t prev_kick = 0;
uint8_t cur_kick = getKickTriggerFlag(nrf_device.rx_data + 5 * robot_id);
if (cur_kick && !prev_kick) {
    kick_sel = 1;
} else {
    kick_sel = 0;  // auto-reset
}
prev_kick = cur_kick;
```

O alternativamente, en `drive_task.c:DriveFunction()`, después de encolar el disparo:
```c
if(kick_sel && kick_flag == KICKER_CHARGED) {
    osMutexWait(kickFlagHandle, osWaitForever);
    kick_flag = KICKER_START;
    osMutexRelease(kickFlagHandle);
    osMessagePut(kickQueueHandle, 0, 0);
    kick_sel = 0;  // ← auto-reset aquí
}
```

**Verificación con stm32monitor**:
```bash
# Live watch: kick_sel, kick_flag, kick_count
# Forzar kick_sel = 1 desde monitor
# Observar: kick_flag debe pasar por 0→CARGANDO→CARGADO→DESCARGADO
# Confirmar que kick_sel vuelve a 0 automáticamente
# Confirmar que kick_count aumenta solo 1 por disparo
```

---

### 1.3 Bug #3 — Máscara de decodificación vth incorrecta

**Dónde**: `Core/Src/drive_task.c:284`
**Problema**: `v_vel[2]` se decodifica con `(buffer[4] & 0x0F)` (4 bits = 11-bit total), pero el protocolo especifica 3 bits MSB en `buffer[4][3:2]` para 10-bit total. Máscara correcta: `0x0E` → `0x0C` (bits [3:2]).
**Fix**: Cambiar en línea 284:

```c
// ANTES (11-bit - incorrecto):
v_vel[2] = (buffer[3] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0x0F) << 7 | (uint16_t)(buffer[3] & 0x7F))/100.0f
                                : (float)((uint16_t)(buffer[4] & 0x0F) << 7 | (uint16_t)(buffer[3] & 0x7F))/100.0f;

// DESPUÉS (10-bit - correcto):
v_vel[2] = (buffer[3] & 0x80) ? -(float)((uint16_t)(buffer[4] & 0x0C) << 7 | (uint16_t)(buffer[3] & 0x7F))/100.0f
                                : (float)((uint16_t)(buffer[4] & 0x0C) << 7 | (uint16_t)(buffer[3] & 0x7F))/100.0f;
```

**Verificación**: Inspeccionar que `v_vel[2]` decode correctamente el rango ±5.11 rad/s (10-bit signed), no ±10.23 (11-bit).

---

## Fase 2: 📄 Sincronizar Documentación con Código Real

### 2.1 PID values: Docs → código real

**Dónde**: `docs/characterization-guide.md`
**Qué**: Actualizar valores PID para reflejar el código real. Cambios exactos:

**Línea 15**: Diagrama de arquitectura de control en cascada
```diff
- │  │  pid_vth: Kp=5.0 Ki=2.0        │ │
+ │  │  pid_vth: Kp=10.0 Ki=5.0       │ │
```

**Línea 20**: Wheel PID en diagrama
```diff
- │  │  motor[0..3]: Kp=25 Ki=44      │ │
+ │  │  motor[0..3]: Kp=50 Ki=5       │ │
```

**Sección 2.6 (línea 425)**: Valores de referencia Vth PID
```diff
- pid_vth: Kp=5.0, Ki=2.0, outputMax=5.0, outputMin=-5.0, integralMax=2.0
+ pid_vth: Kp=10.0, Ki=5.0, outputMax=10.0, outputMin=-10.0, integralMax=10.0
```

**Sección 3.5 (línea 634)**: Valores actuales
```diff
- Valores actuales: `Kp=0.8, Ki=1.6, outputMax=1.5, outputMin=-1.5`
+ Valores actuales: `pid_vx/vy: Kp=0.8, Ki=1.6, outputMax=1.5, outputMin=-1.5`
```

Agregar nota al inicio del documento: *"Los valores PID listados son los defaults de fábrica. Se actualizarán tras caracterización FOPDT mediante stm32monitor y luego se persisten en system_globals.c."*

### 2.2 Bugs-and-improvements.md: Registrar nuevos bugs y el plan

**Dónde**: `docs/bugs-and-improvements.md`
**Qué**: Agregar bugs encontrados en la auditoría (30-Jun-2026) y referenciar el plan.

**Nuevos bugs a agregar en la sección "Bugs Conocidos (No corregidos aún)"**:

```
### #13 — Test mode mapping faltante en processRadioCommand

**Archivo**: `Core/Src/drive_task.c:274-306`
**Severidad**: 🔴 Crítico

**Problema**: La documentación (`docs/characterization-guide.md` línea 65 y `docs/motor-characterization.md`) indica que `processRadioCommand` mapea vx→test_motor y vy→test_voltage cuando test_enable==1. Sin embargo, el código de `processRadioCommand()` solo decodifica v_vel[0..2] sin verificar test_enable ni actualizar test_motor/test_voltage. Esto hace que la caracterización de motores (characterize.lua + test_motor_step) no funcione.

**Fix**: Ver `.omo/plans/implementation-plan.md` — Fase 1.1

### #14 — kick_sel nunca se resetea (auto-rekick infinito)

**Archivo**: `Core/Src/radio_task.c:56`, `Core/Src/kick_task.c:11`
**Severidad**: 🔴 Crítico

**Problema**: `kick_sel` se escribe desde radio_task pero nunca se resetea a 0. Una vez activado, kick_task queda en ciclo perpetuo: carga 4s → espera → dispara → vuelve a cargar inmediatamente. Bug #6 parcialmente relacionado.

**Fix**: Ver `.omo/plans/implementation-plan.md` — Fase 1.2

### #15 — Decodificación vth con máscara incorrecta (11-bit vs 10-bit)

**Archivo**: `Core/Src/drive_task.c:284`
**Severidad**: 🟡 Alto

**Problema**: `v_vel[2]` usa `(buffer[4] & 0x0F) << 7` (4 bits = 11-bit total) pero el protocolo especifica bits [3:2] de buffer[4] = 2 bits MSB para 10-bit total. Máscara correcta: `0x0C`. Rango incorrecto: ±10.23 rad/s en vez de ±5.11 rad/s.

**Fix**: Ver `.omo/plans/implementation-plan.md` — Fase 1.3

### #16 — vz (dribbler vertical velocity) ignorado en control loop

**Archivo**: `Core/Src/drive_task.c`
**Severidad**: 🟡 Medio

**Problema**: El protocolo de radio envía velocidad vertical (vz) para el dribbler en algunos paquetes, pero el firmware no lo procesa. Dribbler se controla solo por `dribbler_sel` (bits [4:2] del byte 0).

**Estado**: Aceptado (diseño actual usa dribbler_sel como control discreto 0-7).
```

**Además**: Corregir entrada histórica errónea en línea 234:
```diff
- | 14-Jun-2026 | `Core/Src/drive_task.c` | ... processRadioCommand mapea test mode |
+ | 14-Jun-2026 | `Core/Src/drive_task.c` | Init con wheel_pid_params[i], watchdog salta en test mode, loop con bypass open-loop |
+ | 30-Jun-2026 | — | Auditoría: Bug #13 (test mode no implementado), Bug #14 (kick_sel auto-reset), Bug #15 (vth mask), Bug #16 (vz ignorado) |
+ | 30-Jun-2026 | `.omo/plans/implementation-plan.md` | Plan de implementación creado — bugs + smells + caracterización |
```

**Agregar mejoras pendientes**:
```
| # | Mejora | Impacto |
| 7 | Cachear dribbler_sel para evitar 1000 I2C writes/s (solo escribir si cambia) | Bajo |
| 8 | Separar robot_pid_enable en flags individuales por eje (vx/vy/vth) | Medio |
```

### 2.3 Eliminar parámetros velocity/turn de processRadioCommand

**Dónde**: `Core/Src/drive_task.c` y `Core/Inc/drive_task.h` (si existe)
**Qué**: Simplificar la firma de `processRadioCommand`:
```c
// ANTES:
void processRadioCommand(uint8_t *buffer, float *velocity, uint8_t *turn);

// DESPUÉS:
void processRadioCommand(uint8_t *buffer);
```

Actualizar también el `extern` en radio_task.h o donde se declare, y la llamada en `radio_task.c:54`:
```c
// ANTES:
processRadioCommand(nrf_device.rx_data + 5 * robot_id, speed, direction);

// DESPUÉS:
processRadioCommand(nrf_device.rx_data + 5 * robot_id);
```

---

## Fase 3: 🔧 Soporte para Caracterización (con stm32monitor)

### 3.1 Separar robot_pid_enable en 3 flags por eje

**Dónde**: `Core/Inc/system_globals.h`, `Core/Src/system_globals.c`, `Core/Src/drive_task.c`
**Qué**: Reemplazar `robot_pid_enable` por `pid_vx_enable`, `pid_vy_enable`, `pid_vth_enable`.
**Por qué**: Permite deshabilitar PID por eje desde STM32Monitor sin reflashear — esencial para caracterización Vth sin afectar Vx/Vy.

**En `system_globals.h`**:
```c
// REEMPLAZAR:
extern uint8_t robot_pid_enable;
// POR:
extern uint8_t pid_vx_enable;
extern uint8_t pid_vy_enable;
extern uint8_t pid_vth_enable;
```

**En `system_globals.c`**:
```c
// REEMPLAZAR:
uint8_t robot_pid_enable = 0;
// POR:
uint8_t pid_vx_enable = 1;
uint8_t pid_vy_enable = 1;
uint8_t pid_vth_enable = 1;
```

**En `drive_task.c:235-252`**:
```c
// REEMPLAZAR:
if (robot_pid_enable) {
    PID_CloseLoop(&pid_vx, vx_cmd, vx_meas);
    PID_CloseLoop(&pid_vy, vy_cmd, vy_meas);
    PID_CloseLoop(&pid_vth, vth_cmd, vth_meas);
    vx_cmd = pid_vx.output;
    vy_cmd = pid_vy.output;
    vth_cmd = pid_vth.output;
}
// POR:
if (pid_vx_enable) {
    PID_CloseLoop(&pid_vx, vx_cmd, vx_meas);
    vx_cmd = pid_vx.output;
}
if (pid_vy_enable) {
    PID_CloseLoop(&pid_vy, vy_cmd, vy_meas);
    vy_cmd = pid_vy.output;
}
if (pid_vth_enable) {
    PID_CloseLoop(&pid_vth, vth_cmd, vth_meas);
    vth_cmd = pid_vth.output;
}
```

También actualizar referencias a `robot_pid_enable` en watchdog (línea 153, 167).

**Verificación con stm32monitor**:
```bash
# Live watch: pid_vx_enable, pid_vy_enable, pid_vth_enable, v_vel[0..2], vx_est, vy_est, vth_est
# Forzar pid_vth_enable = 0, enviar vth por radio
# Confirmar que vth_cmd = v_vel[2] directo (sin PID)
```

### 3.2 Documentar flujo de caracterización con stm32monitor

**Dónde**: `docs/characterization-guide.md`
**Qué**: Actualizar los procedimientos para usar stm32monitor como herramienta principal: live watch de variables, escritura en caliente de PID params, grabación CSV.

---

## Fase 4: 🧹 Code Smells + Mejoras

### 4.1 M5 — Mover handles DAC a static

**Dónde**: `Core/Src/drive_task.c`
**Qué**: `driveDAC` y `dribblerDAC` son actualmente locales en el stack del loop infinito (DriveFunction). Moverlos a `static` fuera del loop:

```c
// Fuera del for(;;):
static MAX581x_Handler_t driveDAC;
static MAX581x_Handler_t dribblerDAC;

// En init (1 sola vez):
MAX581x_Init(&driveDAC, &hi2c1, MAX581x_REF_20);
MAX581x_Init(&dribblerDAC, &hi2c2, MAX581x_REF_20);
```

**Beneficio**: ~200 bytes de stack liberados y las estructuras persisten entre ciclos.

### 4.2 M9 — Cachear dribbler_sel para evitar I2C writes innecesarios

**Dónde**: `Core/Src/drive_task.c:206`
**Qué**: Solo escribir DAC si `dribbler_sel` cambió:

```c
// ANTES:
MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, Dribbler_SpeedSet[dribbler_sel]);

// DESPUÉS:
static uint8_t prev_dribbler_sel = 0xFF;
if (dribbler_sel != prev_dribbler_sel) {
    MAX581x_Code(&dribblerDAC, MAX581x_OUTPUT_A, Dribbler_SpeedSet[dribbler_sel]);
    prev_dribbler_sel = dribbler_sel;
}
```

**Beneficio**: Reduce I2C writes de 1000/s a ~10-20/s (solo cuando cambia), liberando el bus I2C.

### 4.3 M4 — Simplificar includes legacy F0/F4/F7 en Tools

**Dónde**: `Tools/PID.h`, `Tools/motor.h`, `Tools/encoder.h`, `Tools/MAX581x.h`, `Tools/MAX5814.h`
**Qué**: Eliminar los bloques condicionales `#if defined(STM32F0xx) ... #elif defined(STM32F4xx) ... #elif defined(STM32F7xx)` ya que el proyecto solo compila para STM32F767ZI. Dejar solo el include directo `#include "stm32f7xx_hal.h"`.

**Ejemplo (PID.h:18-52)**: Reducir 35 líneas de condicionales a solo `#include "stm32f7xx_hal.h"`.

### 4.4 M8 — Decisión sobre ball_detector_task

**Dónde**: `Core/Src/main.c`
**Qué**: Dejar comentado como está. NO eliminar referencias en system_globals.h — el hardware VL6180X puede agregarse después. Agregar un comentario claro:

```c
// ball_detector_task: Desactivada. Hardware VL6180X no presente.
// Para activar: descomentar y configurar I2C4.
```

---

## Fase 5: 🧪 Pipeline Completo de Caracterización (Documentación)

### 5.1 Actualizar docs/characterization-guide.md

Incluir:
- Valores PID sincronizados con código real
- Procedimientos actualizados con stm32monitor
- Nota sobre variables live-watch recomendadas para cada fase
- Comandos de stm32monitor para modificar gains en caliente
- Flujo de caracterización: Motor → Vth → Vx/Vy → Validación mixta

### 5.2 Actualizar docs/bugs-and-improvements.md

Agregar sección de plan de implementación.

---

## Dependencias Entre Tareas

```
Fase 1 (Bugs)
  ├── 1.1 Test mode mapping  ← Sin dependencias
  ├── 1.2 Kick_sel reset     ← Sin dependencias
  └── 1.3 Vth mask fix       ← Sin dependencias

Fase 2 (Docs sync)
  ├── 2.1 PID values         ← Sin dependencias
  ├── 2.2 Bugs-and-improvements ← Después de escribir el plan
  └── 2.3 velocity/turn      ← Sin dependencias

Fase 3 (Caracterización)
  └── 3.1 Flags por eje      ← Sin dependencias (pero toca drive_task.c, ojo con conflictos con 1.1)

Fase 4 (Code smells)
  ├── 4.1 M5 (DAC static)    ← drive_task.c (posible conflicto con 1.1 y 3.1)
  ├── 4.2 M9 (dribbler cache)← drive_task.c (posible conflicto)
  ├── 4.3 M4 (includes)      ← Tools/*.h (sin conflictos)
  └── 4.4 M8 (ball_detector) ← main.c (sin conflictos)

Fase 5 (Docs)
  └── 5.1-5.2 update docs    ← Después de todas las fases (reflejar cambios finales)
```

> **Nota**: Las fases 1.1, 3.1, 4.1, y 4.2 modifican `drive_task.c`. Ejecutar secuencialmente para evitar conflictos de merge.

---

## TODOs

- [x] 1. Bug #1 — Test mode mapping en `processRadioCommand` (`Core/Src/drive_task.c`)
- [x] 2. Bug #2 — Auto-rekick infinito: `kick_sel` reset en `radio_task.c` + `kick_task.c`
- [x] 3. Bug #3 — Máscara vth incorrecta (`Core/Src/drive_task.c:284`)
- [x] 4. Actualizar PID values en `docs/characterization-guide.md` a valores reales del código
- [x] 5. Registrar bugs #13-#16 en `docs/bugs-and-improvements.md` + referenciar plan
- [x] 6. Eliminar parámetros `velocity`/`turn` de `processRadioCommand` (`drive_task.c` + `radio_task.c`)
- [x] 7. Separar `robot_pid_enable` → `pid_vx_enable`, `pid_vy_enable`, `pid_vth_enable`
- [x] 8. M5 — Mover handles DAC (`driveDAC`, `dribblerDAC`) a `static` (`drive_task.c`)
- [x] 9. M9 — Cachear `dribbler_sel` para evitar I2C writes innecesarios (`drive_task.c`)
- [x] 10. M4 — Simplificar includes legacy F0/F4/F7 en `Tools/*.h`
- [x] 11. M8 — Dejar ball_detector_task comentada con nota (`main.c`)
- [x] 12. Actualizar `docs/characterization-guide.md` con procedimientos stm32monitor
- [x] 13. Actualizar `docs/bugs-and-improvements.md` con sección del plan

## Final Verification Wave
- [x] F1. 🛡️ Oracle review: Verificar que bugs #1-#3 corregidos, sin regresiones
- [x] F2. 🔬 stm32monitor QA: Live-read de variables prueba bugs resueltos
- [x] F3. 📋 Momus plan review: Plan ejecutado completamente, checkboxes marcados
- [x] F4. 📄 Docs sync verification: characterization-guide.md + bugs-and-improvements.md sincronizados

## Criterios de Éxito

1. ✅ Radio → test_motor/test_voltage mapea correctamente (verificado con stm32monitor)
2. ✅ kick_sel se resetea automáticamente, sin ciclos infinitos
3. ✅ vth decode usa 10 bits, rango ±5.11 rad/s
4. ✅ processRadioCommand ya no tiene parámetros velocity/turn
5. ✅ PID flags independientes por eje, manejables desde stm32monitor
6. ✅ DAC handles son static, ~200 bytes de stack liberados
7. ✅ Dribbler I2C solo escribe cuando cambia `dribbler_sel`
8. ✅ Tools/*.h sin includes legacy F0/F4/F7
9. ✅ ball_detector_task comentada con razón documentada
10. ✅ Docs sincronizados (PID values, characterization-guide.md)
