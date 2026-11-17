# Caracterización de Motores y Ajuste PID

## Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────────────┐
│                         PC Engine (Rust)                            │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  Lua script (characterize.lua)                              │   │
│  │  test_motor_step(id, motor, voltage)                        │   │
│  └──────────┬───────────────────────────────────────────────────┘   │
│             │ test_motor_step(id, motor, voltage)                   │
│             ▼                                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  radio.rs → packet_serializer.rs                            │   │
│  │  vx = motor_idx (0-3), vy = voltage/4095*5.11               │   │
│  └──────────┬───────────────────────────────────────────────────┘   │
└─────────────┼───────────────────────────────────────────────────────┘
              │ USB Serial → Base Station → nRF24
┌─────────────┼───────────────────────────────────────────────────────┐
│  Robot (STM32F767ZI)                                                │
│              ▼                                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  radio_task.c → processRadioCommand()                       │   │
│  │  Si test_enable==1:                                          │   │
│  │    v_vel[0] → test_motor (motor index)                     │   │
│  │    v_vel[1] → test_voltage (DAC code -4095..4095)          │   │
│  └──────────┬───────────────────────────────────────────────────┘   │
│             │ test_motor, test_voltage (globals visibles)            │
│             ▼                                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  drive_task.c loop (1 kHz)                                   │   │
│  │  Si test_enable==1:                                          │   │
│  │    i != test_motor → disable + brake                       │   │
│  │    i == test_motor → open-loop DAC + encoder reading       │   │
│  └──────────┬───────────────────────────────────────────────────┘   │
│             │ motor[i].measSpeed (rad/s)                            │
│             ▼                                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  STM32Monitor @1kHz via STLink                              │   │
│  │  Log: time, test_voltage, motor[i].measSpeed, ...           │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

## Cambios Realizados

### Variables Globales Nuevas

| Variable | Tipo | Descripción |
|---|---|---|
| `wheel_pid_params[4]` | `PID_Params_t[4]` | Parámetros PID individuales por rueda |
| `test_enable` | `uint8_t` | 0=normal, 1=test mode |
| `test_motor` | `uint8_t` | Índice del motor a testear (0-3) |
| `test_voltage` | `float` | Voltaje DAC (-4095 a 4095) |

### Archivos Modificados

#### `Core/Inc/system_globals.h`
- Declaraciones de `wheel_pid_params`, `test_enable`, `test_motor`, `test_voltage`

#### `Core/Src/system_globals.c`
- `wheel_pid_params[4]` inicializado con los valores default (Kp=25, Ki=44, Kd=0)
- `test_enable=0`, `test_motor=0`, `test_voltage=0.0f`

#### `Core/Src/drive_task.c`

**Inicialización** (línea 94-103):
```c
// Antes: PID params fijos, iguales para todos
PID_Params_t pidParams = {25, 44, 0, 4095, -4095, 4095, 0.001};
PID_Init(&motor[i].pid, pidParams, PID_STATUS_ENABLE);

// Después: params individuales modificables via STM32Monitor
PID_Init(&motor[i].pid, wheel_pid_params[i], PID_STATUS_ENABLE);
```

**Radio watchdog** (línea 144):
```c
// Antes: siempre activo
if (radio_ever_received && radio_now - radio_last_rx_tick > RADIO_TIMEOUT_MS)

// Después: se salta en test mode
if (!test_enable && radio_ever_received && ...)
```

**Loop de motores** (línea 171-203):
```c
// Cuando test_enable == 1:
//   i != test_motor → disable + brake, measSpeed = 0
//   i == test_motor → DAC directo (MAX581x_CodeLoad) + lectura encoder
// Cuando test_enable == 0: comportamiento original (Motor_CLDrive)
```

**`processRadioCommand()`** (línea 274-319):
```c
// Cuando test_enable == 1:
//   v_vel[0] (vx) → test_motor = round(fabs(vx))
//   v_vel[1] (vy) → test_voltage = vy / 5.11 * 4095
//   return (salta límites de velocidad/aceleración)
// Cuando test_enable == 0: comportamiento original
```

#### `engine/engine/src/lua_interface/api/control.rs`
Nueva función Lua `test_motor_step(id, motor_idx, voltage)`:
- Escala voltage (-4095..4095) a vy (-5.11..5.11)
- Usa motor_idx como vx
- Bypass del check de robot activo (funciona sin visión)
- Team forzado a 0 (blue)

#### `engine/engine/lua/api_globals.lua` y `lua/api_globals.lua`
- Documentación de `test_motor_step`

### Archivos Creados

#### `engine/engine/lua/characterize.lua`
Script de caracterización multi-step. State machine que:
- Itera sobre 4 motores, 8 steps cada uno (±300, ±600, ±900, ±1200 DAC)
- ~833ms entre steps (50 frames a ~60 FPS)
- Loggea progreso por consola

---

## Flujo de Caracterización

### Paso 1: Captura de datos

```
1. Flashear firmware con los cambios
2. Conectar STLink, abrir STM32Monitor
3. En live watch de STM32Monitor: test_enable = 1
4. Iniciar grabación CSV
5. En engine: Load Script → engine/engine/lua/characterize.lua
6. El script ejecuta la secuencia automáticamente
7. Detener grabación cuando el script indique "Complete!"
```

Columnas del CSV generado:
- `time` — timestamp (s)
- `test_voltage` — escalón de voltaje aplicado
- `motor[0].measSpeed` — respuesta del motor 0 (rad/s)
- `motor[1..3].measSpeed` — respuestas (0 durante el test)
- Otras variables globales según configuración de STM32Monitor

### Paso 2: Análisis Open-Loop (FOPDT)

```bash
# Extraer modelo FOPDT (K, L, T) + tuning rules
python3 analysis/pid_tuner.py Log_*.csv \
  --cmd test_voltage -m "motor[0].measSpeed" --open-loop
```

El tuner identifica:
- **K** — ganancia del proceso (rad/s por unidad de DAC)
- **L** — tiempo muerto (ms)
- **T** — constante de tiempo (ms)
- Relación T/L → controlabilidad

Aplica reglas de sintonía:
| Método | Característica |
|---|---|
| Ziegler-Nichols | Agresivo, punto de partida |
| Tyreus-Luyben | Conservador |
| IMC | Robusto, recomendado para PI |

### Paso 3: Ajuste en Vivo

Las ganancias se modifican en STM32Monitor sin reflashear:
```
wheel_pid_params[0].Kp = 32.5
wheel_pid_params[0].Ki = 58.1
wheel_pid_params[0].Kd = 0.0
```

### Paso 4: Verificación Closed-Loop

```bash
# Volver a modo normal: test_enable = 0
# Las ruedas reciben comandos vía radio (v_vel)
# Capturar step response con STM32Monitor

python3 analysis/csv_analyzer.py step_vth.csv -c v_vel[2] -a
python3 analysis/pid_tuner.py step_vth.csv \
  -c v_vel[2] -m "motor[0].measSpeed" --current Kp,Ki,Kd
```

---

## Secuencia de Calibración Completa

```
Fase 1: Wheel PID (inner loop, 1 kHz)
  ├─ Test mode: step voltage → motor[i].measSpeed
  ├─ Extraer FOPDT con pid_tuner.py --open-loop
  ├─ Cargar ganancias en wheel_pid_params[i] via STM32Monitor
  ├─ Closed-loop: step refSpeed → measSpeed
  └─ Target: rise < 30ms, sin overshoot

Fase 2: Vth PID (outer loop, 200 Hz)
  ├─ test_enable=0, pid_vx_enable=0, pid_vy_enable=0, pid_vth_enable=0
  ├─ Step v_vel[2] → vth_enc (lazo abierto)
  ├─ Extraer FOPDT + tuning
  ├─ Cargar ganancias en pid_vth (drive_task.c)
  ├─ pid_vth_enable=1, step v_vel[2] → vth_est
  └─ Target: rise < 100ms, OS < 10%

Fase 3: Vx/Vy PID
  └─ Misma metodología que Vth
```

---

## Historial de Cambios

| Fecha | Archivo | Cambio |
|---|---|---|
| 14-Jun-2026 | `Core/Inc/system_globals.h` | Agregadas declaraciones: wheel_pid_params, test_enable, test_motor, test_voltage |
| 14-Jun-2026 | `Core/Src/system_globals.c` | Agregadas definiciones: wheel_pid_params (default Kp=25,Ki=44), test_enable=0, test_motor=0, test_voltage=0 |
| 14-Jun-2026 | `Core/Src/drive_task.c` | Init usa wheel_pid_params[i], radio watchdog salta en test mode, loop con test mode (open-loop DAC directo), processRadioCommand mapea vx/vy a test_motor/test_voltage |
| 14-Jun-2026 | `engine/.../api/control.rs` | Agregada función Lua test_motor_step(id, motor_idx, voltage) |
| 14-Jun-2026 | `engine/engine/lua/api_globals.lua` | Documentada test_motor_step |
| 14-Jun-2026 | `lua/api_globals.lua` | Documentada test_motor_step |
| 14-Jun-2026 | `engine/engine/lua/characterize.lua` | Script de caracterización multi-step creado |
| 14-Jun-2026 | `docs/motor-characterization.md` | Este documento |
