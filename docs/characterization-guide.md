# Guía Completa de Caracterización de Controladores

> **NOTA:** Los valores PID listados son los defaults de fábrica. Se actualizarán tras caracterización FOPDT mediante stm32monitor y luego se persisten en system_globals.c.

### stm32monitor Integration

All characterization procedures use [stm32monitor](https://github.com/sysmic/stm32monitor) for live variable inspection and modification **without reflashing**. This enables rapid PID tuning and motor characterization.

**Key variables available via stm32monitor:**

| Variable | Type | Description | Live watch | Live write |
|----------|------|-------------|-----------|-----------|
| `test_enable` | uint8_t | Enable test mode (0/1) | ✅ | ✅ |
| `test_motor` | uint8_t | Selected motor index (0-3) | ✅ | ✅ |
| `test_voltage` | float | Motor voltage command (mV) | ✅ | ✅ |
| `pid_vx_enable` | uint8_t | Vx PID enable (0/1) | ✅ | ✅ |
| `pid_vy_enable` | uint8_t | Vy PID enable (0/1) | ✅ | ✅ |
| `pid_vth_enable` | uint8_t | Vth PID enable (0/1) | ✅ | ✅ |
| `v_vel[0..2]` | float[3] | Radio velocity commands (m/s, rad/s) | ✅ | ❌ |
| `wheel_pid_params[i].Kp` | float | Per-motor proportional gain | ✅ | ✅ |
| `wheel_pid_params[i].Ki` | float | Per-motor integral gain | ✅ | ✅ |

**Example session:**
```bash
# Live-watch motor speeds during characterization
stm32monitor live motor[0].measSpeed motor[1].measSpeed motor[2].measSpeed motor[3].measSpeed

# Disable Vth PID to characterize angular velocity
stm32monitor native write pid_vth_enable 0

# Tune Vx PID gains in real time
stm32monitor native write pid_vx.Kp 1.2
stm32monitor native write pid_vx.Ki 2.0
```

## Arquitectura de Control en Cascada

```
PC Engine                          Robot (STM32F767ZI)
┌────────────────────┐             ┌──────────────────────────────────────┐
│ Lua scripts        │   radio    │                                      │
│ send_velocity()    │ ────────→  │  processRadioCommand()              │
│ test_motor_step()  │            │    v_vel[0]=vx, [1]=vy, [2]=vth     │
└────────────────────┘            │                                      │
                                  │  ┌─ Robot PID (200 Hz) ───────────┐ │
                                  │  │  pid_vx: Kp=0.8  Ki=1.6        │ │
                                  │  │  pid_vy: Kp=0.8  Ki=1.6        │ │
                                  │  │  pid_vth: Kp=10.0 Ki=5.0       │ │
                                  │  └──────────┬──────────────────────┘ │
                                  │             │ speed[i] = kinematic   │
                                  │             ▼                        │
                                  │  ┌─ Wheel PID (1 kHz) ────────────┐ │
                                  │  │  motor[0..3]: Kp=50 Ki=5       │ │
                                  │  │  + ffVoltage=300 + IIR α=0.15  │ │
                                  │  └──────────┬──────────────────────┘ │
                                  │             │ DAC → H-Bridge         │
                                  │             ▼                        │
                                  │  Encoder ← DC Motor ← Wheel         │
                                  │             │                        │
                                  │  ┌─ Kalman Filters ───────────────┐ │
                                  │  │  vx_est = Kalman1D(enc, acc)  │ │
                                  │  │  vy_est = Kalman1D(enc, acc)  │ │
                                  │  │  vth_est = Kalman2D(enc, gyro)│ │
                                  │  └────────────────────────────────┘ │
                                  └──────────────────────────────────────┘
```

## Loop Cerrado (Closed-Loop)

```
v_cmd ─→[PID]──→[kinematic]──→[Wheel PID]──→[Motor Plant]──→[Encoders]
  ↑                                                            │
  └──────────────────── Kalman ────────────────────────────────┘
```

## Loop Abierto (Open-Loop para caracterización)

```
v_cmd ──────────────────────────→[kinematic]──→[Wheel PID]──→[Motor Plant]──→[Encoders]
                                                                              │
                                                                    vx_enc/vy_enc/vth_enc
```

---

# Fase 1: Caracterización de Motores (Wheel PID, Inner Loop @ 1 kHz)

**Planta**: `DAC_code [0-4095] → motor[i].measSpeed [rad/s]`

## 1.1 Preparación del Firmware

Los siguientes cambios ya están implementados en el firmware (ver `docs/motor-characterization.md`):

| Archivo | Cambio |
|---------|--------|
| `Core/Inc/system_globals.h` | Declaraciones de `test_enable`, `test_motor`, `test_voltage`, `wheel_pid_params[4]` |
| `Core/Src/system_globals.c` | Definiciones: `wheel_pid_params[i]` con defaults Kp=25, Ki=44, Kd=0 |
| `Core/Src/drive_task.c` | Init usa `wheel_pid_params[i]`, watchdog skip en test mode, loop con bypass open-loop DAC directo, `processRadioCommand` mapea vx/vy a test_motor/test_voltage |
| `engine/.../api/control.rs` | Función Lua `test_motor_step(id, motor_idx, voltage)` |

## 1.2 Script Lua: characterize.lua (ya existe)

**Archivo**: `engine/engine/lua/characterize.lua`

```lua
-- Caracterización multi-step de motores
-- Aplica escalones ±300, ±600, ±900, ±1200 DAC a cada motor secuencialmente
-- Usa test_motor_step(id, motor_idx, voltage)
```

**Configurar**: editar `ROBOT_ID` al ID del robot a testear.

## 1.3 Procedimiento de Test

```
1. Flashear firmware en el robot
2. Conectar STLink, abrir STM32Monitor
3. Agregar al live watch:
   - test_enable  (uint8)
   - test_motor   (uint8)
   - test_voltage (float)
   - motor[0].measSpeed (float)
   - motor[1].measSpeed
   - motor[2].measSpeed
   - motor[3].measSpeed
4. En STM32Monitor, forzar test_enable = 1
5. Iniciar grabación CSV (seleccionar variables del live watch)
6. En Engine: Lua > Load Script > engine/engine/lua/characterize.lua
7. Observar consola:
   === Motor Characterization ===
   Robot ID: 0, Team: 0
   Motor[0] +300  (1/8, motor 1/4)
   Motor[0] +600  (2/8, motor 1/4)
   ...
8. Cuando muestre "=== Characterization Complete! ===", detener grabación
```

### Motor Characterization with stm32monitor

1. Set `test_enable = 1` via stm32monitor:
   ```bash
   stm32monitor native write test_enable 1
   ```

2. Send a radio command (vx selects motor 0-3, vy sets voltage):
   - `vx = 0.1` → `test_motor = 1`
   - `vy = 0.5` → `test_voltage = 500 mV`

   Or write directly:
   ```bash
   stm32monitor native write test_motor 0
   stm32monitor native write test_voltage 500
   ```

3. Live-watch motor response:
   ```bash
   stm32monitor live motor[0].measSpeed motor[0].pid.output driveDAC.currentChannel
   ```

4. Record step response to CSV:
   ```bash
   stm32monitor record --vars motor[0].measSpeed,motor[0].pid.output --output motor0_step.csv
   ```

## 1.4 Análisis Open-Loop (FOPDT)

Para cada motor por separado:

```bash
# Motor 0
python3 analysis/pid_tuner.py Log_*.csv \
  --cmd test_voltage \
  -m "motor[0].measSpeed" \
  --open-loop

# Motor 1
python3 analysis/pid_tuner.py Log_*.csv \
  --cmd test_voltage \
  -m "motor[1].measSpeed" \
  --open-loop

# Motor 2
python3 analysis/pid_tuner.py Log_*.csv \
  --cmd test_voltage \
  -m "motor[2].measSpeed" \
  --open-loop

# Motor 3
python3 analysis/pid_tuner.py Log_*.csv \
  --cmd test_voltage \
  -m "motor[3].measSpeed" \
  --open-loop
```

### Interpretación del Output

```
FOPDT Plant Model
  Process gain K:  5.2345       (rad/s por unidad DAC)
  Dead time  L:    2.1 ms       (latencia pura)
  Time const T:    18.5 ms      (constante de tiempo)
  T / L ratio:     8.8          (> 3 → controlable)

Tuning Rules
  Method            Type          Kp          Ki            Kd
  Ziegler-Nichols   PI         0.0523     7.4818        0.0000
  Tyreus-Luyben     PI         0.0180     2.5780        0.0000
  IMC               PI         0.0286     4.0900        0.0000

Recommended (IMC-PI conservative)
  Kp = 0.0286
  Ki = 4.0900
  Kd = 0.0000
```

### Diagnóstico de T/L Ratio

| T/L | Significado | Acción |
|-----|-------------|--------|
| > 10 | Fácil de controlar | IMC-PI funciona bien |
| 3 – 10 | Controlable | ZN-PI o IMC-PI |
| < 3 | Dominado por tiempo muerto | PID va a oscilar, considerar feed-forward |

## 1.5 Aplicar Ganancias en Caliente

Modificar en vivo (sin reflashear) mediante stm32monitor:

```bash
# Motor 0
stm32monitor native write wheel_pid_params[0].Kp <valor>
stm32monitor native write wheel_pid_params[0].Ki <valor>
stm32monitor native write wheel_pid_params[0].Kd 0.0

# Motor 1
stm32monitor native write wheel_pid_params[1].Kp <valor>
stm32monitor native write wheel_pid_params[1].Ki <valor>
stm32monitor native write wheel_pid_params[1].Kd 0.0

# Motor 2
stm32monitor native write wheel_pid_params[2].Kp <valor>
stm32monitor native write wheel_pid_params[2].Ki <valor>
stm32monitor native write wheel_pid_params[2].Kd 0.0

# Motor 3
stm32monitor native write wheel_pid_params[3].Kp <valor>
stm32monitor native write wheel_pid_params[3].Ki <valor>
stm32monitor native write wheel_pid_params[3].Kd 0.0
```

O desde la GUI de STM32Monitor:
```
wheel_pid_params[0].Kp = <valor>
wheel_pid_params[0].Ki = <valor>
...
```

Si funciona bien, persistir en `Core/Src/system_globals.c` y reflashear.

## 1.6 Verificación Closed-Loop

```bash
1. STM32Monitor: test_enable = 0
2. En Engine, enviar velocidad constante a una rueda:
   send_velocity(0, 0, 0.5, 0, 0)  →  vx=0.5 m/s
3. Capturar CSV con:
   - v_vel[0] (cmd)
   - motor[0].measSpeed  (medición)
   - speed[0] (referencia del wheel PID)
4. Analizar:
   python3 analysis/csv_analyzer.py Log_*.csv -c v_vel[0]
   python3 analysis/pid_tuner.py Log_*.csv -c speed -m "motor[0].measSpeed" --current Kp,Ki,Kd
```

**Target**: rise < 30ms, sin overshoot, sin oscilaciones.

### Tabla de Ajuste Rápido (Wheel PID)

| Síntoma | Acción |
|---------|--------|
| Rise > 30ms | Aumentar Kp +20% |
| Overshoot > 5% | Reducir Kp -20% |
| Error estacionario | Aumentar Ki +20% |
| Oscilación sostenida | Reducir Kp -50%, reducir Ki |
| Ruido en medición | Aumentar α del IIR (actual α=0.15) |

---

# Fase 2: Caracterización de Vth (Robot Angular, Outer Loop @ 200 Hz)

**Planta**: `vth_cmd [rad/s] → vth_enc [rad/s]` (lazo abierto)
**Planta**: `vth_cmd [rad/s] → vth_est [rad/s]` (lazo cerrado, con Kalman)

## 2.1 Cambios en el Firmware (drive_task.c)

Para poder hacer open-loop en Vth (bypass del PID de robot), se usa el flag `pid_vth_enable`. Estos flags son **per-axis** y ya están implementados:

```c
// En drive_task.c:
if (pid_vth_enable) {
    PID_CloseLoop(&pid_vth, vth_cmd, vth_meas);
    vth_cmd = pid_vth.output;
}
if (pid_vx_enable) {
    PID_CloseLoop(&pid_vx, vx_cmd, vx_meas);
    vx_cmd = pid_vx.output;
}
if (pid_vy_enable) {
    PID_CloseLoop(&pid_vy, vy_cmd, vy_meas);
    vy_cmd = pid_vy.output;
}
```

Los flags se declaran en `system_globals.h`, se definen en `system_globals.c` (default = 0, deshabilitados — se habilitan explícitamente para pruebas cerradas), y se pueden modificar en vivo desde stm32monitor:

```bash
# Deshabilitar Vth PID para caracterización open-loop
stm32monitor native write pid_vth_enable 0

# Re-habilitar después de la caracterización
stm32monitor native write pid_vth_enable 1

# Deshabilitar solo un eje lineal
stm32monitor native write pid_vx_enable 0
```

Esto permite deshabilitar PID por eje independientemente desde stm32monitor sin reflashear.

## 2.2 Script Lua: characterize_vth.lua

**Crear archivo**: `engine/engine/lua/characterize_vth.lua`

```lua
-- characterize_vth.lua
-- Open-loop Vth characterization: applies omega steps with pid_vth_enable=0
--
-- Usage:
--   1. STM32Monitor: pid_vth_enable = 0
--   2. Add to live watch: v_vel[2], vth_enc, vth_est
--   3. Start CSV recording
--   4. Load this script in Engine
--   5. Stop recording when "Complete!"
--   6. Analyze:
--      python3 analysis/pid_tuner.py Log_*.csv -c v_vel[2] -m vth_enc --open-loop

local ID = 0
local TEAM = 0

-- Steps in rad/s (omega). Robot max ~15 rad/s
local STEPS = {
     0.5,  1.0,  1.5,  2.0,  3.0,  4.0,
    -0.5, -1.0, -1.5, -2.0, -3.0, -4.0
}

local WAIT = 50  -- frames (~833ms @ 60fps)
local step = 1
local wait = 0
local running = false

function process()
    if not running then
        running = true
        print("=== Vth Characterization (Open-Loop) ===")
        print(string.format("Robot ID: %d, Steps: %d, Wait: %d frames", ID, #STEPS, WAIT))
        print("Ensure pid_vth_enable=0 in STM32Monitor!")
        print("Start CSV recording now.")
    end

    if step > #STEPS then
        print("=== Vth Characterization Complete! ===")
        send_velocity(ID, TEAM, 0, 0, 0)
        running = false
        return
    end

    if wait > 0 then
        wait = wait - 1
        send_velocity(ID, TEAM, 0, 0, STEPS[step])
        return
    end

    print(string.format("Vth step %d/%d: %+.1f rad/s", step, #STEPS, STEPS[step]))
    send_velocity(ID, TEAM, 0, 0, STEPS[step])
    step = step + 1
    wait = WAIT
end
```

## 2.3 Script Lua: characterize_vth_closed.lua (para verificación closed-loop)

**Crear archivo**: `engine/engine/lua/characterize_vth_closed.lua`

```lua
-- characterize_vth_closed.lua
-- Closed-loop Vth verification: step response with pid_vth_enable=1
--
-- Usage:
--   1. STM32Monitor: pid_vth_enable = 1
--   2. Load this script, capture CSV
--   3. Analyze: pid_tuner.py --current Kp,Ki,Kd

local ID = 0
local TEAM = 0

-- Escalones de omega con PID habilitado (más pequeños, el PID ya compensa)
local STEPS = {1.0, 2.0, 3.0, -1.0, -2.0, -3.0}

local WAIT = 70   -- ~1.17s (más tiempo para ver settling)
local step = 1
local wait = 0
local running = false

function process()
    if not running then
        running = true
        print("=== Vth Closed-Loop Verification ===")
        print("Ensure pid_vth_enable=1 in STM32Monitor!")
        print("Start CSV recording now.")
    end

    if step > #STEPS then
        print("=== Vth Closed-Loop Complete! ===")
        send_velocity(ID, TEAM, 0, 0, 0)
        running = false
        return
    end

    if wait > 0 then
        wait = wait - 1
        send_velocity(ID, TEAM, 0, 0, STEPS[step])
        return
    end

    print(string.format("Vth step %d/%d: %+.1f rad/s", step, #STEPS, STEPS[step]))
    send_velocity(ID, TEAM, 0, 0, STEPS[step])
    step = step + 1
    wait = WAIT
end
```

## 2.4 Procedimiento de Test (Open-Loop)

```
1. Fijar robot al banco permitiendo solo rotación (eje Z libre)
2. stm32monitor:
   ```bash
   stm32monitor native write pid_vth_enable 0
   ```
   Agregar live watch: `v_vel[2]`, `vth_enc`, `vth_est`
3. Iniciar grabación CSV
   ```bash
   stm32monitor record --vars v_vel[2],vth_enc,vth_est --output vth_openloop.csv
   ```
4. Engine: Lua > Load Script > engine/engine/lua/characterize_vth.lua
5. Esperar "Complete!", detener grabación (Ctrl+C en stm32monitor record)
```

## 2.5 Análisis Open-Loop

```bash
# Extraer FOPDT de Vth
python3 analysis/pid_tuner.py Log_*.csv \
  -c v_vel[2] \
  -m vth_enc \
  --open-loop
```

### Interpretación

El output muestra la **planta completa**: comando de omega → velocidad angular medida por encoder (pasando por cinemática inversa + 4 wheel PIDs + motores + cinemática directa).

Si quieres ver el efecto sin Kalman vs con Kalman:
```bash
# Sin Kalman (encoder raw)
python3 analysis/pid_tuner.py Log_*.csv -c v_vel[2] -m vth_enc --open-loop

# Con Kalman
python3 analysis/pid_tuner.py Log_*.csv -c v_vel[2] -m vth_est --open-loop
```

## 2.6 Aplicar Ganancias Vth

Modificar en vivo mediante stm32monitor:

```bash
stm32monitor native write pid_vth.Kp <valor>
stm32monitor native write pid_vth.Ki <valor>
stm32monitor native write pid_vth.Kd 0.0
```

O desde la GUI de STM32Monitor:
```
pid_vth.params.Kp = <valor>
pid_vth.params.Ki = <valor>
pid_vth.params.Kd = 0.0
```

Valores de referencia desde `drive_task.c`:
```c
pid_vth: Kp=10.0, Ki=5.0, outputMax=10.0, outputMin=-10.0, integralMax=10.0
```

## 2.7 Verificación Closed-Loop

```bash
1. Ensure Vth PID is enabled:
   ```bash
   stm32monitor native write pid_vth_enable 1
   ```
2. Agregar live watch: `v_vel[2]`, `vth_est`  (cmd y estimación Kalman)
3. Iniciar grabación:
   ```bash
   stm32monitor record --vars v_vel[2],vth_est --output vth_closedloop.csv
   ```
4. Engine: Load Script > characterize_vth_closed.lua
5. Analizar:
   python3 analysis/csv_analyzer.py Log_*.csv -c v_vel[2] -a
   python3 analysis/pid_tuner.py Log_*.csv -c v_vel[2] -m vth_est --current Kp,Ki,Kd
```

**Target**: rise < 100ms, OS < 10%, sin oscilaciones.

### Tabla de Ajuste Rápido (Vth PID)

| Síntoma | Acción |
|---------|--------|
| Rise > 100ms | Aumentar Kp +30% |
| OS > 10% | Reducir Kp -20% (o reducir Ki) |
| Error estacionario ≠ 0 | Aumentar Ki |
| No alcanza setpoint por saturación | Aumentar outputMax (> 5.0) |
| Overshoot + lento | Aumentar Kd (empezar con 0.05) |

---

# Fase 3: Caracterización de Vx/Vy (Robot Lineal, Outer Loop @ 200 Hz)

**Planta**: `vx_cmd [m/s] → vx_enc [m/s]` o `vy_cmd [m/s] → vy_enc [m/s]`

## 3.1 Script Lua: characterize_vx.lua

**Crear archivo**: `engine/engine/lua/characterize_vx.lua`

```lua
-- characterize_vx.lua
-- Open-loop Vx characterization
--
-- Usage:
--   1. STM32Monitor: pid_vx_enable = 0
--   2. Add live watch: v_vel[0], vx_enc, vx_est
--   3. Start CSV recording → Load this script → Stop when "Complete!"
--   4. Analyze: python3 analysis/pid_tuner.py Log_*.csv -c v_vel[0] -m vx_enc --open-loop

local ID = 0
local TEAM = 0

-- Steps in m/s (robot max linear vel ~2.5 m/s)
local STEPS = {
     0.3,  0.5,  0.8,  1.0,  1.2,
    -0.3, -0.5, -0.8, -1.0, -1.2
}

local WAIT = 50
local step = 1
local wait = 0
local running = false

function process()
    if not running then
        running = true
        print("=== Vx Characterization (Open-Loop) ===")
        print("Ensure pid_vx_enable=0 in STM32Monitor!")
        print("Start CSV recording now.")
    end

    if step > #STEPS then
        print("=== Vx Characterization Complete! ===")
        send_velocity(ID, TEAM, 0, 0, 0)
        running = false
        return
    end

    if wait > 0 then
        wait = wait - 1
        send_velocity(ID, TEAM, STEPS[step], 0, 0)
        return
    end

    print(string.format("Vx step %d/%d: %+.1f m/s", step, #STEPS, STEPS[step]))
    send_velocity(ID, TEAM, STEPS[step], 0, 0)
    step = step + 1
    wait = WAIT
end
```

## 3.2 Script Lua: characterize_vy.lua

**Crear archivo**: `engine/engine/lua/characterize_vy.lua`

```lua
-- characterize_vy.lua
-- Open-loop Vy characterization
-- Same as characterize_vx.lua but using send_velocity(ID, TEAM, 0, STEPS[step], 0)

local ID = 0
local TEAM = 0

local STEPS = {
     0.3,  0.5,  0.8,  1.0,  1.2,
    -0.3, -0.5, -0.8, -1.0, -1.2
}

local WAIT = 50
local step = 1
local wait = 0
local running = false

function process()
    if not running then
        running = true
        print("=== Vy Characterization (Open-Loop) ===")
        print("Ensure pid_vy_enable=0 in STM32Monitor!")
        print("Start CSV recording now.")
    end

    if step > #STEPS then
        print("=== Vy Characterization Complete! ===")
        send_velocity(ID, TEAM, 0, 0, 0)
        running = false
        return
    end

    if wait > 0 then
        wait = wait - 1
        send_velocity(ID, TEAM, 0, STEPS[step], 0)
        return
    end

    print(string.format("Vy step %d/%d: %+.1f m/s", step, #STEPS, STEPS[step]))
    send_velocity(ID, TEAM, 0, STEPS[step], 0)
    step = step + 1
    wait = WAIT
end
```

## 3.3 Script Lua: characterize_vx_closed.lua

**Crear archivo**: `engine/engine/lua/characterize_vx_closed.lua`

```lua
-- characterize_vx_closed.lua
-- Closed-loop Vx verification with pid_vx_enable=1

local ID = 0
local TEAM = 0

local STEPS = {0.5, 1.0, 1.5, -0.5, -1.0, -1.5}
local WAIT = 70
local step = 1
local wait = 0
local running = false

function process()
    if not running then
        running = true
        print("=== Vx Closed-Loop Verification ===")
        print("Ensure pid_vx_enable=1 in STM32Monitor!")
        print("Start CSV recording now.")
    end

    if step > #STEPS then
        print("=== Vx Closed-Loop Complete! ===")
        send_velocity(ID, TEAM, 0, 0, 0)
        running = false
        return
    end

    if wait > 0 then
        wait = wait - 1
        send_velocity(ID, TEAM, STEPS[step], 0, 0)
        return
    end

    print(string.format("Vx step %d/%d: %+.1f m/s", step, #STEPS, STEPS[step]))
    send_velocity(ID, TEAM, STEPS[step], 0, 0)
    step = step + 1
    wait = WAIT
end
```

## 3.4 Procedimiento de Test (Vx)

```
1. Fijar robot al banco permitiendo solo movimiento en X
2. stm32monitor:
   ```bash
   stm32monitor native write pid_vx_enable 0
   ```
3. Live watch: `v_vel[0]`, `vx_enc`, `vx_est`
4. Iniciar grabación:
   ```bash
   stm32monitor record --vars v_vel[0],vx_enc,vx_est --output vx_openloop.csv
   ```
5. Engine: Load Script > characterize_vx.lua
6. Analizar:
   python3 analysis/pid_tuner.py Log_*.csv -c v_vel[0] -m vx_enc --open-loop
```

## 3.5 Aplicar Ganancias Vx/Vy

Modificar en vivo mediante stm32monitor:

```bash
stm32monitor native write pid_vx.Kp <valor>
stm32monitor native write pid_vx.Ki <valor>
stm32monitor native write pid_vx.Kd 0.0

stm32monitor native write pid_vy.Kp <valor>
stm32monitor native write pid_vy.Ki <valor>
stm32monitor native write pid_vy.Kd 0.0
```

O desde la GUI de STM32Monitor:
```
pid_vx.params.Kp = <valor>
pid_vx.params.Ki = <valor>
...
```

Valores actuales: `pid_vx/vy: Kp=0.8, Ki=1.6, outputMax=1.5, outputMin=-1.5`

## 3.6 Verificación Closed-Loop

```bash
1. stm32monitor native write pid_vx_enable 1
2. Engine: Load Script > characterize_vx_closed.lua
3. Analizar:
   python3 analysis/pid_tuner.py Log_*.csv -c v_vel[0] -m vx_est --current Kp,Ki,Kd
```

**Target**: rise < 100ms, OS < 10%.

---

# Fase 4: Validación Multi-Sensor (Telemetría vs Cámara)

## 4.1 Captura Simultánea

```
1. Robot en cancha con cámara SSL activa
2. Engine grabando log CSV (ventana Record)
3. STM32Monitor grabando telemetría interna @ 1kHz
4. Ejecutar movimiento combinado desde Lua:
   - send_velocity(ID, TEAM, 1.0, 0.5, 2.0)  — vx + vy + vth simultáneo
   - O usar un script que barra varias velocidades
```

## 4.2 Comparación

```bash
# Comparar Vx: telemetría interna vs cámara
python3 analysis/sensor_compare.py telemetria.csv camara.csv --var vx

# Comparar todas las velocidades
python3 analysis/sensor_compare.py telemetria.csv camara.csv --var vx vy vth

# Exportar estadísticas
python3 analysis/sensor_compare.py telemetria.csv camara.csv --all --export validacion.json

# Comparación visual de logs antes/después de tuning
python3 analysis/csv_analyzer.py vth_v1.csv vth_v2.csv --compare
```

### Interpretación de Métricas

| Métrica | Bueno | Regular | Malo |
|---------|-------|---------|------|
| RMS error | < 0.1 m/s | 0.1–0.3 | > 0.3 |
| Bias (mean_error) | < 0.05 | 0.05–0.15 | > 0.15 |
| Correlación | > 0.95 | 0.8–0.95 | < 0.8 |
| Delay | < 30ms | 30–80ms | > 80ms |

---

# Fase 5: Movimiento Combinado (Validación Final)

## 5.1 Script Lua: characterize_mixed.lua

**Crear archivo**: `engine/engine/lua/characterize_mixed.lua`

```lua
-- characterize_mixed.lua
-- Mixed motion: sweeps through vx+vy+vth combinations
-- For final validation with all PIDs enabled

local ID = 0
local TEAM = 0

local MOTIONS = {
    {vx=1.0, vy=0.0, vth=0.0},   -- solo X
    {vx=0.0, vy=1.0, vth=0.0},   -- solo Y
    {vx=0.0, vy=0.0, vth=2.0},   -- solo rotación
    {vx=1.0, vy=1.0, vth=0.0},   -- diagonal
    {vx=1.0, vy=0.0, vth=1.0},   -- X + rotación
    {vx=0.0, vy=1.0, vth=1.0},   -- Y + rotación
    {vx=1.0, vy=1.0, vth=2.0},   -- todo combinado
}

local WAIT = 80  -- ~1.33s
local step = 1
local wait = 0
local running = false

function process()
    if not running then
        running = true
        print("=== Mixed Motion Characterization ===")
        print(string.format("Robot ID: %d, %d motion patterns", ID, #MOTIONS))
        print("All PIDs enabled. Start CSV + Engine recording.")
    end

    if step > #MOTIONS then
        print("=== Mixed Motion Complete! ===")
        send_velocity(ID, TEAM, 0, 0, 0)
        running = false
        return
    end

    if wait > 0 then
        wait = wait - 1
        local m = MOTIONS[step]
        send_velocity(ID, TEAM, m.vx, m.vy, m.vth)
        return
    end

    local m = MOTIONS[step]
    print(string.format("Motion %d/%d: vx=%+.1f vy=%+.1f vth=%+.1f",
        step, #MOTIONS, m.vx, m.vy, m.vth))
    send_velocity(ID, TEAM, m.vx, m.vy, m.vth)
    step = step + 1
    wait = WAIT
end
```

---

# Resumen de Comandos

## stm32monitor

```bash
# Live-watch motor speeds
stm32monitor live motor[0].measSpeed motor[1].measSpeed motor[2].measSpeed motor[3].measSpeed

# Enable/disable test mode
stm32monitor native write test_enable 1
stm32monitor native write test_motor 0
stm32monitor native write test_voltage 500

# Per-axis PID control
stm32monitor native write pid_vth_enable 0  # disable angular PID (open-loop)
stm32monitor native write pid_vx_enable 0   # disable Vx PID
stm32monitor native write pid_vy_enable 0   # disable Vy PID

# Live gain tuning
stm32monitor native write pid_vth.Kp 5.0
stm32monitor native write pid_vx.Kp 0.8
stm32monitor native write wheel_pid_params[0].Kp 25.0

# Record step response to CSV
stm32monitor record --vars motor[0].measSpeed,motor[0].pid.output --output motor0_step.csv
```

## Análisis

```bash
# Motor open-loop FOPDT
python3 analysis/pid_tuner.py Log_*.csv --cmd test_voltage -m "motor[0].measSpeed" --open-loop

# Motor closed-loop
python3 analysis/pid_tuner.py Log_*.csv -c speed -m "motor[0].measSpeed" --current Kp,Ki,Kd

# Vth open-loop
python3 analysis/pid_tuner.py Log_*.csv -c v_vel[2] -m vth_enc --open-loop

# Vth closed-loop
python3 analysis/pid_tuner.py Log_*.csv -c v_vel[2] -m vth_est --current Kp,Ki,Kd

# Vx open-loop
python3 analysis/pid_tuner.py Log_*.csv -c v_vel[0] -m vx_enc --open-loop

# Vx closed-loop
python3 analysis/pid_tuner.py Log_*.csv -c v_vel[0] -m vx_est --current Kp,Ki,Kd

# CSV analyzer (métricas generales)
python3 analysis/csv_analyzer.py Log_*.csv -c v_vel[2] -a

# Comparar dos CSVs
python3 analysis/csv_analyzer.py antes.csv despues.csv --compare

# Validación sensor vs cámara
python3 analysis/sensor_compare.py telemetria.csv camara.csv --var vx vy vth
```

## Exportar Resultados

```bash
# Exportar tuning a JSON
python3 analysis/pid_tuner.py Log_*.csv -c v_vel[2] -m vth_est --current 5.0,2.0,0.0 -e vth_tune.json

# Exportar métricas de step
python3 analysis/csv_analyzer.py Log_*.csv -c v_vel[2] -a -e vth_metrics.json -n

# Exportar comparación de sensores
python3 analysis/sensor_compare.py telemetria.csv camara.csv --var vx vy vth -e validacion.json
```

---

# Referencias

| Recurso | Descripción |
|---------|-------------|
| `docs/motor-characterization.md` | Arquitectura e implementación del modo test en firmware |
| `docs/hardware.md` | Pines, conexiones, especificaciones de motores y sensores |
| `analysis/pid_tuner.py --help` | Tool de tuning PID (FOPDT + reglas clásicas) |
| `analysis/csv_analyzer.py --help` | Tool de análisis de step response |
| `analysis/sensor_compare.py --help` | Tool de validación multi-sensor |
| `engine/engine/lua/characterize.lua` | Script de caracterización de motores |
| `engine/engine/src/lua_interface/api/control.rs` | API Lua (send_velocity, test_motor_step, etc.) |
| `Core/Src/drive_task.c` | Firmware: control loop, test mode, PIDs |
| `Core/Src/system_globals.c` | Variables globales y defaults |

## Flujo de Calibración Completo

```
Motor PID (Inner)
  ├─ characterize.lua (open-loop steps)
  ├─ pid_tuner.py --open-loop → FOPDT + gains
  ├─ Cargar wheel_pid_params[i] en STM32Monitor
  └─ Verificar closed-loop: rise < 30ms

Vth PID (Outer)
  ├─ stm32monitor native write pid_vth_enable 0
  ├─ characterize_vth.lua (open-loop omega steps)
  ├─ pid_tuner.py --open-loop → FOPDT + gains
  ├─ stm32monitor native write pid_vth.Kp <val> / pid_vth.Ki <val>
  ├─ stm32monitor native write pid_vth_enable 1
  └─ characterize_vth_closed.lua + pid_tuner.py --current

Vx PID (Outer)
  ├─ stm32monitor native write pid_vx_enable 0
  ├─ characterize_vx.lua (open-loop vx steps)
  ├─ pid_tuner.py --open-loop → FOPDT + gains
  ├─ stm32monitor native write pid_vx.Kp <val> / pid_vx.Ki <val>
  ├─ stm32monitor native write pid_vx_enable 1
  └─ characterize_vx_closed.lua + pid_tuner.py --current

Vy PID (Outer)
  └─ (same as Vx but with characterize_vy.lua)

Validación
  ├─ characterize_mixed.lua (vx+vy+vth combinado)
  ├─ sensor_compare.py (telemetría vs cámara)
  └─ csv_analyzer.py --compare (antes vs después)
```
