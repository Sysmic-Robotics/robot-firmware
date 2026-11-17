# Firmware del Robot (STM32F767ZI)

## Platform

- **MCU**: STM32F767ZI (Cortex-M7, 216 MHz)
- **RTOS**: FreeRTOS (CMSIS-RTOS2 v2, API CMSIS-RTOS2)
- **Toolchain**: STM32CubeIDE / HAL + LL
- **Clock**: HSE 8 MHz → PLL (PLLM=4, PLLN=216, PLLP=/2) → 216 MHz
- **Estructura**: Código generado por CubeMX con secciones USER CODE para regeneración segura

### Arquitectura de Inicialización

El proyecto está estructurado para coexistir con CubeMX:

- **`Core/Src/main.c`** (CubeMX): Contiene las funciones `MX_*_Init()` estáticas para cada periférico. Todo el código personalizado está dentro de secciones `USER CODE BEGIN/END` — seguro ante regeneración.
- **`Core/Src/system_init.c`** (personalizado): Define `MX_Init_Custom()` que se llama después de `MX_*_Init()` en `main()` para hacer overrides: timings I2C forzados a `0x1FF01428`, SPI DataSize 8-bit, registros FRXTH/SSI/CRCL/SPE, filtros encoder, SPI2 force-reset y NSSPMode.
- **`Core/Src/system_globals.c/h`**: Variables globales compartidas entre tareas (handles no-CubeMX, colas, flags), con tipos CMSIS-RTOS2.
- Las tareas FreeRTOS se crean con `osThreadNew()` + `osThreadAttr_t` (CMSIS-RTOS2 API).

## Tasks FreeRTOS

Actualmente se crean **5 tareas** en `main.c` mediante `osThreadNew()`. La tarea `ball_detector` está comentada (código existe pero no se ejecuta).

### drive_task.c — Control de Motores
**Prioridad**: osPriorityAboveNormal | **Stack**: 256 bytes | **Loop**: 1 kHz (wheel + velocity) / 200 Hz (position + EKF)
**Timing**: TIM7 genera interrupción a 1 kHz → `HAL_TIM_PeriodElapsedCallback` → `vTaskNotifyGiveFromISR(driveTaskHandle)` → despierta drive_task

**Control en cascada desacoplado por tasa** (27-Ago-2026):
- **Motor PID @1 kHz**: Encoder_Update → measSpeed (LowPass α=0.15) → wheel_speeds_rads → refSpeed → feedforward + PID_CloseLoop → voltaje DAC
- **Velocity PID @1 kHz** (encoder-based): ForwardKinematics(wheel_speeds_rads → vx_enc/vy_enc/vth_enc) a 1 kHz → PID_CloseLoop(pid_vx/vy/vth, ref, enc) → vx_cmd/vy_cmd/vth_cmd → cinemática inversa → speed[i]. Usa encoders (no EKF) para eliminar la limitación del MPU6050 a 200 Hz en el control de velocidad.
- **Position loop @200 Hz** (EKF-based): EKF_Predict + EKF_UpdateEncoders + EKF_UpdateEngineDelayed (corrección cámara) → Trajectory_ComputeVelocity(ekf.x/y/θ → waypoint) → escribe la **referencia** vx_cmd_ref/vy_cmd_ref/vth_cmd_ref (consumida por el velocity PID a 1 kHz). Rate limiters lineales a 200 Hz sobre la referencia.

**Contrato de datos entre tasas**: el loop de 200 Hz (position) escribe `vx_cmd_ref`/`vy_cmd_ref`/`vth_cmd_ref`; el loop de 1 kHz (velocity PID) las consume y produce `vx_cmd`/`vy_cmd`/`vth_cmd`. Separación de bandwidth: motor 1 kHz → velocity 1 kHz → position 200 Hz (5x).

- Inicializa DAC (MAX581x en I2C1 para motores, MAX581x en I2C2 para dribbler, ambos 2.0V ref)
- Configura 4 motores con pines enable/direction/brake + timers encoder
- Wheel PID: Kp=50.0, Ki=5.0, Kd=0.0, salida ±4095, integral max 4095, sampleTime=1ms
- Robot PID (velocity, 1 kHz): Vx/Vy (Kp=2.5, Ki=9.2, DISABLED), Vth/omega (Kp=0.8, Ki=40.0, ENABLED). El PID de posición theta (pid_theta) fue eliminado; la posición theta la cierra el engine (heading hold)
- Rate limiters internos: vx/vy por ROBOT_MAX_LINEAR_ACC (200 Hz sobre la referencia), vth por ROBOT_MAX_ANGULAR_ACC (50 rad/s², 1 kHz sobre el comando)
- Low-pass filter en encoder: α = 0.15
- Feed-forward voltage: 300
- Congelamiento de integral PID cuando |error de seguimiento| < 0.01
- Odometría: `measSpeed / WHEEL_GEAR_RATIO` para convertir motor rad/s → wheel rad/s
- Deadband: `WHEEL_MAX_SPEED_RAD * WHEEL_GEAR_RATIO * 0.005f` (escalado al espacio motor)
- EKF init deferred al primer ciclo; forward kinematics (1 kHz) → predict → update encoders → save history (200 Hz)
- Radio watchdog: coast motors tras 500ms sin RX, re-enable cuando llega señal (resetea refs y cmd a 0)

**Función `processRadioCommand()`**: Decodifica vx/vy/vth del paquete de radio (10-bit signed / 100 para vx, vy y vth/omega), limita velocidad lineal a 2.5 m/s y aceleración a 1.5 m/s². v_vel[2] = omega (rad/s) para el PID de velocidad angular (pid_vth). La posición theta la cierra el engine (heading hold); el PID de posición theta (pid_theta) fue eliminado del firmware. Solo recibe `buffer`.

**Función `processCameraCorrection()`**: Procesa el paquete de corrección de cámara del engine (18 bytes, Pipe 1). Valida el header `0xFE` y el `robot_id`, lee `x_cam`/`y_cam`/`theta_cam`/`age` (f32 LE). Solo aplica si `camera_correction_enabled == 1`. Mientras el robot está en reposo (`|vx| < 0.05`, `|vy| < 0.05`), aprende el offset de alineación local↔global (`offset_x/y/theta`). Aplica la corrección en el marco local vía `EKF_UpdateEngineDelayed(&ekf, x_local, y_local, theta_local, t_cam, ax, ay, gyro_z)` donde `t_cam = ekf.time - age`.

### radio_task.c — Comunicación nRF24L01+
**Prioridad**: osPriorityNormal | **Stack**: 512 bytes

- Inicializa nRF24 en modo RX, canal 0x6B, pipe 0 con dirección "sysrx"+ID
- Dirección TX: "systx"+ID (último byte = robot_id)
- Deshabilitado Auto-Acknowledge en TX pipe
- **Protegido por radioMutex** (compartido con TelemetryFunction)
- Procesa paquete cada vez que drive_task envía mensaje a nrf24CheckHandle queue
- Lee payload de 30 bytes (5 bytes × 6 robots), extrae comando para su ID
- Sección crítica para decodificar comando + dribbler + kick (taskENTER/EXIT_CRITICAL)
- Stagger TX por robot_id para evitar colisiones en el canal
- `Radio_SendPacket()` activa en el bucle principal tras cada RX exitoso
- Timeout de 10ms en `nRF24_TxPacket()` y loops de modo TX/RX para evitar cuelgues
- nRF24 address width corregido: SETUP_AW = addr_width - 3 (off-by-one fijo)
- **Pipe 1 (corrección de cámara)**: Configurada con dirección `correction_node_addr` (`"sysrc"`), payload de 18 bytes, AA off. Cuando llega un paquete por Pipe 1 (`pipe == nRF24_PIPE1`), llama `processCameraCorrection()` en lugar de `processRadioCommand()`. Pipe 0 sigue recibiendo los comandos de 30 bytes.

**`packTelemetryBuffer()`**: Pack de telemetría (32 bytes) con estado EKF
```
Byte 0:  [ID3 (bits 7:5)][sat4 (bits 5:2)][kicker (bit 1)][ball (bit 0)]
Bytes 1-2: ball_range (uint16 LE, mm)
Bytes 3-6: x_est (float32, m, EKF)
Bytes 7-10: y_est (float32, m, EKF)
Bytes 11-14: vx_est (float32, m/s, EKF)
Bytes 15-18: vy_est (float32, m/s, EKF)
Bytes 19-22: theta_est (float32, rad, EKF heading via atan2)
Bytes 23-26: omega_est (float32, rad/s, EKF angular velocity)
Bytes 27-30: gyro_z (float32, rad/s, calibrado, diagnóstico)
Byte 31: robot_max_angular_acc × 10 (uint8, resolución 0.1 rad/s²)
```

### telemetry_task.c — Telemetría Asíncrona (NUEVO)
**Prioridad**: osPriorityAboveNormal | **Stack**: 256 bytes | **Loop**: 100 Hz (10ms)

- **Nuevo**: Tarea dedicada para transmisión asíncrona de telemetría
- Ejecuta cada TELEMETRY_PERIOD_MS (10ms = 100 Hz)
- Delay de staggering: `osDelay(TELEMETRY_PERIOD_MS + robot_id)` para evitar colisiones entre robots
- Adquiere radioMutex antes de transmitir
- Llama `Radio_SendTelemetryPacket()` que:
  1. Cambia canal a TELEMETRY_CHANNEL (0x6C)
  2. Envia paquete de 32 bytes (packTelemetryBuffer)
  3. Espera TX_DS o timeout (10ms)
  4. Cambia canal de vuelta a nRF24L01_SYSMIC_CHANNEL (0x6B)
- Libera radioMutex después de transmitir
- Telemetría ahora es independiente de la recepción de comandos

### kick_task.c — Control del Kicker
**Prioridad**: osPriorityLow | **Stack**: 512 bytes

- Pre-carga capacitor al startup (antes del loop principal) para respuesta inmediata
- Helpers estáticos `charge_capacitor()` (GPIO HIGH 4s → LOW, marca KICKER_CHARGED) y `fire_kicker()` (GPIO pulse 10ms, marca KICKER_DISCHARGED)
- Loop: espera `kick_sel` activo → espera comando en kickQueueHandle (timeout 5000ms) → dispara → recarga → repite
- Timeout de 5000ms en `osMessageQueueGet` evita cuelgues si no llega comando
- Decodifica: dribbler speed index `getDribblerSpeedIndex()` (bits [4:2] del byte 0), kick trigger `getKickTriggerFlag()` (bit [1] del byte 0)

### accel_task.c — Acelerómetro MPU6050 (I2C3)
**Prioridad**: osPriorityNormal | **Stack**: 512 bytes | **Loop**: 5ms (200 Hz)

- Inicializa MPU6050 por I2C3
- Calibración: 100 muestras de bias en ejes X/Y
- Lee aceleración (g) y giroscopio (dps) en cada ciclo
- Convierte acceleración a m/s² con corrección de bias (clamp ±25g)
- Convierte giro Z a rad/s para el EKF (clamp ±10 rad/s)
- Almacena raw values (Accel_X_RAW, Gyro_X_RAW) y calibrados (accel_x_g, gyro_z_dps)

### ~~ball_detector_task.c — (COMENTADO en main.c)~~
Código existe pero la tarea no se crea. Usa VL6180X ToF por I2C3 con moving average de 10 muestras. Loop a ~5ms (200 Hz). Umbral: 65mm para ball_possession.

## Drivers de Hardware (robot/Tools/)

| Driver | Bus | Función |
|---|---|---|
| motor.h/c | — | Enable/direction/brake, open-loop y closed-loop drive |
| encoder.h/c | TIM3/8/2/5 | Quadrature encoder → rad/s, diff-based speed con jump detection |
| odometry.h/c | — | Forward kinematics (matriz pseudo-inversa) |
| PID.h/c | — | PID con anti-windup condicional, clamping |
| ekf.h/c | — | Extended Kalman Filter (7 estados: x,y,sinθ,cosθ,vx,vy,ω), predict+update encoders+engine |
| MAX581x.h/c | I2C1/I2C2 | DAC 12-bit (MAX5815, 0-4095) para voltaje de motores y dribbler |
| nrf24.h/c | SPI1 | nRF24L01+ register map, payload, FIFO |
| board.h/c | GPIO | LEDs (PI12/13/14), DIP switch ID (6 bits, NOPULL — pulldown externo en PCB) |
| MPU6050.h/c | I2C3 | MPU6050 accel+gyro (EN USO) |
| vl6180x.h/c | I2C3 | VL6180X ToF ranging sensor (timeouts agregados) |
| kalman_1d.h/c | — | Filtro Kalman 1D (NO USADO actualmente) |
| kalman_2d.h/c | — | Filtro Kalman 2D (NO USADO actualmente, reemplazado por EKF) |
| lowpass_iir.h/c | — | Filtro pasa-bajos IIR de 1° orden (LowPassFilter_*) |
| trajectory.h/c | — | Controlador de posición P con saturación: `Trajectory_ComputeVelocity()` genera comandos de velocidad en marco cuerpo (vx, vy, vth) hacia un waypoint, con deadbands de posición/orientación |

## Parámetros Físicos

| Parámetro | Valor | Descripción |
|---|---|---|
| WHEEL_ANGLE 1-4 | 60°, 130°, 230°, 300° | Ángulos de montaje de ruedas |
| WHEEL_RADIO | 0.02704 m | Radio de rueda |
| ROBOT_RADIO | 0.08215 m | Distancia centro → rueda |
| ROBOT_MAX_LINEAR_VEL | 2.5 m/s | Velocidad lineal máxima |
| ROBOT_MAX_LINEAR_ACC | 1.5 m/s² | Aceleración lineal máxima |
| ENCODER_CPR | 8192 (4.0f × 2048) | Pulsos por revolución del encoder (TI12, 4x) |
| WHEEL_GEAR_RATIO | 3.0f | Relación de reducción motor→rueda (1.0f = direct drive) |
| SPEED_CNT_RATIO | GEAR_RATIO / WHEEL_RADIO | Conversión m/s → motor rad/s (refSpeed = speed × RATIO) |
| WHEEL_MAX_SPEED_RPS | 15 RPS | Velocidad máxima de rueda |
| VL6180X_THRESHOLD | 65 mm | Umbral de detección de pelota |
| nRF24L01_SYSMIC_CHANNEL | 0x6B (107) | Canal de radio |
| TELEMETRY_CHANNEL | 0x6C (108) | Canal de telemetría |
| TELEMETRY_PERIOD_MS | 10 | Periodo de telemetría (ms) |
| ANGULAR_SPEED_FACTOR | 30.0 | Factor de escala para velocidad angular |
| RADIO_TIMEOUT_MS | 500 | Timeout watchdog de radio (ms) |
| correction_node_addr | "sysrc" | Dirección Pipe 1 para corrección de cámara (18 bytes) |
| camera_correction_enabled | 0 | 1 = aplicar corrección de cámara al EKF local (deshabilitado por defecto) |

## Mapeo de Pines (STM32F7)

| Pin | Función | Conectado a |
|---|---|---|
| PA5, PA6, PA7 | SPI1 (SCK, MISO, MOSI) | nRF24L01+ (FRXTH, SSI, 8-bit) |
| PG10 | GPIO Output | nRF24 CE |
| PG9 | GPIO Output | nRF24 CSN |
| PB6, PB7 | I2C1 (SCL, SDA) | MAX5815 DAC motores (400 kHz, 0x1FF01428) |
| PF1, PF0 | I2C2 (SCL, SDA) | MAX5814 DAC dribbler (400 kHz, 0x1FF01428) |
| PH7, PH8 | I2C3 (SCL, SDA) | VL6180X ToF + MPU6050 (400 kHz, PULLUP) |
| PF14, PF15 | I2C4 (SCL, SDA) | (sin usar, 400 kHz, PULLUP) |
| PI12/13/14 | GPIO Output | LEDs indicadores |
| PJ0/1, PI15, PB2, PC4, PH4 | GPIO Input NOPULL | DIP Switch ID (6 bits — pulldown externo en PCB) |
| PA8/9/10 | GPIO Output | Motor 0 brake/direction/enable (encoder: TIM3 PC6/PC7) |
| PC11/12, PD0 | GPIO Output | Motor 1 enable/direction, brake (encoder: TIM8 PI5/PI6) |
| PK7, PG15, PK6 | GPIO Output | Motor 2 enable/direction, brake (encoder: TIM2 PA0/PA1) |
| PF5/3/4 | GPIO Output | Motor 3 enable/direction, brake (encoder: TIM5 PH10/PH11) |
| PJ4 | GPIO Output | Kicker charge |
| PF11 | GPIO Output | Kicker fire |
| PJ8/7/6 | GPIO Output | Dribbler enable/direction/brake |
| PB13, PD2 | UART5 (TX, RX) | Debug serial (115200 baud, LSB-First, DMA TX) |
