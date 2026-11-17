# Resumen Ejecutivo del Proyecto

## SysmicProyect — RoboCup SSL

Sistema completo de control para robots de la RoboCup Small Size League, desarrollado por **Sysmic Robotics**.

---

## Arquitectura en 3 Capas

| Capa | Plataforma | Lenguaje | Archivos | Líneas aprox. |
|---|---|---|---|---|
| **Robot Firmware** | STM32F767ZI | C / FreeRTOS | `robot/` | ~3,500 |
| **Base Station** | STM32F103 | C | `basestation/` | ~500 |
| **Engine** | PC (Linux/Windows) | Rust / Iced | `engine/` | ~5,000 |
| **Estrategia** | Lua 5.4 | Lua | `lua/` | ~500 |

**Total estimado**: ~9,500 líneas de código fuente

---

## Componentes Principales

### Robot (STM32F767ZI)
- **4 tareas FreeRTOS activas**: drive, radio, kick, accel (ball_detector comentado)
- **4 motores DC** con encoder PID (1 kHz) + robot-level PID (200 Hz)
- **Sensores**: MPU6050 (acelerómetro + giroscopio por I2C3 @200Hz), VL6180X (ToF, tarea comentada)
- **Radio**: nRF24L01+ a 2.4 GHz, canal 0x6B
- **Kicker**: carga capacitiva 4s, disparo por solenoide

### Base Station (STM32F1)
- Puente USB-Serial ↔ nRF24L01+ (bidireccional)
- 2 módulos nRF24: TX + RX independientes

### PC Engine (Rust)
- **GUI**: Iced 0.14 con visualización de cancha en tiempo real
- **Visión**: SSL Vision multicast UDP + EKF tracking
- **Path Planning**: Fast planner con evasión de obstáculos (left/right-hand rule)
- **Motion Control**: BangBang time-optimal, PID, LookAhead PID
- **Estrategia**: Lua 5.4 con API completa (control + estado + GUI overlay)
- **Simulación**: interfaz con grSim via UDP
- **Telemetría**: Recepción de estado EKF del robot vía basestation, visualización en charts panel (EkfX/Y/Theta/Vx/Vy/Omega, GyroZ, BallRange)

### Protocolo de Comunicación
- **Downlink**: 30 bytes (5 bytes × 6 robots) → USB → nRF24 → robots
- **Uplink**: 32 bytes de telemetría (estado EKF: x, y, vx, vy, θ, ω + gyro_z + ball + kicker + sat)
- **Codificación**: 10-bit signed con signo + 7 bits + 2/3/4 MSB bits (vx/vy/omega (vth): /100)
- **Formato**: señales escaladas × 100 (vx/vy/omega (vth)), clamped a ±511

---

## Últimas Modificaciones

| Archivo | Fecha | Cambio |
|---|---|---|
| `Tools/trajectory.h/c` | 27-Ago 2026 | Nuevo controlador de posición P con saturación: Trajectory_ComputeVelocity() genera vx/vy/vth en marco cuerpo hacia waypoint (deadbands TRAJ_POS_DEADBAND=0.01m, TRAJ_HEAD_DEADBAND=0.02rad, ganancias TRAJ_KP_LINEAR=2.0, TRAJ_KP_ANGULAR=4.0) |
| `Core/Src/drive_task.c` | 27-Ago 2026 | Nuevo processCameraCorrection(): valida header 0xFE + robot_id, lee x/y/theta/age (f32 LE), aprende offset en reposo, aplica EKF_UpdateEngineDelayed (B2 corrección de cámara) |
| `Core/Src/radio_task.c` | 27-Ago 2026 | Pipe 1 configurada con correction_node_addr ("sysrc"), payload 18 bytes, AA off; pipe==PIPE1 → processCameraCorrection() (B2) |
| `Core/Src/system_globals.c/h` | 27-Ago 2026 | Añadidos correction_node_addr={'s','y','s','r','c'} y camera_correction_enabled=0 (B2) |
| `basestation/Src/main.c` | 27-Ago 2026 | Multiplexación del canal de corrección: si ready_data[0]==0xFE → SetAddr PIPETX correction_node_addr, TxPacket 18, restaurar rx_node_addr (B2) |
| `engine/.../sender/radio.rs` | 27-Ago 2026 | Nuevos send_camera_correction() (paquete 18 bytes 0xFE) y active_robot_ids() (B2) |
| `engine/.../tracker.rs` | 27-Ago 2026 | Nuevo get_last_raw_vision(team,id) → (x_cam,y_cam,theta_cam,capture_instant) (B2) |
| `engine/main.rs` | 27-Ago 2026 | Tras send_commands, envía correcciones de cámara a robots activos vía tracker + send_camera_correction (B2) |
| `docs/communication-protocol.md` | 27-Ago 2026 | Sección Corrección de Cámara (18 bytes, Pipe 1 dedicada "sysrc", header 0xFE) |
| `docs/robot-firmware.md` | 27-Ago 2026 | processCameraCorrection(), Pipe 1 radio, trajectory.c/h, camera_correction_enabled |
| `docs/base-station.md` | 27-Ago 2026 | Canal de Corrección de Cámara (Pipe 1, 18 bytes), multiplexación 0xFE |
| `docs/engine.md` | 27-Ago 2026 | send_camera_correction(), active_robot_ids(), get_last_raw_vision(), flujo de corrección |
| `docs/architecture.md` | 27-Ago 2026 | Canal de corrección en diagrama de radio protocol (Pipe 1 "sysrc", 18 bytes) |
| `engine/src/environment.rs` | 23-May 19:49 | Exclusión por ID de robot en vez de equipo, radio colisión ball 0.1 |
| `engine/src/vision.rs` | 23-May 19:48 | Refactor: config tracker, PPS, geometría, RobotUpdateData |
| `Core/Src/system_init.c` | 10-Jun 2026 | SPI1 FRXTH+SSI+CRCL fix, I2C timing unificado 0x1FF01428, DIP Switch PULLDOWN, UART5 LSB-First, TIM7 start, SPI2 Mode 3 |
| `Core/Src/stm32f7xx_hal_msp.c` | 10-Jun 2026 | I2C4 clock source PCLK1, I2C3/I2C4 PULLUP |
| `Tools/encoder.h/c` | 10-Jun 2026 | Acumulador 32-bit + overflow handling (reemplaza AngleNormalize) |
| `Tools/nrf24.c` | 10-Jun 2026 | Address width off-by-one corregido (addr_width - 3), do-while loop |
| `Tools/vl6180x.c` | 10-Jun 2026 | Write16 buffer bug, ReadLux división, timeouts 100-200ms |
| `Tools/motor.c` | 10-Jun 2026 | LowPassFilter_Update (reemplaza LowPassIIR_Update), brake nota active-LOW |
| `Core/Src/main.c` | 10-Jun 2026 | Error_Handler: LED blink triple + NVIC_SystemReset |
| `Core/Src/drive_task.c` | 10-Jun 2026 | Encoder init con accumulator/lastRaw, (void) params en processRadioCommand |
| `Core/Src/radio_task.c` | 10-Jun 2026 | Timeout 10ms en nRF24_TxPacket y loops modo TX/RX, comentario osDelay corregido |
| `Core/Inc/system_globals.h` / `Core/Src/system_globals.c` | 10-Jun 2026 | Eliminados who_am_i_debug, ret_code_debug |
| `docs/hardware.md` | 10-Jun 2026 | Pin mapping corregido contra MSP, motor-TIM mapping actualizado |
| `docs/robot-firmware.md` | 10-Jun 2026 | Pin mapping actualizado, HSE 8MHz, encoder accumulator |
| `docs/architecture.md` | 10-Jun 2026 | MSB-First → LSB-First |
| `docs/communication-protocol.md` | 10-Jun 2026 | MSB-First → LSB-First |
| `Core/Src/main.c` | 09-Jul 2026 | Migración CMSISv1→v2: osThreadDef→osThreadNew+osThreadAttr_t, osMessageQ→osMessageQueue, creación explícita de radioTask/kickTask, TIM7→vTaskNotifyGiveFromISR(driveTaskHandle), cola kick con osMessageQueueNew |
| `Core/Src/accel_task.c` | 09-Jul 2026 | CMSISv1→v2: osKernelSysTick→osKernelGetTickCount, osDelayUntil v1→v2 |
| `Core/Src/drive_task.c` | 09-Jul 2026 | CMSISv2: osMutexWait→osMutexAcquire, osMessagePut→osMessageQueuePut |
| `Core/Src/radio_task.c` | 09-Jul 2026 | CMSISv2: void const*→void*, osMessageGet→osMessageQueueGet |
| `Core/Src/kick_task.c` | 09-Jul 2026 | CMSISv2: void const*→void*, osMessageGet→osMessageQueueGet, osMutexWait→osMutexAcquire |
| `Core/Inc/system_globals.h` | 09-Jul 2026 | CMSISv2: tipos actualizados (osThreadId_t, osMessageQueueId_t, osMutexId_t) |
| `Core/Src/system_globals.c` | 09-Jul 2026 | Eliminados HAL handles duplicados con main.c CubeMX (linker error fix) |
| `Core/Src/system_init.c` | 09-Jul 2026 | Refactor completo: eliminados MX_*_Init duplicados (ahora static en main.c), solo MX_Init_Custom() con overrides (I2C timing 0x1FF01428, SPI DataSize 8-bit, FRXTH/SSI/CRCL/SPE, encoder filters 0→8, SPI2 force-reset+NSSPMode) |
| `Core/Src/main.c` | 09-Jul 2026 | Error_Handler restaurado (LED blink triple + NVIC_SystemReset desde backup), Motor_Init DISABLE, cinemática trigonométrica restaurada (sin/cos WHEEL_ANGLE + ROBOT_RADIO) |
| `Core/Inc/main.h` | 09-Jul 2026 | Agregado `#include \"system_globals.h\"` en USER CODE Includes
| `Tools/motor.c` | 11-Jul 2026 | Clamped voltaje a [0, 4095] antes de DAC write (Bug #17) |
| `Core/Src/drive_task.c` | 11-Jul 2026 | Rate limiter reset en timeout radio (#18), PID integral por error (#21), theta rate limiter post-PID (#22), processRadioCommand params eliminados (#25) |
| `Tools/ekf.c` | 11-Jul 2026 | P matrix con temps (#19), sin/cos renormalizado (#20) |
| `Core/Src/kick_task.c` | 11-Jul 2026 | osMessageQueueGet timeout 5000ms (#23) |
| `engine/.../logger.rs` | 11-Jul 2026 | BufWriter flush cada 60 frames (#24), timestamp con milisegundos (#29) |
| `engine/.../tracker.rs` | 11-Jul 2026 | Eliminado set_noise_parameters() per-frame (#28) |
| `Tools/PID.c` + `PID.h` | 11-Jul 2026 | Eliminado PID_UpdateController dead code (#26) |
| `Core/Inc/system_globals.h` | 11-Jul 2026 | #include movido debajo del header guard (#27) |
| `Tools/nrf24.c` | 11-Jul 2026 | EN_AA cambiado de 0x3F a 0x01, solo pipe 0 (#30) |
| `Core/Src/kick_task.c` | 13-Jul 2026 | Pre-carga capacitor al inicio, helpers charge_capacitor/fire_kicker (#31) |
| `Core/Src/drive_task.c` | 13-Jul 2026 | kick_sel=0 post-disparo, odometría/measSpeed / WHEEL_GEAR_RATIO, deadband escalado (#31, #32) |
| `Tools/motor.h` | 13-Jul 2026 | WHEEL_GEAR_RATIO=3.0f, SPEED_CNT_RATIO usa gear ratio (#32) |
| `Core/Src/radio_task.c` | 13-Jul 2026 | Telemetría: estado EKF completo (x/y/vx/vy/θ/ω) reemplaza wheel speeds (#32) |
| `engine/gui/types.rs` | 15-Jul 2026 | Agregado TelemetryUpdate struct + telemetry_rx en GuiChannels |
| `engine/gui/panels/charts.rs` | 15-Jul 2026 | 8 PlotVariable EKF (EkfX/Y/Theta/Vx/Vy/Omega, GyroZ, BallRange) + DataSample fields + CSV |
| `engine/gui/mod.rs` | 15-Jul 2026 | telemetry_rx drain en Tick handler → ChartsPanel samples |
| `engine/main.rs` | 15-Jul 2026 | Canal telemetry_gui_tx/rx (256 cap), send tras read_telemetry() |
| `basestation/Src/main.c` | 15-Jul 2026 | Dual-nRF24: RX telemetry con addr rx_node_addr, SPI PRESCALER_32, timeout TxPacket |
| `basestation/Src/stm32f1xx_hal_msp.c` | 15-Jul 2026 | AFIO remap USART1 PB6/PB7 (fallback para pin quemado) |

---

## Cómo compilar

### Robot Firmware
```
# Usando STM32CubeIDE
cd robot/
# Abrir con STM32CubeIDE y compilar
```

### Base Station
```
cd basestation/
make  # o compilar con STM32CubeIDE
```

### PC Engine
```bash
cd engine/
cargo build --release
cargo run -- lua/main.lua           # con estrategia
cargo run -- lua/main.lua --sim     # con grSim
```

### Dependencias del Engine
- Rust 2021 edition
- Protobuf compiler (para build.rs)
- Librerías del sistema: `libudev-dev` (Linux, para serialport)

---

## Tests

```bash
cd engine/
cargo test                           # unit tests (PID, tracker, types, packet_serializer)
cargo test -- --ignored              # integration tests si los hay
```

---

## Archivos Clave por Subsistema

### Robot
| Archivo | Rol |
|---|---|
| `Core/Src/main.c` | Entry point, init periféricos (CubeMX) + MX_Init_Custom() overrides, crear tareas (osThreadNew) |
| `Core/Src/system_init.c` | MX_Init_Custom(): overrides post-CubeMX (I2C timing, SPI config, encoder filters) |
| `Core/Src/drive_task.c` | Control PID motores (1kHz wheel / 200Hz robot), odometría, cinemática |
| `Core/Src/radio_task.c` | nRF24 comunicación, telemetría |
| `Core/Src/kick_task.c` | Carga y disparo del kicker |
| `Core/Src/accel_task.c` | MPU6050 lectura (aceleración + giroscopio @200Hz) |
| `Core/Inc/system_globals.h` | Constantes y variables globales (CMSIS-RTOS2 types) |
| `Core/Src/system_globals.c` | Definiciones globales no-CubeMX |
| `Tools/motor.h/c` | Driver de motor: enable, direction, brake, PID, LowPassFilter |
| `Tools/odometry.h/c` | Forward kinematics matrix |
| `Tools/ekf.h/c` | Extended Kalman Filter (7 estados: x,y,sinθ,cosθ,vx,vy,ω) |
| `Tools/nrf24.h/c` | nRF24L01+ driver SPI |
| `Tools/MPU6050.h/c` | MPU6050 accel+gyro driver I2C |
| `Tools/vl6180x.h/c` | VL6180X ToF driver |

### Engine (`engine/`)
| Archivo | Rol |
|---|---|
| `src/main.rs` | Entry point, Iced + tokio engine loop |
| `src/types.rs` | Core types: Vec2D, RobotState, BallState, commands |
| `src/world.rs` | World state management |
| `src/vision.rs` | SSL Vision multicast receiver |
| `src/tracker.rs` | EKF Kalman filter (7-state) |
| `src/environment.rs` | Collision detection environment |
| `src/motion/mod.rs` | High-level motion API |
| `src/motion/controllers/bangbang/mod.rs` | Time-optimal trajectory |
| `src/motion/controllers/pid/pid.rs` | PID controller |
| `src/motion/controllers/pid/lookahead.rs` | LookAhead path following PID |
| `src/motion/pathplanning/mod.rs` | Fast path planner |
| `src/radio.rs` | Command dispatcher (serial + grSim) |
| `src/packet_serializer.rs` | Binary packet encoding |
| `src/grsim.rs` | grSim UDP interface |
| `src/lua_interface/` | Lua 5.4 VM + API bindings |
| `src/gui/` | Iced GUI dashboard |

### Estrategia (`lua/`)
| Archivo | Rol |
|---|---|
| `main.lua` | Formation movement demo |
| `skills/move.lua` | Basic movement skill |
| `skills/go_to_ball.lua` | Ball approach with dribbler |
| `skills/kick.lua` | Kick skill |
| `skills/kick_to_point.lua` | Position + kick to point |
| `skills/move_ball.lua` | Dribble ball to target |
| `tactics/active/receive_pass.lua` | Receive pass positioning |
| `tactics/non_active/position_for_pass.lua` | Pass reception scoring |
| `utils/utils.lua` | Ball possession detection |
| `api_globals.lua` | Lua API type definitions (IDE autocomplete) |
