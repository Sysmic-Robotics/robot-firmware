# Arquitectura del Sistema

## Visión General

Sistema cliente-servidor de 3 capas para control de robots RoboCup SSL:

```
┌─────────────────────────────────────────────────────┐
│              PC Engine (Rust/Iced)                   │
│  ┌─────────┐ ┌──────────┐ ┌────────────────────┐   │
│  │ Vision  │→│  World   │←│  Lua Strategy AI   │   │
│  │ UDP Rx  │ │  State   │ │  (skills/tactics)  │   │
│  └─────────┘ └────┬─────┘ └──────────┬─────────┘   │
│                   │                  │              │
│                   v                  v              │
│  ┌─────────────────────────────────────────────┐   │
│  │          Radio Dispatcher                    │   │
│  │  (USB Serial or grSim UDP)                  │   │
│  └───────────────────┬─────────────────────────┘   │
└──────────────────────┼─────────────────────────────┘
                       │ USB Serial (115200 baud)
┌──────────────────────┼─────────────────────────────┐
│  Base Station (STM32F1)                            │
│  tx_device (0x6B) → Robot RX (Pipe 0, comandos)    │
│  tx_device (0x6B) → Robot RX (Pipe 1, corrección)  │
│  rx_device (0x6C) ← Robot TX (telemetría)          │
└──────────────────────┼─────────────────────────────┘
                       │ 2.4 GHz (nRF24L01+)
┌──────────────────────┼─────────────────────────────┐
│  Robot (STM32F767ZI)                                │
│  FreeRTOS Tasks (5 activas):                        │
│  ├─ radio_task (nRF24 RX, canal 0x6B)              │
│  ├─ telemetry_task (nRF24 TX, canal 0x6C)          │
│  ├─ drive_task (Motor PID @1kHz + Velocity PID @1kHz + Position/EKF @200Hz) │
│  ├─ kick_task (Kicker charge/fire)                  │
│  └─ accel_task (MPU6050 accel+gyro @200Hz)          │
│                                                     │
│  radioMutex: acceso mutuo al NRF24L01+             │
│  Tarea comentada (código existe):                   │
│  └─ ball_detector_task (VL6180X ToF)               │
└─────────────────────────────────────────────────────┘
```

## Principios de Diseño

1. **Control en cascada desacoplado por tasa**: Motor PID (1 kHz) → Velocity PID (1 kHz, encoders) → Position loop (200 Hz, EKF). Separación de bandwidth 5x entre velocity y position.
2. **Comunicación bidireccional dual-channel**: Canal 0x6B envía comandos (30 bytes → 6 robots × 5 bytes), canal 0x6C recibe telemetría (32 bytes)
3. **Estrategia en Lua**: Los scripts Lua se cargan en caliente sin reiniciar el engine
4. **Per-robot PID state**: Cada robot tiene su propio estado PID en el engine (HashMap por ID)
5. **Modularidad**: Sensores y actuadores como drivers separados, tareas RTOS independientes

## Componentes Clave

### Robot (STM32F767ZI)
- **MCU**: Cortex-M7 a 216 MHz
- **RTOS**: FreeRTOS con CMSIS-RTOS2
- **5 tareas activas**: drive, radio, telemetry, kick, accel
- **radioMutex**: acceso mutuo al NRF24L01+ entre radio_task y telemetry_task
- **4 motores DC** con encoder cuadratura (TIM2/3/5/8, CPR 8192)
- **Control de voltaje**: DAC MAX581x (I2C1 para motores, I2C2 para dribbler) — 12-bit, 2.0V ref
- **Radio**: nRF24L01+ (SPI1)
- **Sensor inercial**: MPU6050 (I2C3) — acelerómetro + giroscopio @200Hz
- **ToF**: VL6180X (I2C3) — detección de pelota (código existe, tarea comentada)
- **State estimation**: Extended Kalman Filter en firmware (7 estados: x, y, sinθ, cosθ, vx, vy, ω) + forward kinematics de encoders

### Base Station (STM32F1)
- **Propósito**: Puente transparente UART ↔ nRF24L01+
- **Baud rate**: 115200, 8N1, LSB First
- **2 módulos nRF24**: TX para enviar comandos, RX para recibir telemetría

### Engine (Rust)
- **Async runtime**: tokio
- **GUI**: Iced 0.14 (daemon multi-window)
- **Tracking**: Extended Kalman Filter (7 estados: x, y, sinθ, cosθ, vx, vy, ω)
- **Motion control**: PID con anti-windup, BangBang (trayectoria tiempo-óptimo), LookAhead PID
- **Path planning**: Fast planner con regla izquierda/derecha y simplificación de waypoints
- **Estrategia**: Lua 5.4 via mlua, con API completa de control y estado

## Radio Protocol

- **Canal comandos**: 0x6B (107) nRF24L01+ — BS TX → Robot RX (Pipe 0)
- **Canal telemetría**: 0x6C (108) nRF24L01+ — Robot TX → BS RX
- **Corrección de cámara**: Pipe 1 del robot (dirección `"sysrc"`, 18 bytes) — BS TX → Robot RX, multiplexada por la BS al detectar header `0xFE`
- **Direcciones**: TX = "systx" (robot envía a esta dirección), RX = "sysrx" (BS envía a esta dirección), Pipe 1 = "sysrc" (corrección de cámara)
- **Half-duplex**: Ambas direcciones comparten el mismo NRF24L01+ físico en el robot, protegido por radioMutex
- **Payload RX (Pipe 0)**: 30 bytes (5 bytes/robot × 6 robots)
- **Payload RX (Pipe 1)**: 18 bytes (corrección de cámara: header 0xFE, id, x/y/theta f32, age f32)
- **Payload TX**: 32 bytes (telemetría: ID, ball, kicker, motor_sat, EKF state x/y/vx/vy/θ/ω, gyro_z, angular acc limit)
- **Encoding**: 10-bit signed integers para vx/vy/vth (escala ÷100)
