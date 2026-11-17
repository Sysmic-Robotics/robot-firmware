# Diagrama de Stack de Software

## Capas de Software

```mermaid
graph BT
    subgraph LAYER5["Capa 5: Estrategia (Lua)"]
        L1["skills/<br/>move.lua, kick.lua<br/>go_to_ball.lua<br/>move_ball.lua"]
        L2["tactics/<br/>shoot.lua, pass.lua<br/>receive_pass.lua<br/>position_for_pass.lua"]
        L3["play_engine/<br/>play_engine.lua"]
        L4["main.lua<br/>(entry point)"]
    end

    subgraph LAYER4["Capa 4: Motion Control (Rust)"]
        M1["motion/mod.rs<br/>face_to, move_to<br/>motion, move_direct"]
        M2["controllers/bangbang/<br/>Trajectory1D + 2D<br/>Time-optimal control"]
        M3["controllers/pid/<br/>PID + LookAhead PID"]
        M4["pathplanning/<br/>FastPathPlanner<br/>Obstacle avoidance"]
    end

    subgraph LAYER3["Capa 3: State & Comms (Rust)"]
        S1["world.rs<br/>Robot + Ball State"]
        S2["vision.rs<br/>SSL Vision Receiver"]
        S3["tracker.rs<br/>EKF Kalman Filter"]
        S4["game_controller.rs<br/>Referee Receiver"]
        S5["radio.rs<br/>Command Dispatcher"]
        S6["packet_serializer.rs<br/>Binary Encoding"]
        S7["grsim.rs<br/>Simulator Interface"]
    end

    subgraph LAYER2["Capa 2: Base Station (C/STM32F1)"]
        B1["UART RX (115200)<br/>← PC"]
        B2["nRF24L01+ TX<br/>→ Robot"]
        B3["nRF24L01+ RX<br/>← Robot"]
        B4["UART TX<br/>→ PC"]
    end

    subgraph LAYER1["Capa 1: Robot Firmware (C/STM32F7)"]
        R1["radio_task.c<br/>nRF24 driver"]
        R2["drive_task.c<br/>Control de motores<br/>PID + Odometría"]
        R3["kick_task.c<br/>Carga/disparo"]
        R4["accel_task.c<br/>MPU6050 @200Hz"]
        R5["ball_detector_task.c<br/>VL6180X @50Hz"]
    end

    L1 --- L4
    L2 --- L4
    L3 --- L4
    LAYER5 --> LAYER4
    LAYER4 --> LAYER3
    LAYER3 --> LAYER2
    LAYER2 --> LAYER1
```

## Dependencias de Librerías (Rust Engine)

```mermaid
graph TD
    ENGINE["engine (Rust)"] --> TOKIO["tokio (async runtime)"]
    ENGINE --> ICED["iced 0.14 (GUI)"]
    ENGINE --> MLUA["mlua 0.10 (Lua 5.4)"]
    ENGINE --> PROTOBUF["protobuf 3.7.2"]
    ENGINE --> NALGEBRA["nalgebra 0.33 (EKF)"]
    ENGINE --> SERIALPORT["serialport 4"]
    ENGINE --> SOCKET2["socket2 0.5"]
    ENGINE --> SERDE["serde / serde_json"]
    ENGINE --> CSV["csv 1.3"]
    ENGINE --> TRACING["tracing / tracing-subscriber"]
```

## Tareas FreeRTOS (Robot Firmware)

| Tarea | Prioridad | Stack (bytes) | Frecuencia | Propósito |
|---|---|---|---|---|
| `driveTask` | AboveNormal | 1024 | 1kHz wheel / 200Hz robot | Control de motores, PID, odometría, EKF |
| `accelTask` | Normal | 2048 | 200 Hz (5ms) | Lectura MPU6050 (acelerómetro + giroscopio) |
| `radioTask` | Normal | 2048 | Por evento | Comunicación nRF24L01+, telemetría |
| `kickTask` | Low | 2048 | 4s carga + disparo | Control del kicker |
| `ballDetectorTask` | Low | 128 | 50 Hz (comentada) | Sensor VL6180X ToF |
