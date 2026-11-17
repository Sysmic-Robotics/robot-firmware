# Diagrama de Arquitectura del Sistema

```mermaid
graph TB
    subgraph PC["PC — Engine (Rust)"]
        direction TB
        SSL["SSL Vision<br/>UDP 224.5.23.2:10020"] -->|"SSL_WrapperPacket<br/>(protobuf)"| VISION["vision.rs<br/>Multicast UDP Receiver"]
        GC["Game Controller<br/>UDP 224.5.23.1:10003"] -->|"Referee Message<br/>(protobuf)"| GC_RS["game_controller.rs<br/>Referee Receiver"]
        
        VISION --> TRACKER["tracker.rs<br/>EKF Kalman Filter<br/>[x,y,sinθ,cosθ,vx,vy,ω]"]
        TRACKER --> WORLD["world.rs<br/>World State<br/>(robots + ball)"]
        GC_RS --> WORLD
        
        LUA["lua_interface/<br/>Lua 5.4 VM"] -->|"get_state()"| WORLD
        LUA -->|"motion/kick commands"| RADIO["radio.rs<br/>Radio Dispatcher"]
        
        MOTION["motion/<br/>PID + BangBang +<br/>Path Planning"] -->|"MotionCommand"| RADIO
        
        WORLD --> GUI["gui/<br/>Iced GUI Dashboard"]
        LUA -->|"draw commands"| GUI
    end

    subgraph BASESTATION["Base Station — STM32F1"]
        direction TB
        UART["UART RX<br/>115200 baud"] --> NRF24_BS["nRF24L01+<br/>TX Mode"]
        NRF24_BS_RX["nRF24L01+<br/>RX Mode"] --> UART_TX["UART TX"]
    end

    subgraph ROBOT["Robot — STM32F767ZI"]
        direction TB
        NRF24_R["nRF24L01+<br/>RX/TX"] --> RADIO_TASK["radio_task.c<br/>nRF24 driver"]
        RADIO_TASK -->|"setSpeed()"| DRIVE_TASK["drive_task.c<br/>Motor Control"]
        
        DRIVE_TASK --> MOTOR1["Motor 1<br/>(TIM3 Encoder)"]
        DRIVE_TASK --> MOTOR2["Motor 2<br/>(TIM8 Encoder)"]
        DRIVE_TASK --> MOTOR3["Motor 3<br/>(TIM2 Encoder)"]
        DRIVE_TASK --> MOTOR4["Motor 4<br/>(TIM5 Encoder)"]
        
        DRIVE_TASK -->|"DAC I2C1"| DAC["MAX5815<br/>Motor Voltages"]
        DRIVE_TASK -->|"DAC I2C2"| DRIBBLER_DAC["MAX5814<br/>Dribbler"]
        
        KICK_TASK["kick_task.c<br/>Kicker Control"] --> KICKER["Kicker Solenoid<br/>(GPIO)"]
        
        ACCEL["accel_task.c<br/>MPU6050 (@200Hz)"] -->|"I2C3"| MPU["MPU6050<br/>Accel+Gyro"]
        BALL["ball_detector_task.c<br/>VL6180X (@50Hz)"] -->|"I2C3"| VL6180["VL6180X<br/>ToF Sensor"]
        
        DRIBBLER["Dribbler Motor"] --> DRIBBLER_DAC
        
        DRIVE_TASK -->|"kinematic<br/>mapping"| ODOM["Odometry Estimation"]
        DRIVE_TASK -->|"radio telemetry"| RADIO_TASK
    end

    PC -->|"USB Serial<br/>115200 baud"| BASESTATION
    BASESTATION -->|"2.4 GHz<br/>nRF24L01+"| ROBOT
    PC -->|"UDP 20011<br/>(opcional)"| grSim["grSim<br/>Simulator"]
```

## Flujo de Datos

1. **SSL Vision** envía posiciones de robots/pelota → `vision.rs` → `tracker.rs` (Kalman) → `world.rs`
2. **Game Controller** envía estado del partido → `game_controller.rs` → `world.rs`
3. **Lua AI** lee `world.rs`, ejecuta estrategia (skills/tactics), envía comandos → `radio.rs`
4. **Motion Control** (PID/BangBang) genera velocity commands → `radio.rs`
5. **Radio Dispatcher** serializa comandos → USB → Base Station → nRF24 → Robot
6. **Robot** ejecuta comandos: control PID de ruedas (1kHz), odometría, kicker, sensores
7. **Robot** envía telemetría (velocidades estimadas, ball possession) → nRF24 → Base Station → PC
