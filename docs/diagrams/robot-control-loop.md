# Diagrama de Ciclo de Control del Robot

## Control en Cascada: Robot PID + Wheel PID

```mermaid
graph LR
    subgraph PC["PC Engine"]
        LUA["Lua AI"] -->|"vx_target<br/>vy_target<br/>vth_target"| RADIO_PC["radio.rs"]
    end

    subgraph ROBOT["STM32F7 Firmware"]
        RADIO_ROBOT["radio_task.c"] -->|"v_vel[0..2]"| ROBOT_PID["Robot-Level PID<br/>(200 Hz)"]
        
        subgraph WHEEL["Wheel-Level Control (1 kHz)"]
            IK["Inverse Kinematics<br/>speed[i] =  Σ(kinematic[i][j] * v_cmd[j])"] --> W0["PID Motor 0<br/>(TIM3 Encoder)"]
            IK --> W1["PID Motor 1<br/>(TIM8 Encoder)"]
            IK --> W2["PID Motor 2<br/>(TIM2 Encoder)"]
            IK --> W3["PID Motor 3<br/>(TIM5 Encoder)"]
            
            W0 -->|"DAC A"| M0["Motor 0<br/>(40°)"]
            W1 -->|"DAC B"| M1["Motor 1<br/>(140°)"]
            W2 -->|"DAC C"| M2["Motor 2<br/>(210°)"]
            W3 -->|"DAC D"| M3["Motor 3<br/>(330°)"]
        end
        
        M0 --> E0["Encoder TIM3"]
        M1 --> E1["Encoder TIM8"]
        M2 --> E2["Encoder TIM2"]
        M3 --> E3["Encoder TIM5"]
        
        E0 --> FK
        E1 --> FK
        E2 --> FK
        E3 --> FK
        
        FK["Forward Kinematics"] -->|"vx_est, vy_est, vth_est"| ROBOT_PID
        
        ROBOT_PID -->|"corrected<br/>vx/vy/vth"| IK
    end
```

## Diagrama de Estados del Kicker

```mermaid
stateDiagram-v2
    [*] --> CHARGING: charge GPIO ON
    CHARGING --> CHARGING: osDelay(4000ms)
    CHARGING --> CHARGED: kick_flag = CHARGED
    
    CHARGED --> FIRING: kick_sel && kick_flag == CHARGED
    FIRING --> FIRING: GPIO pulse (10ms)
    FIRING --> DISCHARGED: kick_flag = DISCHARGED
    
    DISCHARGED --> CHARGING: loop restart
    DISCHARGED --> [*]: kick_count++
```

## Configuraciones PID

| Controlador | Kp | Ki | Kd | Rango Salida | Integral Max |
|---|---|---|---|---|---|
| Wheel PID | 25.0 | 44.0 | 0.0 | ±4095 | 4095 |
| Robot Vx PID | 0.8 | 1.6 | 0.0 | ±1.5 | 0.5 |
| Robot Vy PID | 0.8 | 1.6 | 0.0 | ±1.5 | 0.5 |
| Robot Vth PID | 5.0 | 2.0 | 0.0 | ±5.0 | 2.0 |

## Filtro de Odometría

- Low-pass filter en encoder: `y += alpha * (x - y)` con alpha = 0.15
- Feed-forward voltage: 300
- Congelamiento de integral cuando |comando| < 0.01
- Radio watchdog: 500ms timeout → coast motors + disable PID

## Fusión Sensorial (Kalman)

| Filtro | Estado estimado | Entradas | Q | R |
|---|---|---|---|---|
| KalmanVelocity1D (vx) | Velocidad lineal X | aceleración (ax_ms2) + encoder (vx_enc) | 0.2 | 0.05 |
| KalmanVelocity1D (vy) | Velocidad lineal Y | aceleración (ay_ms2) + encoder (vy_enc) | 0.2 | 0.05 |
| KalmanAngular2D (vth) | Vel angular + bias giro | giroscopio (gyro_z_rads) + encoder (vth_enc) | Qw=1.0, Qb=0.001 | 0.5 |

## Matriz Cinemática

Ángulos de ruedas: 60°, 130°, 230°, 300° (radio de rueda: 0.02704m, radio del robot: 0.08215m)

**Inversa** (en `kinematic[4][3]` en firmware):
```
speed[i] = kinematic[i][0]*vx + kinematic[i][1]*vy + kinematic[i][2]*vth
```

**Directa** (en `odometry.h`, precomputada como pseudoinversa):
```
vx_est = WHEEL_RADIO * (FK_00*w0 + FK_01*w1 + FK_02*w2 + FK_03*w3)
vy_est = WHEEL_RADIO * (FK_10*w0 + FK_11*w1 + FK_12*w2 + FK_13*w3)
vth_est = WHEEL_RADIO * (FK_20*w0 + FK_21*w1 + FK_22*w2 + FK_23*w3)
```
