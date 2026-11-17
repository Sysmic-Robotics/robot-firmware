# Engine (Rust/Iced)

## Descripción General

Motor de control principal que corre en la PC. Integra:
- Recepción de visión SSL (multicast UDP)
- Filtro Kalman extendido (EKF) para tracking
- Planificación de trayectorias con evasión de obstáculos
- Control PID y BangBang
- Estrategias de IA escritas en Lua
- Interfaz gráfica Iced
- Envío de comandos por radio serial o grSim

## Stack Tecnológico

| Librería | Versión | Uso |
|---|---|---|
| tokio | 1.x | Runtime asíncrono |
| iced | 0.14 | GUI multiplataforma |
| mlua | 0.10 | Lua 5.4 VM (vendored) |
| protobuf | 3.7.2 | Parseo de paquetes SSL |
| nalgebra | 0.33 | Álgebra lineal (EKF) |
| serialport | 4.x | Comunicación serial |
| socket2 | 0.5 | Configuración multicast UDP |
| serde / serde_json | 1.x | Serialización |
| csv | 1.3 | Logging de datos |

## Módulos

### `main.rs` — Entry Point
- Inicializa logging (tracing)
- Crea canales de comunicación (tokio mpsc) entre GUI y engine
- Spawnea runtime tokio en thread separado
- Corre Iced daemon en thread principal

### `types.rs` — Core Data Types
- `Vec2D`: Vector 2D con operaciones (add, sub, mul, rotate, normalize)
- `RobotState`: id, team, position, velocity, orientation, angular_velocity, active
- `BallState`: position, velocity, is_moving()
- `MotionCommand`: id, team, vx, vy, angular
- `KickerCommand`: id, team, kick_x, kick_z, dribbler
- `RobotCommand`: id, team, motion + kicker
- `DrawCommand`: Point, Line, HighlightRobot

### `world.rs` — World State
- `HashMap<i32, RobotState>` para blue_robots y yellow_robots
- BallState: posición y velocidad
- `update_robot()`: actualiza posición/orientación/velocidad, marca active si update < 2s
- `set_commanded_velocity()`: registra comandos enviados
- `to_json()`: exporta estado completo para debugging

### `vision.rs` — SSL Vision Receiver
- Multicast UDP en 224.5.23.2:10020
- Reuse address + loopback habilitado
- Parsea SSL_WrapperPacket (protobuf)
- Extrae: detection (robots + ball) y geometry (field lines/arcs)
- Aplica EKF tracking por robot
- Envía VisionUpdate a GUI (posición, velocidad, PPS, geometría)
- Soporta configuración dinámica del tracker via VisionCommand

### `tracker.rs` — Extended Kalman Filter
- 7 estados: [x, y, sinθ, cosθ, vx, vy, ω]
- Filtro EKF completo: predict (jacobiano F) + update (jacobiano H)
- Normalización de ángulo en update
- `Tracker`: mantiene HashMap de EKF por (team, id)
- Parámetros configurables: process_noise_p, process_noise_v, measurement_noise
- Fallback: derivada simple si EKF deshabilitado
- `get_last_raw_vision(team, id)`: devuelve `Option<(f64, f64, f64, Instant)>` con la última visión cruda de cámara `(x_cam, y_cam, theta_cam, capture_instant)`. Lo consume el engine para el envío de correcciones de cámara (`EKF_UpdateEngineDelayed`).

### `game_controller.rs` — Referee Receiver
- Multicast UDP en 224.5.23.1:10003
- Parsea mensajes de referee (HALT, STOP, START, FORCE_START, etc.)

### `environment.rs` — Collision Detection
- Construye entorno desde world state excluyendo self_robot por ID
- Colisiones: bordes de cancha (3.0m × 2.15m), áreas de penal, robots (radio 0.2m), pelota (radio 0.1m)

### `motion/mod.rs` — High-Level Motion API
- `face_to()`: PID de orientación hacia punto target (por robot ID)
- `move_to()`: Path planning + bangbang con evasión de obstáculos
- `move_direct()`: BangBang directo sin path planning
- `motion()`: BangBang + corrección PID en velocidad (kp_x, ki_x, kp_y, ki_y)
- `bangbang_trajectory()`: Trayectoria explícita con límites configurables
- PIDs almacenados en HashMap<i32, PID> (un PID por robot)

### `motion/controllers/bangbang/mod.rs`
- Trajectory1D: Tiempo-óptimo con aceleración máxima en 1D
- Trajectory2D: Descomposición en X/Y con búsqueda binaria de α para sincronizar tiempos
- `compute_motion()`: Calcula velocidad next desde posición+velocidad actual
- `is_near_to_break()`: Simplificación de waypoints (frenado)

### `motion/controllers/pid/pid.rs`
- PID discreto con anti-windup (clamp integral)
- Límites configurables de integral y salida
- Reset de estado

### `motion/controllers/pid/lookahead.rs`
- LookAhead PID para seguimiento de trayectoria
- Búsqueda de closest waypoint con ventana local (±5, +40)
- Punto lookahead por distancia acumulada
- Lateral PID, longitudinal speed (slowdown al final), heading correction

### `motion/pathplanning/mod.rs`
- FastPathPlanner con evasión de obstáculos
- Evalúa ruta A (right-hand rule) y ruta B (left-hand rule), elige la más corta
- `search_subgoal()`: genera waypoints de evasión con rotación de 45°
- `simplify_path()`: elimina waypoints redundantes verificando colisión directa

### `radio.rs` — Command Dispatcher + Telemetry Receiver
- Soporta serial port (físico) y grSim (simulador)
- `add_motion_command()`: mergea comandos (suma vx/vy, override angular si ≠0)
- `add_kicker_command()`: acumula kick y dribbler
- `prepare_frame()`: prepara comandos para robots del frame anterior
- `send_commands()`: serializa y envía (serial.write_all o grsim.communicate_grsim)
- Merge permite combinar send_velocity + face_to en mismo ciclo
- `read_telemetry()`: lee paquetes de 32 bytes del serial (basestation → engine)
- `parse_telemetry()`: decodifica estado EKF del robot (x, y, vx, vy, theta, omega, gyro_z, ball, kicker)
- `send_camera_correction()`: envía corrección de cámara (18 bytes) a un robot por la pipe dedicada. Paquete: `[0]=0xFE`, `[1]=id`, `[2..6]=x_cam f32 LE`, `[6..10]=y_cam f32 LE`, `[10..14]=theta_cam f32 LE`, `[14..18]=age f32 LE` (segundos desde captura). Solo envía si `use_radio` está activo.
- `active_robot_ids()`: devuelve `Vec<(i32, i32)>` con los IDs de robots activos (del `HashSet<(i32,i32)>` interno). Se usa para iterar y enviar correcciones de cámara.

### `types.rs` — Core Data Types
- `Vec2D`: Vector 2D con operaciones (add, sub, mul, rotate, normalize)
- `RobotState`: id, team, position, velocity, orientation, angular_velocity, active
- `BallState`: position, velocity, is_moving()
- `MotionCommand`: id, team, vx, vy, angular
- `KickerCommand`: id, team, kick_x, kick_z, dribbler
- `RobotCommand`: id, team, motion + kicker
- `DrawCommand`: Point, Line, HighlightRobot
- `RobotTelemetry`: id, team, x_est, y_est, vx_est, vy_est, theta_est, omega_est, gyro_z, ball_possession, kicker_charged, motor_saturated, ang_acc_limit, ball_range
- `TelemetryUpdate`: robot_id, team, x_est, y_est, vx_est, vy_est, theta_est, omega_est, gyro_z, ball_possession, kicker_charged, ball_range

### `packet_serializer.rs` — Binary Protocol
- 5 bytes por robot:
  - Byte 0: ID(3) | dribbler(3) | kick(1) | callback(1)
  - Byte 1: sign_vx(1) | abs_vx(7)
  - Byte 2: sign_vy(1) | abs_vy(7)
  - Byte 3: sign_vth(1) | abs_vth(7)
  - Byte 4: msb_vx(2) | msb_vy(2) | msb_vth(4)
- vx/vy/vth: scaled ×100, clamped to [-511, 511]

### `grsim.rs` — Simulator Interface
- Envía comandos a grSim via UDP en puerto 20011
- Soporta: communicate_grsim(), teleport robot, teleport ball

### `lua_interface/` — Lua API
- Carga y ejecuta scripts Lua 5.4
- API de control: send_velocity, move_to, move_direct, motion, face_to, kickx, kickz, dribbler, reset_pids
- API de estado: get_robot_state, get_ball_state, get_blue_team_state, get_yellow_team_state, get_ref_message
- API de GUI: draw_point, draw_line, draw_text, highlight_robot
- API de simulación: grsim.teleport_robot, grsim.teleport_ball
- Los comandos de motion se mergean en radio.rs

### `gui/` — Iced GUI
- FieldCanvas: visualización de cancha, robots, pelota
- Toolbar, Sidebar, BottomPanel
- Paneles: control de visión, radio, grabación, charts, kalman
- Recibe VisionUpdate con posición/velocidad/PPS/geometría
- Draw commands desde Lua

### `gui/panels/charts.rs` — Telemetry Visualization
- **PlotVariable** con 17 variantes: Time, X, Y, θ, Vx, Vy, CmdVx, CmdVy, Cmdω, **EkfX, EkfY, EkfTheta, EkfVx, EkfVy, EkfOmega, GyroZ, BallRange**
- **DataSample**: campos vision (x, y, theta, vx, vy, cmd_vx, cmd_vy, cmd_omega) + campos EKF (ekf_x, ekf_y, ekf_theta, ekf_vx, ekf_vy, ekf_omega, gyro_z, ball_range)
- **MultiSeriesChartProgram**: canvas de Iced con gráficos en tiempo real, soporte multi-robot
- **CSV export**: header + rows con columnas vision + EKF
- **Telemetría data flow**: basestation UART → `radio.rs::read_telemetry()` → `main.rs` envía `TelemetryUpdate` → canal → `mod.rs` Tick drain → `charts.rs` push `DataSample`

## Engine Loop (~60 FPS)

```
while running:
  1. Process GUI commands (vision config, radio config, script load, etc.)
  2. Call Lua process()
  3. Collect draw commands from Lua
  4. Log frame (CSV recording)
  5. Prepare radio frame
  6. Send commands (serial or grSim)
  7. Send camera corrections to active robots (radio.rs::send_camera_correction)
  8. Read telemetry from basestation (32-byte EKF packets)
  9. Send TelemetryUpdate to GUI channel (charts)
  10. Sleep for remaining frame time (~16.667ms)
```

### Envío de Correcciones de Cámara

Tras `send_commands()`, el engine itera los robots activos y envía una corrección de cámara a cada uno con la última visión cruda del tracker:

```rust
let active = r.active_robot_ids();
if !active.is_empty() {
    let t = tracker.lock()...;
    for (id, team) in active {
        if let Some((x_cam, y_cam, theta_cam, capture_instant)) =
            t.get_last_raw_vision(team, id)
        {
            r.send_camera_correction(id, team, x_cam, y_cam, theta_cam, capture_instant);
        }
    }
}
```

Esto permite que el EKF local de cada robot corrija el drift contra el marco global de visión (ver `communication-protocol.md` para el layout del paquete de 18 bytes).

## Telemetry Integration (GUI Charts)

### Variables Disponibles

| Variable | Tipo | Origen | Descripción |
|----------|------|--------|-------------|
| `EkfX` | float | EKF firmware | Posición X (m) |
| `EkfY` | float | EKF firmware | Posición Y (m) |
| `EkfTheta` | float | EKF firmware | Heading, atan2(sinθ, cosθ) (rad) |
| `EkfVx` | float | EKF firmware | Velocidad lineal X (m/s) |
| `EkfVy` | float | EKF firmware | Velocidad lineal Y (m/s) |
| `EkfOmega` | float | EKF firmware | Velocidad angular (rad/s) |
| `GyroZ` | float | MPU6050 | Gyro Z crudo calibrado (rad/s) |
| `BallRange` | uint16 | VL6180X | Distancia bola en mm |

### Flujo de Datos

```
basestation UART (32 bytes)
  → radio.rs::read_telemetry() / parse_telemetry()
    → main.rs envía TelemetryUpdate al canal (256 cap)
      → mod.rs Tick handler drains canal
        → charts.rs push DataSample a ChartsPanel
          → MultiSeriesChartProgram renderiza en tiempo real
```

### Configuración de Canales

- Canal `telemetry_gui_tx/rx`: `mpsc::channel::<TelemetryUpdate>(256)`
- `GuiChannels.telemetry_rx`: `mpsc::Receiver<TelemetryUpdate>`
- `EngineApp.telemetry_rx`: `Arc<Mutex<mpsc::Receiver<TelemetryUpdate>>>`
- Drain en Tick handler: `while let Ok(tel) = telemetry_rx.try_recv() { ... }`

### Nota sobre Calidad de Señal

Los datos de telemetría pueden no ser continuos debido a:
- Latencia de radio half-duplex (TX + RX por turnos)
- Stagger entre robots (`osDelay(robot_id)`)
- Posibles colisiones en canal compartido
- Frecuencia de muestreo del EKF (200 Hz) vs frecuencia de envío (~60 Hz por robot)

El engine puede usar estos datos para:
1. Comparar con visión SSL (validar tracking)
2. Fusionar estimaciones (cámara + robot EKF)
3. Detectar divergencia entre sensores
4. Monitoreo en tiempo real de estado del robot
