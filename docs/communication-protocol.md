# Protocolo de Comunicación

## Visión General

```
PC ──USB──→ Base Station ──nRF24──→ Robot
PC ←──USB── Base Station ←──nRF24── Robot
PC ──UDP──→ grSim (simulador, opcional)
```

## Radio nRF24L01+

| Parámetro | Valor | Descripción |
|---|---|---|
| Canal RF (comandos) | 0x6B (107) | BS → Robot (TX pipe) |
| Canal RF (telemetría) | 0x6C (108) | Robot → BS (RX pipe1) |
| Dirección TX Pipe | "systx"+robot_id (5 bytes) | BS tx → robot rx |
| Dirección RX Pipe | "systx" (5 bytes) | robot tx → BS rx (pipe1) |
| Dirección Pipe 1 (corrección) | "sysrc" (5 bytes) | BS → robot, corrección de cámara |
| Tasa de datos | 1 Mbps (default) | |
| Potencia TX | 0 dBm (default) | |
| CRC | 1 byte | |
| ARQ | Deshabilitado en TX pipe | |
| Payload RX (Pipe 0) | 30 bytes | Comandos (5 robots × 6 bytes) |
| Payload RX (Pipe 1) | 18 bytes | Corrección de cámara (1 robot) |
| Payload TX | 32 bytes | Telemetría por robot |

## Arquitectura Dual Channel

El robot tiene un solo NRF24L01+ conectado por SPI1, operando en modo half-duplex. Las funciones de RX (comandos) y TX (telemetría) comparten el mismo módulo y se protegen con `radioMutex`.

### Flujo en el robot

| Función | Canal | Operación | Mutex |
|---|---|---|---|
| `RadioFunction` | 0x6B | RX comandos del BS | Adquiere `radioMutex`, recibe, libera |
| `TelemetryFunction` | 0x6C | TX telemetría al BS | Adquiere `radioMutex`, cambia canal → TX → cambia a 0x6B → libera |

`TelemetryFunction` ejecuta a `TELEMETRY_PERIOD_MS` (10 ms, 100 Hz) con un delay de stagger basado en `robot_id` para evitar colisiones entre robots:

```
stagger_delay = robot_id × (TELEMETRY_PERIOD_MS / NUM_ROBOTS)
```

Cada ciclo de telemetría: adquiere mutex → cambia a canal 0x6C → envía payload → cambia a canal 0x6B → libera mutex.

### Flujo en la Base Station

La BS usa dos dispositivos nRF24 dedicados:

- **`tx_device`**: Transmite comandos en canal 0x6B a `"systx"+robot_id`.
- **`rx_device`**: Recibe telemetría en canal 0x6C por pipe1 con dirección `"systx"`.

## Comando PC → Robot (30 bytes, 5 bytes × 6 robots)

### Byte 0: Control
| Bits | Campo | Descripción |
|---|---|---|
| [7:5] | ID | Robot ID (0-5) |
| [4:2] | Dribbler | Velocidad del dribbler (0-7) |
| [1] | Kick | 1 = disparar kicker |
| [0] | — | Reservado (sin uso) |

### Bytes 1-4: Velocidades (10-bit signed, / 100 para vx/vy/omega)
| Byte | Bits | Campo |
|---|---|---|
| 1 | [7] = sign, [6:0] = abs(vx)[6:0] | vx LSB |
| 2 | [7] = sign, [6:0] = abs(vy)[6:0] | vy LSB |
| 3 | [7] = sign, [6:0] = abs(vth)[6:0] | omega LSB (velocidad angular, rad/s) |
| 4 | [7:6] = abs(vx)[8:7], [5:4] = abs(vy)[8:7], [3:0] = abs(vth)[9:7] | MSB bits |

Decodificación en firmware (`processRadioCommand` en `drive_task.c`):
```c
v_vel[0] = (sign_vx ? -1 : 1) * abs(vx) / 100.0      // vx en m/s
v_vel[1] = (sign_vy ? -1 : 1) * abs(vy) / 100.0      // vy en m/s
v_vel[2] = (sign_vth ? -1 : 1) * abs(vth) / 100.0  // omega en rad/s
```

**Nota**: vx/vy y vth (omega) usan divisor 100 (resolución ±5.11 m/s y ±5.11 rad/s respectivamente).

### Decodificación de Kick y Dribbler
```c
dribbler_sel = (buffer[0] & 0x1C) >> 2;  // bits [4:2]
kick_sel     = buffer[0] & 0x02 ? 1 : 0; // bit [1]
```

## Telemetría Robot → PC (32 bytes)

| Offset | Tamaño | Campo | Tipo | Descripción |
|---|---|---|---|---|
| 0 | 1 | ID(3) + ball(1) + kicker_charged(1) + motor_saturated(4) | uint8 | Bits [7:5]=ID, [0]=ball, [1]=kicker, [5:2]=sat[0-3] |
| 1-2 | 2 | ball_range | uint16 LE | Distancia bola en mm |
| 3-6 | 4 | x_est | float32 | Posición X del EKF (m) |
| 7-10 | 4 | y_est | float32 | Posición Y del EKF (m) |
| 11-14 | 4 | vx_est | float32 | Velocidad lineal X del EKF (m/s) |
| 15-18 | 4 | vy_est | float32 | Velocidad lineal Y del EKF (m/s) |
| 19-22 | 4 | theta_est | float32 | Heading del EKF, atan2(sinθ, cosθ) (rad) |
| 23-26 | 4 | omega_est | float32 | Velocidad angular del EKF (rad/s) |
| 27-30 | 4 | gyro_z | float32 | Gyro Z crudo calibrado (rad/s, diagnóstico) |
| 31 | 1 | ang_acc_limit × 10 | uint8 | ROBOT_MAX_ANGULAR_ACC × 10 (resolución 0.1 rad/s²) |

### Decodificación en firmware (`packTelemetryBuffer` en `radio_task.c`):
```c
buffer[0] = (robot_id << 5) | (ball_bit << 0) | (kicker_bit << 1) | (sat_bit << 2);
memcpy(&buffer[3],  &ekf.x[0], sizeof(float));  // x_est
memcpy(&buffer[7],  &ekf.x[1], sizeof(float));  // y_est
memcpy(&buffer[11], &ekf.x[4], sizeof(float));  // vx_est
memcpy(&buffer[15], &ekf.x[5], sizeof(float));  // vy_est
memcpy(&buffer[19], &theta_est, sizeof(float));  // theta_est (global)
memcpy(&buffer[23], &ekf.x[6], sizeof(float));  // omega_est
memcpy(&buffer[27], &gyro_z_rads, sizeof(float)); // gyro_z (diagnóstico)
```

### Decodificación en engine (`parse_telemetry` en `radio.rs`):
```rust
let x_est     = f32::from_le_bytes([buf[3], buf[4], buf[5], buf[6]]);
let y_est     = f32::from_le_bytes([buf[7], buf[8], buf[9], buf[10]]);
let vx_est    = f32::from_le_bytes([buf[11], buf[12], buf[13], buf[14]]);
let vy_est    = f32::from_le_bytes([buf[15], buf[16], buf[17], buf[18]]);
let theta_est = f32::from_le_bytes([buf[19], buf[20], buf[21], buf[22]]);
let omega_est = f32::from_le_bytes([buf[23], buf[24], buf[25], buf[26]]);
```

**Nota sobre estado EKF**: La telemetría envía el estado completo del EKF (posición + velocidad + orientación + velocidad angular) en lugar de velocidades individuales de rueda. Esto permite al engine comparar directamente con la visión del campo (SSL Vision) y fusionar datos de cámara + robot.

**Nota sobre gyro_z**: Se envía como campo de diagnóstico. El engine puede usarlo para validar la estimación del EKF o detectar divergencia.

## Configuración de Unicast por Robot

**BS → Robot (comandos, canal 0x6B)**: La BS usa su `tx_device` con dirección `"systx"` + robot_id (5° byte = ID). El robot recibe en canal 0x6B con la misma dirección.

**Robot → BS (telemetría, canal 0x6C)**: El robot transmite en canal 0x6C con dirección `"systx"` + robot_id. La BS recibe en su `rx_device` en pipe1 con dirección `"systx"`.

Los 30 bytes de comando contienen datos para todos los robots (5 bytes c/u).
Cada robot solo procesa el bloque correspondiente a su ID:

```c
processRadioCommand(nrf_device.rx_data + 5 * robot_id);
```

## Corrección de Cámara (18 bytes, Pipe 1 dedicada)

El engine envía correcciones de cámara a cada robot activo para que su EKF local corrija el drift contra el marco global de visión. Este tráfico viaja por una **pipe dedicada** (Pipe 1) con dirección `"sysrc"`, distinta de la pipe de comandos (Pipe 0, `"sysrx"`), para que el firmware distinga ambos tipos de paquete por pipe.

### Flujo

```
Engine (radio.rs) ──18 bytes──→ Base Station ──nRF24 Pipe 1 ("sysrc")──→ Robot
```

1. El engine, tras `send_commands()`, itera los robots activos y llama `send_camera_correction()` con la última visión cruda del tracker (`get_last_raw_vision()`).
2. La Base Station recibe los 18 bytes por USB. Si `ready_data[0] == 0xFE`, cambia su `tx_device` a la dirección de corrección `"sysrc"`, envía los 18 bytes, y restaura la dirección de comandos `"sysrx"`.
3. El robot recibe el paquete por **Pipe 1** (configurada a 18 bytes con dirección `"sysrc"`) y lo procesa con `processCameraCorrection()`.

### Layout del paquete (18 bytes)

| Offset | Tamaño | Campo | Tipo | Descripción |
|---|---|---|---|---|
| 0 | 1 | Header | uint8 | `0xFE` (distingue del broadcast de 30 bytes) |
| 1 | 1 | Robot ID | uint8 | ID del robot destino |
| 2-5 | 4 | x_cam | float32 LE | Posición X de la cámara (marco global, m) |
| 6-9 | 4 | y_cam | float32 LE | Posición Y de la cámara (marco global, m) |
| 10-13 | 4 | theta_cam | float32 LE | Orientación de la cámara (marco global, rad) |
| 14-17 | 4 | age | float32 LE | Segundos desde la captura (no timestamp absoluto) |

**Nota sobre `age`**: Se envía la edad del dato en lugar de un timestamp absoluto porque la PC y el STM32 tienen relojes independientes. El robot busca la entrada de historia en `robot_time_now - age`.

### Generación en engine (`send_camera_correction` en `radio.rs`)

```rust
let age = capture_instant.elapsed().as_secs_f64();
let mut pkt = Vec::with_capacity(18);
pkt.push(0xFE);
pkt.push(id as u8);
pkt.extend_from_slice(&(x_cam as f32).to_le_bytes());
pkt.extend_from_slice(&(y_cam as f32).to_le_bytes());
pkt.extend_from_slice(&(theta_cam as f32).to_le_bytes());
pkt.extend_from_slice(&(age as f32).to_le_bytes());
```

### Procesamiento en firmware (`processCameraCorrection` en `drive_task.c`)

```c
if (buffer[0] != 0xFE || buffer[1] != robot_id)
    return;
// ... lee x_cam, y_cam, theta_cam, age (f32 LE) ...
if (!camera_correction_enabled)
    return;
// Aprende offset local<->global en reposo, aplica EKF_UpdateEngineDelayed
```

- El firmware valida el header `0xFE` y el `robot_id`.
- Solo aplica la corrección si `camera_correction_enabled == 1` (deshabilitado por defecto).
- Mientras el robot está en reposo (`|vx| < 0.05`, `|vy| < 0.05`), aprende el offset de alineación local↔global (`offset_x/y/theta`).
- Aplica la corrección en el marco local vía `EKF_UpdateEngineDelayed(&ekf, x_local, y_local, theta_local, t_cam, ax, ay, gyro_z)`.

## Protocolo Serial (Base Station ↔ PC)

| Dirección | Idle | Baud rate | Formato |
|---|---|---|---|
| PC → Base Station | 30 bytes | 115200 | 8N1, LSB First |

| Base Station → PC | 32 bytes | 115200 | 8N1, LSB First |
## SSL Vision Protocol

- **Multicast**: 224.5.23.2:10020
- **Formato**: protobuf SSL_WrapperPacket
- **Campos**: detection (robots_blue, robots_yellow, balls) + geometry (field_lines, field_arcs)
- **Coordenadas**: mm en protobuf, convertidas a metros (/1000) en engine

## SSL Game Controller

- **Multicast**: 224.5.23.1:10003
- **Formato**: protobuf SSL_Referee
- **Comandos**: HALT, STOP, NORMAL_START, FORCE_START, PREPARE_KICKOFF_*, PREPARE_PENALTY_*, etc.

## grSim Protocol

- **UDP**: 127.0.0.1:20011
- **Formato**: protobuf grSim_Packet
- **Comandos**: robot commands (velocity, kick), teleport robot, teleport ball
