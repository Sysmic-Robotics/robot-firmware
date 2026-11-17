# Hardware — Mapeo de Pines y Conexiones

## STM32F767ZI — Robot

### GPIO — LEDs, DIP Switch y misc

| Pin | Dirección | Pull | Función |
|---|---|---|---|
| PI12 | Output | — | LED 1 |
| PI13 | Output | — | LED 2 |
| PI14 | Output | — | LED 3 |
| PJ1 | Input | NOPULL | DIP Switch bit 0 (ID) — pulldown externo en PCB |
| PJ0 | Input | NOPULL | DIP Switch bit 1 — pulldown externo en PCB |
| PI15 | Input | NOPULL | DIP Switch bit 2 — pulldown externo en PCB |
| PB2 | Input | NOPULL | DIP Switch bit 3 — pulldown externo en PCB |
| PC4 | Input | NOPULL | DIP Switch bit 4 — pulldown externo en PCB |
| PH4 | Input | NOPULL | DIP Switch bit 5 — pulldown externo en PCB |
| PB12 | Output | — | (sin uso) |

Los 6 bits del DIP switch se leen en `Board_GetID()` como ID del robot (0-63).

Nota: `Error_Handler()` hace parpadear los 3 LEDs (500K ciclos on/off/on) y luego ejecuta `NVIC_SystemReset()`.

### GPIO — Motores

| Pin | Dirección | Función |
|---|---|---|
| PA10 | Output | Motor 0 Enable |
| PA9 | Output | Motor 0 Direction |
| PA8 | Output | Motor 0 Brake |
| PC11 | Output | Motor 1 Enable |
| PC12 | Output | Motor 1 Direction |
| PD0 | Output | Motor 1 Brake |
| PK7 | Output | Motor 2 Enable |
| PG15 | Output | Motor 2 Direction |
| PK6 | Output | Motor 2 Brake |
| PF5 | Output | Motor 3 Enable |
| PF3 | Output | Motor 3 Direction |
| PF4 | Output | Motor 3 Brake |

### GPIO — Kicker

| Pin | Dirección | Función |
|---|---|---|
| PJ4 | Output | Kicker Charge (MOSFET gate) |
| PF11 | Output | Kicker Fire (solenoid pulse) |

### GPIO — Dribbler

| Pin | Dirección | Función |
|---|---|---|
| PJ8 | Output | Dribbler Enable |
| PJ7 | Output | Dribbler Direction |
| PJ6 | Output | Dribbler Brake |

### SPI1 — nRF24L01+ (Radio)

| Pin | Función | nRF24 |
|---|---|---|
| PA5 | SCK | SCK |
| PA6 | MISO | MISO |
| PA7 | MOSI | MOSI |
| PG10 | GPIO | CE (Chip Enable) |
| PG9 | GPIO | CSN (Chip Select) |

- Data size: 8-bit
- Mode 0 (CPOL=Low, CPHA=1Edge)
- Prescaler: /16 (~6.75 MHz a 216 MHz APB2)
- Interrupción IRQ no conectada (polling)
- FRXTH activado (RXNE se genera con 1 byte, no esperar 2)
- SSI=1 forzado (internal NSS HIGH), CRCL limpio, SPE activado tras init

### I2C1 — MAX5815 (DAC Motores)

| Pin | Función |
|---|---|
| PB6 | SCL |
| PB7 | SDA |

- Timing: `0x1FF01428` (~400 kHz, PRESC=1, SCLDEL=15, SDADEL=15, SCLH=20, SCLL=40)
- 2.0V reference voltage
- Canales A, B, C, D para 4 motores

### I2C2 — MAX5814 (DAC Dribbler)

| Pin | Función |
|---|---|
| PF1 | SCL |
| PF0 | SDA |

- Timing: `0x1FF01428` (~400 kHz)
- Canal A para dribbler motor

### I2C3 — MPU6050 (Accel+Gyro) y VL6180X (ToF)

| Pin | Función |
|---|---|
| PH7 | SCL |
| PH8 | SDA |

- Timing: `0x1FF01428` (~400 kHz)
- Pull-up activado en MSP (GPIO_PULLUP en PH7/PH8)
- MPU6050: acelerómetro + giroscopio (I2C, dirección 0x68)
- VL6180X: sensor de rango ToF (I2C, dirección 0x29)
- Ball detection threshold: 65 mm (código existe, tarea ball_detector comentada)
- Moving average: 10 samples

### I2C4 — (sin usar actualmente)

| Pin | Función |
|---|---|
| PF14 | SCL |
| PF15 | SDA |

- Timing: `0x1FF01428` (~400 kHz)
- Pull-up activado en MSP (GPIO_PULLUP en PF14/PF15)

### UART5 — Debug/Serial

| Pin | Función |
|---|---|
| PB13 | TX |
| PD2 | RX |

- 115200 baud, 8N1, LSB First (estándar)
- `UART_ADVFEATURE_NO_INIT` (MSB-First no forzado, se usa el reset default LSB)
- DMA TX (DMA1 Stream7, Channel 4)

### Timers — Encoders

| Timer | motor[] | Motor (enable/dir/brake) | Pins |
|---|---|---|---|
| TIM3 | `motor[0]` | PA10/PA9/PA8 | Ch1: PC6, Ch2: PC7 |
| TIM8 | `motor[1]` | PC11/PC12/PD0 | Ch1: PI5, Ch2: PI6 |
| TIM2 | `motor[2]` | PK7/PG15/PK6 | Ch1: PA0, Ch2: PA1 |
| TIM5 | `motor[3]` | PF5/PF3/PF4 | Ch1: PH10, Ch2: PH11 |

Modo encoder TI12, prescaler 0, period 65535.

Nota: Los encoders son de 8192 CPR (cuentas por revolución) con multiplicación 4x del modo TI12, dando 32768 cuentas/revolución.

Overflow del timer 16-bit manejado con acumulador `int32_t` en `encoder.c`: se calcula la diferencia `int16_t` entre la lectura actual y la anterior, sumándola al acumulador. Esto preserva el sentido de giro incluso cuando el contador hardware da la vuelta de 65535→0 o 0→65535.

### Timers — Control

| Timer | Propósito | Configuración |
|---|---|---|
| TIM7 | Time base 1 kHz para control loop (drive_task) | Prescaler 108-1, Period 1000-1, IRQn priority 5, `HAL_TIM_Base_Start_IT` |

---

## Base Station — STM32F1

| Periférico | Pines | Conectado a |
|---|---|---|
| UART1 | PA9 (TX), PA10 (RX) | USB-Serial (PC), 115200 baud |
| SPI1 | PA5 (SCK), PA6 (MISO), PA7 (MOSI) | nRF24L01+ TX |
| SPI2 | PB13 (SCK), PB14 (MISO), PB15 (MOSI) | nRF24L01+ RX |
| GPIO | Varios | CE/CSN para módulos nRF24 |

---

## Sensores

| Sensor | Tipo | Bus | Rango | Frecuencia |
|---|---|---|---|---|
| MPU6050 | Acelerómetro 3-axis + Giroscopio 3-axis | I2C3 | ±2g / ±250°/s | 200 Hz |
| VL6180X | Time-of-Flight | I2C3 | 0-200 mm | (tarea comentada) |
| Encoders | Quadrature (4x) | TIM2/3/5/8 | 8192 CPR | 1 kHz |

## Actuadores

| Actuador | Control | Especificación |
|---|---|---|
| Motor DC × 4 | MAX581x DAC (I2C1) + driver H-bridge | 15 RPS max, 5240 RPM nominal |
| Dribbler | MAX581x DAC (I2C2) + driver | 7 niveles de velocidad |
| Kicker solenoid | MOSFET (GPIO) | 4s carga, 10ms pulso de disparo |

---

> **⚠️ Nota sobre pines:** Esta documentación refleja la configuración real del firmware (extraída de `stm32f7xx_hal_msp.c` y `system_init.c`). Si el esquemático de Altium difiere, el firmware necesita ser actualizado para coincidir con el hardware real, o viceversa.
