# Phoenix Firmware — Estado Actual

> **PCB**: Phoenix (basado en moteus-c1, familia 2)
> **MCU**: STM32G431CBU6 @ 170 MHz (HSI + PLL)
> **Toolchain**: CubeMX + STM32CubeIDE (bare-metal C, HAL)
> **Fecha**: 2026-07-07

---

## 1. Periféricos Configurados (CubeMX)

| Periférico | Modo | Pines | Propósito |
|---|---|---|---|
| **TIM1** | Encoder TI12 (4x) | PA8 (CH1), PA9 (CH2) | Encoder incremental (quadrature) |
| **TIM2** | PWM 3-ch, center-aligned | PA0 (CH1), PA1 (CH2), PA2 (CH3) | PWM 3-fases motor, ARR=4250 → 20 kHz ✅ |
| **TIM6** | Basic timer, 20 kHz IRQ | (ninguno) | Timebase FOC loop |
| **ADC1** | Dual master, T2_TRGO trigger | PA3 (CH4), PB1 (CH12) | Corriente fase A y C |
| **ADC2** | Dual slave, T2_TRGO trigger | PA6 (CH3) | Corriente fase B |
| **SPI2** | Slave, 16-bit, full-duplex | PF1 (SCK), PB14 (MISO), PA11 (MOSI) | Comunicación placa master |
| **SPI3** | Master, 16-bit, 5.3 MBit/s | PC10 (SCK), PC11 (MISO), PB5 (MOSI) | DRV8323 gate driver |
| **I2C2** | Master Fast Mode | PC4 (SCL), PF0 (SDA) | AS5600 encoder absoluto |
| **CORDIC** | Hardware acelerador | (interno) | Trigonometría FOC (Park/InvPark) |
| **GPIO** | | PC13 (CS_DRV), PC15 (HiZ), PB13 (nFAULT EXTI), PC6 (nSLEEP), PB0, PB15 | Control DRV8323 + aux |

---

## 2. Drivers Implementados

### Tools/encoder.h / encoder.c — **Encoder incremental (TIM1)**
- ✅ Configuración parametrizable: CPR (1024), pole pairs (7), LPF alpha, update freq
- ✅ Modo TI12 (ambos bordes en ambos canales, 4x resolución → 4096 pulsos/rev)
- ✅ Manejo de overflow 16-bit con delta int16_t
- ✅ Acumulación de posición en int32_t (firmado, +-2^31 pulsos antes de wraparound)
- ✅ Velocidad estimada en rad/s con filtro LPF 1er orden (alpha configurable, default 0.85)
- ✅ Ángulo eléctrico en radianes [0, 2π)
- ✅ `Encoder_SetPosition()` para sincronizar con AS5600 al startup

### Tools/as5600.h / as5600.c — **Encoder absoluto (I2C2)**
- ✅ Driver completo: lectura de ángulo (0.5° resolución digital), registro de configuración (0x00-0x20)
- ✅ Status de campo magnético (MH, ML, MD) + AGC
- ✅ Inicializado como referencia absoluta al startup (sincroniza incremental en `SeedPosition()`)
- ⚠️ Lectura bloqueante por polling I2C (~1-5 ms por lectura en modo bloqueante)
- ⚠️ Configurado en modo 0.5° (12-bit), no se usa modo 0.0879° (16-bit) por limitaciones de ruido

### Tools/drv8323.h / drv8323.c — **Gate driver (SPI3)**
- ✅ Inicialización con configuración completa:
  - PWM_MODE = 3X (tres PWM independientes, complementarios generados por DRV8323)
  - DEAD_TIME = 50 ns (registro CTRL1)
  - CSA = 40 V/V (ganancia del amplificador de corriente)
  - OCP_RETRY = 4 ms, VDS_LVL = 0.6 V
- ✅ Lectura/escritura de registros SPI (16-bit, MSB primero)
- ✅ Status de fault con decodificación bitfield (FAULT1, FAULT2)
- ✅ Enable/disable con pin EN (PC14, inicializado LOW)
- ✅ Calibración CSA offset (`DRV8323_CalibrateCSA()`): CAL_A/B/C en secuencia con enable
- ✅ Control HiZ pin (PC15) para PowerOn/PowerOff de gates
- ✅ Función `DRV8323_ReadFaults()` periódica en main loop con auto-retry (hasta 3 reconexiones)
- ✅ Manejo de nFAULT por EXTI (PB13, falling-edge)

### Core/ — **Inicialización HAL + Aplicación**
- ✅ System clock: 170 MHz (HSI 16MHz → PLL /4 → 4 MHz → ×85 → 340 MHz VCO → /2 → 170 MHz)
- ✅ Secuencia de inicialización HAL:
  1. HAL_Init → SystemClock_Config
  2. MX_GPIO_Init       (pines + EXTI PB13)
  3. MX_ADC1_Init       (master, CH4+CH12)
  4. MX_ADC2_Init       (slave, CH3+CH3)
  5. MX_I2C2_Init       (AS5600)
  6. MX_SPI2_Init       (host comm, slave)
  7. MX_SPI3_Init       (DRV8323, master)
  8. MX_TIM1_Init       (encoder)
  9. MX_TIM2_Init       (PWM)
  10. MX_CORDIC_Init    (hardware math)
- ✅ Secuencia de aplicación:
  1. AS5600_Init → read absolute angle
  2. DRV8323_Init + DRV8323_CalibrateCSA → power on gates (HiZ=0)
  3. Encoder_Init → seed with absolute position
  4. FOC_Init → start TIM6 (20 kHz) + TIM2 PWM
  5. COMM_SPI_Init → enable SPI2 slave
  6. Main loop: fault monitoring + telemetry

---

## 3. Estado del Firmware por Componente

| Componente | Estado | Notas |
|---|---|---|
| **System Clock** | ✅ Listo | 170 MHz, Boost mode, 4 wait states flash |
| **TIM1 (encoder)** | ✅ Listo | Modo TI12 (4x), 16-bit con overflow handling, sin IRQ |
| **TIM2 (PWM)** | ✅ Corregido | PWM center-aligned 3-ch, ARR=4250 → 20 kHz |
| **TIM6 (FOC timer)** | ✅ Listo | Period=8499 → 20 kHz, IRQ preempt priority 0 |
| **CORDIC** | ✅ Listo | Inicializado, disponible para Park/InvPark |
| **ADC1 + ADC2 (corriente)** | ✅ Listo | Dual injected mode, T2_TRGO trigger, 47.5 cyc sampling, 3 fases (Ia, Ib, Ic) |
| **VBUS (DC bus)** | ⚠️ Hardcodeado | VBUS = 24.0V fijo en código. PA9 = TIM1_CH2 (encoder), no ADC. Si se necesita VBUS real, debe recibirse por SPI2 desde master o reasignar pin |
| **SPI3 (DRV8323)** | ✅ Listo | Config + CSA cal + fault polling + EXTI + auto-retry |
| **I2C2 (AS5600)** | ✅ Listo | Lectura ángulo absoluto + status campo magnético + AGC |
| **SPI2 (slave, host)** | ✅ Listo | Protocolo 5-palabras full-duplex, doble buffer TX/RX, IRQ-driven. Commands: STOP/SETPOINT/CALIBRATE/READ |
| **FOC loop** | ✅ Listo | TIM6 ISR @ 20 kHz, Clarke→Park→PI→InvPark→SVM min-max, LUT trig, ~8 µs/ciclo |
| **Control de corriente (PI)** | ✅ Listo | Id/Iq PI con feedforward (decoupling + BEMF). Id target = 0 |
| **Control de velocidad (PI)** | ✅ Listo | Cascade PI sobre lazo de corriente, anti-windup |
| **Control de posición (PID)** | ✅ Listo | Cascade PID sobre lazo de velocidad, anti-windup |
| **ServoControl state machine** | ✅ Listo | STOPPED→CALIBRATING→BRAKE→CURRENT/VELOCITY/POSITION/ZERO_VEL, manejo de FAULT con auto-retry |
| **PID primitives (moteus-port)** | ✅ Listo | Tools/pid.h — PID, SimplePI, ExponentialFilter, estructura Clamp |
| **Calibración motor** | ⚠️ Parcial | CALIBRATING state existe. Falta: calibración offset encoder, medición Rs/Ls, alineación, identificación de parámetros |
| **Protocolo comunicación SPI2** | ✅ Listo | 5 palabras (16-bit c/u = 80 bits por frame): [CMD] [ARG0-1] [CTR] [STS]. Comandos: STOP, SETPOINT (posición/vel/torque), CALIBRATE, READ telemetría. IRQ + callback `HAL_SPI_TxRxCpltCallback` → `COMM_SPI_ProcessFrame()` |
| **Telemetría** | ✅ Listo | Reporte periódico vía SPI2: posición, velocidad, corriente Id/Iq, duty cycles, fault flags, temperatura. Enviado en respuesta a cada frame SPI2 |
| **Protecciones** | ⚠️ Parcial | ✅ EXTI PB13 (nFAULT HW IRQ, priority 0) + polling DRV8323 status + auto-retry (3 intentos). Falta: overcurrent software, overtemperature, UVLO, stall detection, desaturation |

**Nota 1 — TIM2 ARR (CORREGIDO):** El auto-reload de TIM2 se actualizó de 0xFFFFFFFF a 4250. Con center-aligned mode, TIM2CLK=170 MHz y ARR=4250, la frecuencia PWM efectiva es f_pwm = 170 MHz / (2 × 4250) = 20 kHz. Coincide con el FOC rate de TIM6 (20 kHz). Actualización hecha desde CubeMX (`.ioc` → `TIM2.PeriodNoDither=4250`).

---

## 4. SPI2 Protocol — Especificación

### Frame Format (5 palabras × 16-bit, 80 bits total)

| Word | Nombre | Dirección | Descripción |
|---|---|---|---|
| 0 | CMD | Master → FOC | Código de comando |
| 1 | ARG0 | Master → FOC | Argumento 0 (ej: setpoint posición) |
| 2 | ARG1 | Master → FOC | Argumento 1 (ej: setpoint velocidad) |
| 3 | CTR | Master → FOC | Contador de secuencia (para detección de pérdida) |
| 4 | STS | FOC → Master | Status flags |

### Palabras de respuesta (FOC → Master, mismas 5 palabras devueltas):

| Word | Nombre | Contenido |
|---|---|---|
| 0 | CMD_ECHO | Eco del comando recibido |
| 1 | TELEM_0 | Posición (escala: 0.01 rad) |
| 2 | TELEM_1 | Velocidad (escala: 0.01 rad/s) |
| 3 | TELEM_2 | Corriente Iq (escala: 0.01 A) |
| 4 | STATUS | Fault flags + modo actual |

### Comandos Implementados

| CMD | Valor | Descripción | Estado |
|---|---|---|---|
| STOP | 0x0000 | Detener motor (transición a STOPPED) | ✅ |
| SETPOINT_POS | 0x0001 | Setpoint posición + velocidad máxima | ✅ |
| SETPOINT_VEL | 0x0002 | Setpoint velocidad | ✅ |
| SETPOINT_TORQUE | 0x0003 | Setpoint torque (corriente Iq) | ✅ |
| CALIBRATE | 0x0010 | Iniciar secuencia de calibración | ⚠️ Parcial |
| READ_STATUS | 0x0020 | Leer estado completo | ✅ |

---

## 5. FOC Loop Detail

El lazo FOC se ejecuta en la ISR de TIM6 a 20 kHz (Period=8499, 170 MHz / 8500 = 20 kHz).

### Timing
- Trigger: TIM6 update event → TIM6_DAC_IRQHandler
- Frecuencia: 20 kHz (cada 50 µs)
- Duración estimada: ~8 µs (160 ciclos @ 170 MHz)
- PWM update: duty cycles escritos en `__HAL_TIM_SET_COMPARE` al final de cada ISR

### Secuencia por ciclo
1. `GetCurrents()` — dispara ADC1 inyectado (dual con ADC2, trigger por TIM2_TRGO), lee:
   - `hadc1.InjectionData[0]` → Ia (ADC1_CH4 / PA3)
   - `hadc2.InjectionData[1]` → Ib (ADC2_CH3 / PA6)
   - `hadc1.InjectionData[2]` → Ic (ADC1_CH12 / PB1)
2. `adc_raw_to_amps()` — convierte raw ADC (12-bit) a amperios con ganancia DRV8323 CSA (40 V/V) + Rshunt
3. **Clarke transform** — Ia, Ib, Ic → Iα, Iβ
4. **Park transform** — Iα, Iβ → Id, Iq (usando sin/cos del ángulo eléctrico)
5. **PI current control**:
   - PI_Id: controla Id → 0 (flux weakening opcional)
   - PI_Iq: controla Iq → setpoint (del lazo de velocidad o torque)
   - Feedforward: decoupling ωe×L + BEMF
6. **Inverse Park** — Vd, Vq → Vα, Vβ
7. **SVM (Space Vector Modulation)** — implementación min-max:
   - Calcular duty cycles U, V, W desde Vα, Vβ
   - Normalizar al rango [0, 1]
8. `write_pwm_duties()` — escribe duty cycles en TIM2 CCR1-3

### Trigonometría
- Tabla LUT de 1024 entradas para sin/cos (en `trig_lut.h`)
- CORDIC hardware inicializado pero no usado actualmente (todo por LUT)
- Ángulo eléctrico: `encoder_position * pole_pairs % 2π`

### Filtros
- LPF exponencial en velocidad del encoder (alpha = 0.85)
- Promedio de 1 muestra ADC (sin filtro adicional)

---

## 6. ServoControl State Machine

```
                    ┌─────────────┐
                    │   STOPPED   │ ◄── Power-on reset
                    └──────┬──────┘
                           │ CMD_CALIBRATE
                    ┌──────v──────┐
                    │ CALIBRATING │
                    └──────┬──────┘
                           │ calib complete
                    ┌──────v──────┐
                    │    BRAKE    │
                    └──────┬──────┘
           ┌───────────────┼───────────────┐
           │               │               │
    ┌──────v──────┐ ┌──────v──────┐ ┌──────v──────┐
    │   CURRENT   │ │  VELOCITY   │ │  POSITION   │
    │ (torque ctrl)│ │ (velocity   │ │ (position   │
    │             │ │  control)   │ │  control)   │
    └─────────────┘ └─────────────┘ └─────────────┘
                           │
                    ┌──────v──────┐
                    │  ZERO_VEL   │
                    │ (hold vel=0)│
                    └─────────────┘

    Cualquier estado → FAULT (por nFAULT EXTI o watchdog)
    FAULT → STOPPED (después de retry exitoso)
```

### Modos de Control
- **CURRENT**: Lazo único de corriente (Iq target). Kp_scale = 0 → torque mode puro
- **VELOCITY**: Cascade velocidad → corriente (PI velocidad + PI corriente)
- **POSITION**: Cascade posición → velocidad → corriente (PID posición + PI velocidad + PI corriente)
- **ZERO_VEL**: Mantiene velocidad = 0 (freno activo)
- **BRAKE**: Cortocircuita fases (todos los FETs low-side on, high-side off)
- **STOPPED**: PWM deshabilitado, gates en HiZ
- **CALIBRATING**: Secuencia de calibración de parámetros motor

---

## 7. Hardware — Phoenix vs moteus-c1

| Feature | moteus-c1 | Phoenix (actual) |
|---|---|---|
| **MCU** | STM32G431 | STM32G431CBU6 ✅ |
| **Gate driver** | DRV8323 | DRV8323S ✅ (SPI, 3X PWM mode) |
| **PWM** | TIM2 (PA0-2), UP mode | TIM2 (PA0-2), center-aligned, ARR=4250 → 20 kHz ✅ |
| **Encoder** | AS5047 (SPI, absoluto) | AS5600 (I2C, absoluto) + incremental TIM1 (TI12) |
| **Corriente** | PA3 (Ia), PA6 (Ib), PB1 (Ic) | PA3 (Ia), PA6 (Ib), PB1 (Ic) ✅ |
| **Voltaje bus** | PA9 (ADC2_IN5) | Hardcodeado 24.0V (PA9 = TIM1_CH2) ⚠️ |
| **Temperatura** | PB12 | (por definir) |
| **CAN** | PB6 (TD), PB5 (RD) | ❌ No usado → SPI2 slave |
| **Comunicación** | CAN-FD | SPI2 slave (5-word frames, 16-bit, full-duplex) |
| **DRV8323 CS** | PB0 | PC13 (cambiado) |
| **DRV8323 EN** | PC14 | PC14 ✅ |
| **DRV8323 nFAULT** | PB13 | PB13 ✅ (con EXTI falling-edge) |
| **DRV8323 HiZ** | PC15 (family 1) | PC15 ✅ (PC15 initial HIGH = HiZ enabled, luego LOW al PowerOn) |
| **nSLEEP** | — | PC6 (HIGH = awake, añadido Phoenix) |
| **Power** | 250W / 150W | (por medir) |

---

## 8. Roadmap Próximo

### Fase 1 — SPI2 Slave Communication ✅ COMPLETED
- Protocolo 5-word definido e implementado
- Comandos: STOP, SETPOINT_POS/VEL/TORQUE, CALIBRATE, READ_STATUS
- IRQ-driven con doble buffer
- Telemetría incluida en respuesta

### Fase 2 — FOC Control Loop ✅ COMPLETED (Jul 2026)
- TIM6 @ 20 kHz, ISR-driven
- Clarke/Park con LUT trig (1024 entradas)
- PI controllers Id/Iq con feedforward (decoupling + BEMF)
- Inverse Park + SVM min-max
- Integración con encoder (ángulo eléctrico en ISR)
- Arranque seguro: rampa de corriente, alineación de encoder

### Fase 3 — Control de Alto Nivel ✅ COMPLETED
- PID de posición (cascade)
- PI de velocidad (cascade)
- PI de corriente (Id/Iq)
- Modos: torque, velocidad, posición, corriente, zero_vel, brake
- ServoControl state machine con manejo de faults

### Fase 4 — Calibración y Protecciones ⬜ EN PROGRESO
- ⬜ Calibración de offset de corriente (estructura preparada)
- ⬜ Medición de resistencia e inductancia (Rs/Ls)
- ⬜ Calibración de encoder (offset de conmutación)
- ⬜ Identificación de flujo BEMF (Ke)
- ⬜ Protecciones: overcurrent software, overtemperature, UVLO, stall detection
- ⬜ Watchdog de comunicación (timeout SPI2)

### Fase 5 — Integración Plena ⬜ PENDIENTE
- Comunicación bidireccional con placa master (SPI2 funcional, probar integración)
- Streaming de telemetría a frecuencia configurable
- Comandos de configuración vía SPI (PID gains, límites)
- Modo bootloader vía SPI
- Calibración automática de parámetros motor al startup
- VBUS real desde master vía SPI2
