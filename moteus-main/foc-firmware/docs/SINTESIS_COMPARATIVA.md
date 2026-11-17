# Síntesis Comparativa: Phoenix vs moteus-c1 (familia 2)

> **Documento:** Comparación sistemática entre el firmware Phoenix (foc-firmware) y su referencia original moteus-c1 (fw/ familia 2).
> **MCU:** STM32G431CBU6 @ 170 MHz (ambos)
> **Toolchain Phoenix:** CubeMX + STM32CubeIDE, bare-metal C con HAL
> **Toolchain moteus:** Bazel + mbed OS, C++ con mbed abstractions
> **Fecha:** 2026-07-08

---

## Resumen Ejecutivo

El proyecto Phoenix es una reimplementación desde cero del firmware de control motor brushless basado en la arquitectura del **moteus-c1 (familia 2)** de mjbots. Ambos sistemas comparten el mismo MCU (STM32G431CBU6) y gate driver (DRV8323S), pero divergen en aspectos fundamentales de diseño: Phoenix usa **CubeMX + HAL C** frente al **Bazel + mbed C++** de moteus, reemplaza CAN-FD por **SPI2 slave** como interfaz de comunicación, usa **AS5600 I2C + encoder incremental** en vez del **AS5047 SPI** monolítico, y su lazo FOC corre a **20 kHz** (vs 15-30 kHz en moteus).

**Problema crítico identificado:** El timer PWM (TIM2) tiene su auto-reload configurado en **0xFFFFFFFF** (máximo 32-bit), produciendo PWM cuasi-DC (~0.02 Hz) en lugar de switching real a 20 kHz. Los duty cycles se actualizan correctamente desde la ISR de TIM6, pero las transiciones físicas de salida ocurren en escala de segundos.

**Brecha de gate drive:** Las corrientes de gate configuradas en Phoenix (IDRIVEP_HS=370 mA, IDRIVEN_HS=740 mA) son sustancialmente mayores que las de moteus-c1 (60 mA / 20 mA respectivamente), lo que puede indicar FETs con mayor gate charge o una configuración aún no calibrada para el hardware específico.

---

## 1. Identidad de los Sistemas

| Aspecto | Phoenix | moteus-c1 (familia 2) |
|---------|---------|----------------------|
| **Propósito** | Controlador FOC brushless para robótica | Controlador FOC brushless para robótica |
| **PCB** | Custom "Phoenix" (basada en c1) | mjbots moteus-c1 |
| **MCU** | STM32G431CBU6 (UFQFPN48) | STM32G431 (UFQFPN48) |
| **Frecuencia** | 170 MHz (HSI+PLL, Boost mode) | 170 MHz (HSE o HSI+PLL) |
| **Toolchain** | CubeMX v6.17 + STM32CubeIDE (GCC) | Bazel + mbed OS 6 |
| **Lenguaje** | C (HAL) | C++ (mbed) |
| **RTOS** | No (bare-metal, ISR-driven) | mbed OS (abstracción, sin RTOS preemptivo pesado) |
| **IDE** | STM32CubeIDE | Bazel (línea de comandos) |
| **Actualización firmware** | Por depurador SWD (pendiente bootloader SPI2) | Por CAN-FD (bootloader integrado) |
| **Licencia** | Apache 2.0 (derivada) | Apache 2.0 |

---

## 2. Arquitectura de Software

### 2.1 Capas del Firmware

| Capa | Phoenix | moteus-c1 |
|------|---------|-----------|
| **HAL** | STM32 HAL (generado por CubeMX) | mbed HAL + `stm32g4_adc.h`, `stm32_bitbang_spi.h` |
| **Drivers** | Tools/drv8323.c, Tools/as5600.c, Tools/encoder.c | fw/drv8323.cc, fw/aux_spi.h (AS5047) |
| **Control** | Tools/foc.c, Tools/servo_control.c, Tools/pid.h | fw/bldc_servo.cc, fw/foc.cc, fw/bldc_servo_control.h |
| **Comunicación** | Tools/comm_spi.c (SPI2 slave, 80-bit frames) | fw/fdcan.h (CAN-FD, 5 Mbps) |
| **Estado** | ServoControl state machine (7 estados) | BldcServo state machine |
| **Configuración** | Defines en headers + comandos SPI2 | PersistentConfig vía micro::mjlib |

### 2.2 Flujo de Control

| Aspecto | Phoenix | moteus-c1 |
|---------|---------|-----------|
| **Lazo FOC** | TIM6 ISR @ 20 kHz (50 µs) | TIM2 update ISR (15-30 kHz) |
| **Trigger ADC** | TIM2 TRGO (update event → 2× por ciclo PWM) | LPTIM1 (disparo por CC4 + DMA) |
| **Trigonometría** | LUT 1024 entradas (sin/cos Q15) | CORDIC hardware (9 ciclos, 53 ns) |
| **SVM** | Min-max injection (centrado) | `ISR_DoBalancedVoltageControl` (equivalente) |
| **Control corriente** | PI con feedforward decoupling + BEMF | PI con feedforward, conditional integration |
| **Control velocidad** | PI cascade sobre corriente | PI cascade |
| **Control posición** | PID cascade sobre velocidad | PID cascade |
| **Anti-windup** | Clamp de integrador | Back-calculation |
| **Tiempo ISR** | ~8 µs (16% de ventana 50 µs) | ~3-5 µs (típico) |

### 2.3 Módulos y Correspondencia de Archivos

| Función | Phoenix | moteus-c1 (fw/) |
|---------|---------|------------------|
| **Inicialización HAL** | Core/Src/main.c, Core/Src/*.c | fw/moteus_hw.cc, fw/moteus_hw.h |
| **Config pines** | cubemx/FOCTest.ioc | fw/moteus_hw.h (MoteusHwPins) |
| **PWM timer** | Core/Src/tim.c (TIM2) | fw/bldc_servo.cc (ConfigurePwmTimer) |
| **Encoder** | Tools/encoder.c, Tools/encoder.h | fw/aux_mbed.h (Stm32Quadrature) |
| **Encoder absoluto** | Tools/as5600.c, Tools/as5600.h | fw/aux_spi.h (AS5047 SPI) |
| **Gate driver** | Tools/drv8323.c, Tools/drv8323.h | fw/drv8323.cc, fw/drv8323.h |
| **Driver motor (interfaz)** | — (no hay interfaz abstracta) | fw/motor_driver.h |
| **FOC algorithm** | Tools/foc.c, Tools/foc.h | fw/foc.cc, fw/foc.h |
| **Trigonometría** | Tools/trig_lut.h | CORDIC + fw/bldc_servo_control.h |
| **PID control** | Tools/pid.h | fw/bldc_servo_control.h |
| **Servo state machine** | Tools/servo_control.c, Tools/servo_control.h | fw/bldc_servo.cc, fw/bldc_servo.h |
| **Comunicación host** | Tools/comm_spi.c, Tools/comm_spi.h | fw/fdcan.h, fw/fdcan.cc |
| **ADC** | Core/Src/adc.c, Core/Inc/adc.h | fw/stm32g4_adc.h, fw/stm32g4_adc.cc |
| **Hardware math** | Core/Src/cordic.c (inicializado, no usado) | fw/bldc_servo_control.h (CORDIC activo) |
| **Calibración** | (esqueleto, no implementado) | fw/drv8323.cc, fw/bldc_servo.cc |
| **Protecciones** | EXTI PB13 + D RV8323_ReadStatus() | fw/drv8323.cc (PollMillisecond + fault ISR) |
| **I2C** | Core/Src/i2c.c (AS5600) | (no usado en c1) |
| **SPI** | Core/Src/spi.c (SPI2 slave, SPI3 master) | fw/stm32_bitbang_spi.h |

---

## 3. Mapeo de Pines Comparativo

### 3.1 Tabla de Pines

| Pin | Phoenix | moteus-c1 (familia 2) | ¿Coinciden? |
|-----|---------|----------------------|:-----------:|
| PA0 | TIM2_CH1 (PWM U) | TIM2_CH1 (PWM U) | ✅ |
| PA1 | TIM2_CH2 (PWM V) | TIM2_CH2 (PWM V) | ✅ |
| PA2 | TIM2_CH3 (PWM W) | TIM2_CH3 (PWM W) | ✅ |
| **PA3** | ADC1_IN4 (Ia) | ADC1 (Ia) | ✅ |
| **PA6** | ADC2_IN3 (Ib) | ADC2 (Ib) | ✅ |
| PA8 | TIM1_CH1 (Encoder A) | msense (temp) o ADC5_IN1 | ⚠️ |
| **PA9** | **TIM1_CH2 (Encoder B)** | **ADC2_CH5 (VBUS sense)** | **🔴 CONFLICTO** |
| PA11 | SPI2_MOSI | CAN_RD (family 1+ CAN) | ❌ función diferente |
| PB0 | GPIO output (libre) | DRV8323 CS (moteus family 1+) | ❌ |
| **PB1** | ADC1_IN12 (Ic) | ADC3_IN12 (Ic) | ⚠️ mismo pin, ADC diferente |
| PB5 | SPI3_MOSI (DRV8323) | CAN_TD (family 1+) | ❌ |
| PB6 | (no usado en Phoenix) | CAN_TD (family 1+) | ❌ |
| PB13 | EXTI13 (nFAULT) | DRV8323 nFAULT | ✅ |
| PB14 | SPI2_MISO | (no usado, o debug) | ❌ |
| PB15 | GPIO output (debug LED) | Debug LED | ✅ |
| PC4 | I2C2_SCL (AS5600) | (no usado, CS family 0) | ❌ |
| PC6 | nSLEEP (DRV8323) | Power LED | ❌ |
| PC10 | SPI3_SCK (DRV8323) | DRV8323 SCK (bitbang) | ✅ |
| PC11 | SPI3_MISO (DRV8323) | DRV8323 MISO (bitbang) | ✅ |
| PC13 | DRV8323 CS | DRV8323 MOSI (bitbang) | ❌ |
| PC14 | DRV8323 EN | DRV8323 EN | ✅ |
| PC15 | DRV8323 HiZ | DRV8323 HiZ | ✅ |
| PF0 | I2C2_SDA (AS5600) | Debug LED (family 0) | ❌ |
| PF1 | SPI2_SCK (host comm) | Power LED (family 0) | ❌ |

### 3.2 Diferencias Funcionales por Pin

#### PA9 — Conflicto Crítico ⚠️

| Sistema | Función PA9 | Consecuencia |
|---------|-------------|--------------|
| **moteus-c1** | ADC2_CH5 (VBUS sense). La tensión de bus DC se mide en tiempo real para alimentar SVM y protecciones UVLO. | VBUS medido a 1 kHz con filtro dual (lento+rápido). Compensación activa en lazo de corriente. |
| **Phoenix** | TIM1_CH2 (Encoder B, canal incremental). Configurado como entrada de encoder en modo TI1 (solo CH1 cuenta, CH2 da dirección). | VBUS **hardcodeado a 24.0V**. Sin realimentación de bus, el SVM no compensa variaciones de batería (2:1 es típico en LiPo). |

**Impacto:** Este conflicto de pin es la limitación de hardware más significativa de Phoenix. La solución arquitectónica adoptada es recibir VBUS por telemetría SPI2 desde la placa master, lo cual introduce latencia y depende del protocolo de comunicación.

#### DRV8323 — SPI y Pines de Control

| Parámetro | Phoenix | moteus-c1 |
|-----------|---------|-----------|
| **Interfaz SPI** | SPI3 hardware (5.3 MHz) | Bitbang (~1 MHz) |
| **CS** | PC13 | PB0 |
| **MOSI** | PB5 | PC13 |
| **MISO** | PC11 | PC11 (igual) |
| **SCK** | PC10 | PC10 (igual) |
| **EN** | PC14 | PC14 (igual) |
| **HiZ** | PC15 | PC15 (igual) |
| **nFAULT** | PB13 (EXTI falling) | PB13 |
| **nSLEEP** | PC6 (añadido en Phoenix) | No presente |

#### Canales ADC de Corriente

| Fase | Phoenix | moteus-c1 | Diferencia |
|------|---------|-----------|------------|
| Ia | ADC1_IN4 (PA3) | ADC1 (PA3) | ✅ igual |
| Ib | ADC2_IN3 (PA6) | ADC2 (PA6) | ✅ igual |
| Ic | ADC1_IN12 (PB1) | ADC3_IN12 (PB1) | ⚠️ **ADC diferente** |

**Impacto:** En Phoenix, Ia e Ic comparten ADC1 en modo scan → muestreo secuencial con ~353 ns de skew. En moteus, cada fase tiene su propio ADC (ADC1, ADC2, ADC3) para muestreo verdaderamente simultáneo.

#### Comunicación con Host

| Aspecto | Phoenix | moteus-c1 |
|---------|---------|-----------|
| **Protocolo** | SPI2 slave (16-bit, full-duplex) | CAN-FD (5 Mbps) |
| **Pines** | PF1 (SCK), PB14 (MISO), PA11 (MOSI) | PB6 (TD), PB5 (RD) |
| **Formato trama** | 5 palabras × 16-bit (80 bits) | CAN-FD estándar (hasta 64 bytes) |
| **Arbitraje** | No (dedicado, master selecciona) | CSMA/CR (multi-drop nativo) |
| **Distancia** | Corta (SPI, típicamente <30 cm) | Larga (CAN, hasta 40 m a 1 Mbps) |
| **Multi-dispositivo** | No (punto a punto) | Sí (hasta 127 nodos) |
| **Throughput** | Limitado por master (~1-5 kHz frames) | 5 Mbps, hasta ~30k frames/s |

---

## 4. Lazo FOC (Field-Oriented Control)

### 4.1 Parámetros del Lazo

| Parámetro | Phoenix | moteus-c1 |
|-----------|---------|-----------|
| **Frecuencia FOC** | 20 kHz (50 µs) | 15-30 kHz (configurable) |
| **Timer FOC** | TIM6 (básico) | TIM2 update ISR |
| **Trigger ISR** | TIM6 update event | TIM2 update event |
| **Prioridad ISR** | Preempt 0 (máxima) | Default (mbed) |
| **Duración ISR** | ~8 µs (16% CPU) | ~3-5 µs |
| **Trig ADC** | TIM2 TRGO (update) → 2×/ciclo PWM | TIM2 CC4 → LPTIM1 → ADCs (1×/ciclo) |
| **Trig activo** | CORDIC inicializado, **no usado** | CORDIC activo (sin/cos en HW) |
| **Trigonometría** | LUT 1024 entradas (±0.1° error) | CORDIC 9 ciclos (precisión completa Q31) |
| **Feedforward decoupling** | Implementado (−ωₑ·L·i) | Implementado |
| **Feedforward BEMF** | Implementado | Implementado (configurable) |
| **Flux weakening** | No implementado | Implementado (CVCP) |

### 4.2 Secuencia por Ciclo FOC

**Phoenix (TIM6 ISR):**
1. Leer ADC1/2 DR (datos listos por trigger HW)
2. `Encoder_Update()` — posición, velocidad, ángulo eléctrico
3. `GetCurrents()` — raw ADC → amperios (con ganancia CSA 40 V/V)
4. **Clarke transform** — Ia, Ib, Ic → Iα, Iβ
5. **Park transform** — Iα, Iβ → Id, Iq (LUT sin/cos)
6. **PI Id** — control de eje directo (Id_ref = 0)
7. **PI Iq** — control de eje en cuadratura
8. **Feedforward** — decoupling + BEMF
9. **Inverse Park** — Vd, Vq → Vα, Vβ
10. **SVM min-max** — Vα, Vβ → duty cycles U, V, W
11. **Escribir PWM** — `__HAL_TIM_SET_COMPARE(TIM2, CCR1-3)`

**moteus-c1 (TIM2 update ISR):**
1. Leer ADC1/2/3 DR (datos simultáneos por trigger LPTIM1)
2. `MotorPosition::Update()` — encoder + PLL
3. `ISR_DoCurrent()` — Clarke + Park + PI + feedforward + Inverse Park
4. `ISR_DoBalancedVoltageControl()` — SVM centrado
5. Escribir TIMx CCR (vía DMA o registro directo)

### 4.3 Precisión de la Trigonometría

| Método | Phoenix (LUT 1024) | moteus (CORDIC) |
|--------|-------------------|-----------------|
| **Ciclos por sin+cos** | ~3 (lecturas de tabla) | ~9 (hardware) |
| **Tiempo @ 170 MHz** | ~5-10 ns (caché hit) | ~53 ns |
| **Precisión** | 2π/1024 ≈ 0.35° (error de cuantización) | <0.01° (Q31, precisión completa) |
| **Determinismo** | Sí (acceso a memoria predecible) | Sí (pipeline HW fijo) |
| **Jitter** | Dependiente de caché | Cero (HW síncrono) |
| **Uso de CPU** | Mínimo | Mínimo |

---

## 5. Gate Driver DRV8323S

### 5.1 Configuración Comparativa

| Registro | Parámetro | Phoenix | moteus-c1 | Diferencia |
|----------|-----------|---------|-----------|:----------:|
| **DRV_CTRL1** | PWM_MODE | 3X (tres PWM independientes) | 3X | ✅ = |
| | IDRIVEP_HS | **370 mA** (config, register idx 2→260 mA real) | **60 mA** | **⚠️ ~6×** |
| | IDRIVEN_HS | **740 mA** (config, register idx 1→740 mA real) | **20 mA** | **⚠️ 37×** |
| **DRV_CTRL2** | IDRIVEP_LS | 370 mA | 60 mA | ⚠️ ~6× |
| | IDRIVEN_LS | 740 mA | 20 mA | ⚠️ 37× |
| | DEAD_TIME | 50 ns | 50 ns | ✅ = |
| | OCP_MODE | Latched fault | Latched fault | ✅ = |
| | OCP_DEG | 4 µs | 4 µs | ✅ = |
| | VDS_LVL | **260 mV** | **940 mV** | ⚠️ Phoenix más sensible |
| **CSA_CTRL** | CSA_GAIN | 20 V/V | 20 V/V | ✅ = |
| | VREF_DIV | /2 (bidireccional) | /2 | ✅ = |
| | SEN_LVL | 500 mV | 500 mV | ✅ = |
| | CSA_FET | Shunt (0) | Shunt (0) | ✅ = |

### 5.2 Implicancias de las Corrientes de Gate Drive

Las corrientes de gate drive en Phoenix (IDRIVEP_HS=370 mA, IDRIVEN_HS=740 mA) son considerablemente más altas que en moteus-c1 (60 mA / 20 mA). Esto tiene implicancias directas:

| Aspecto | Phoenix (370/740 mA) | moteus-c1 (60/20 mA) |
|---------|---------------------|---------------------|
| **Velocidad de conmutación** | Muy rápida (alta dv/dt) | Moderada |
| **Pérdidas de conmutación** | Bajas (transiciones rápidas) | Más altas |
| **Ruido EMI** | Alto (posible ringing en gates) | Controlado |
| **Estrés en FETs** | Mayor (overshoot en Vgs) | Menor |
| **Requisito de layout** | Crítico (minimizar parásitos) | Tolerante |

**Posible explicación:** Phoenix puede estar usando FETs con mayor gate charge total (Q_g) que requieren más corriente de gate para conmutar a la frecuencia PWM deseada. Sin embargo, la diferencia de 37× en la corriente de sink (IDRIVEN) sugiere que la configuración de Phoenix no ha sido ajustada para FETs específicos y probablemente usa valores por defecto agresivos de la aplicación de referencia del DRV8323.

### 5.3 Umbral de VDS (OCP)

Con VDS_LVL = 260 mV, Phoenix disparará OCP ante una caída de tensión FET de aproximadamente un **tercio** de la que tolera moteus-c1 (940 mV). Esto implica:

- **Protección más temprana** ante sobrecorriente (Vds = I_d × R_ds(on))
- **Posibles falsos positivos** durante transitorios de arranque, especialmente si los FETs tienen R_ds(on) alta
- Si los FETs tienen diferentes R_ds(on) que los de c1, este umbral podría estar correctamente dimensionado

---

## 6. Encoder y Sensado de Posición

| Aspecto | Phoenix | moteus-c1 |
|---------|---------|-----------|
| **Encoder absoluto** | AS5600 (I2C, 12-bit, 0.5°) | AS5047 (SPI, 14-bit, 0.022°) |
| **Encoder incremental** | TIM1 TI12 (PA8, PA9), 4× CPR | No necesita (absoluto nativo) |
| **CPR efectivo** | 4096 pulsos/rev (1024 CPR × 4) | 16384 pulsos/rev (14-bit AS5047) |
| **Resolución angular** | 0.088° (mecánica) | 0.022° (mecánica) |
| **Velocidad** | Derivada por delta posición/LPF | PLL + delta posición |
| **Voltaje de operación** | 3.3V o 5V (AS5600) | 3.3V o 5V (AS5047) |
| **Interfaz** | I2C (400 kHz) + TIM1 quad | SPI (10 MHz) |
| **Lectura startup** | Bloqueante (polling I2C ~1-5 ms) | SPI rápida (~1 µs) |
| **Precisión** | ±0.5° (típico AS5600, sin calibración automática) | ±0.1° (con calibración dinámica) |

**Observaciones:**

- La combinación AS5600 + incremental en Phoenix es más compleja (dos ICs, dos buses de comunicación) pero ofrece redundancia y permite operación a alta velocidad con el incremental.
- El AS5047 de moteus es un encoder SPI monolítico que provee posición absoluta a 14 bits (16384 CPR) sin necesidad de encoder incremental externo. La interfaz es más simple y rápida.
- La resolución 4× superior del AS5047 permite un control de baja velocidad más suave.
- Phoenix actualmente configura el AS5600 en modo 12-bit (0.5°) no en modo 16-bit (0.088°), posiblemente por limitaciones de ruido.

---

## 7. ADC y Sensado de Corriente

### 7.1 Configuración

| Parámetro | Phoenix | moteus-c1 |
|-----------|---------|-----------|
| **Frecuencia ADC** | 42.5 MHz (SYSCLK/4) | 85 MHz (configurable) |
| **Resolución** | 12-bit | 12-bit |
| **Tiempo de muestreo** | 47.5 ciclos (1.12 µs) | Configurable (≥CSA settling) |
| **Trigger** | TIM2 TRGO (update) | LPTIM1 OUT (DWT cycle-accurate) |
| **Modo** | Dual regular parallel | 3 ADCs independientes |
| **Ic** | ADC1_IN12 (scan con Ia) | ADC3_IN12 (independiente) |
| **VBUS** | Hardcodeado 24.0V | ADC5_IN2 (PA9) con filtro dual |
| **Temperatura FET** | No implementado | ADC4_IN12 |
| **Temperatura motor** | No implementado | ADC5_IN1 |
| **Calibración ADC** | Solo ADC1 | Todos los ADCs |

### 7.2 Tiempo de Asentamiento CSA

El DRV8323 requiere que el amplificador de corriente (CSA) se asiente antes de muestrear. La ganancia de 20 V/V necesita al menos **1.5 µs** de conducción low-side (1 µs datasheet + 500 ns margen empírico, según `fw/drv8323.h`). Ambos sistemas deben garantizar esto mediante:

- **Phoenix:** Sampling time de 47.5 ciclos (1.12 µs) + clamp de duty cycle mínimo
- **moteus-c1:** `CsaSettlingTime(20) = 1.50e-6` incorporado en `min_pwm`/`max_pwm` vía `RateConfig`

---

## 8. Comunicación y Protocolo

| Aspecto | Phoenix (SPI2 slave) | moteus-c1 (CAN-FD) |
|---------|---------------------|--------------------|
| **Tipo** | Serial síncrono (SPI) | Bus diferencial (CAN-FD) |
| **Velocidad** | Depende del master (típico 1-10 MHz) | 5 Mbps |
| **Topología** | Punto a punto (1 master ↔ 1 slave) | Multi-drop (hasta 127 nodos) |
| **Distancias** | <30 cm | Hasta 40 m |
| **Inmunidad ruido** | Baja (señales single-ended, sin terminación diferencial) | Alta (par diferencial) |
| **Protocolo** | 5-word frame (80 bits) | CAN-FD estándar (ISO 11898-1) |
| **Detección errores** | No (sin CRC, sin ACK) | CRC, ACK, bit stuffing, form check |
| **Arbitraje** | No (CS controlado por master) | CSMA/CR con prioridades |
| **Comandos implementados** | STOP, SETPOINT_POS/VEL/TORQUE, CALIBRATE, READ_STATUS | Extenso (configuración, calibración, diagnóstico, actualización FW) |
| **Telemetría** | Posición, velocidad, Iq, estado/fault | Completa (posición, velocidad, corriente, voltajes, temperaturas, faults, config) |

**Decisión arquitectónica:** Phoenix reemplaza CAN-FD por SPI2 slave porque el sistema está diseñado para operar con una placa master dedicada (comunicación punto a punto en el mismo robot), no como nodo en un bus CAN multi-dispositivo. Esto simplifica el hardware (sin transceptor CAN) pero limita la escalabilidad y la inmunidad al ruido.

---

## 9. Diferencias Arquitectónicas por Diseño

### 9.1 Toolchain y Entorno de Desarrollo

| Phoenix | moteus-c1 | Implicancia |
|---------|-----------|-------------|
| CubeMX + HAL C | Bazel + mbed C++ | Phoenix: más fácil comenzar, pero el HAL genera código verboso y las ISR tienen overhead. Moteus: mbed abstrae periféricos, pero la toolchain Bazel tiene curva de aprendizaje. |
| Generación de código automática | Código manual completo | Phoenix: cambios en IOC pueden sobrescribir código usuario si no se usan USER CODE. Moteus: control manual completo de cada registro. |
| Sin pruebas automatizadas | Tests host (Bazel) + simulación | Phoenix no tiene suite de regresión. |
| Depuración por SWD | Depuración por CAN-FD + SWD | Phoenix requiere conexión física SWD para depuración. |

### 9.2 Estilo de Programación

| Aspecto | Phoenix | moteus-c1 |
|---------|---------|-----------|
| **Lenguaje** | C (procedural) | C++ (orientado a objetos) |
| **Abstracción periféricos** | HAL (structs + handles) | mbed (clases DigitalOut, SPI, etc.) |
| **Máquina de estados** | Switch-case en struct | State machine con template (BldcServo) |
| **Configuración** | Defines en tiempo de compilación | Config persistente (mjlib/micro) |
| **Telemetría** | Hardcodeada en respuesta SPI2 | Declarativa (mjlib/micro telemetry) |

### 9.3 Manejo de Fallos

| Aspecto | Phoenix | moteus-c1 |
|---------|---------|-----------|
| **Fault HW** | EXTI PB13 (falling edge, prio 0) → MOE clear | nFAULT GPIO, check en ISR |
| **Fault SW** | Polling `DRV8323_ReadStatus()` en main loop | `PollMillisecond()` + `fault()` en ISR |
| **Auto-retry** | Hasta 3 reconexiones con delay | `RecoverFault()` state machine |
| **Watchdog SPI2** | No implementado | No aplica (CAN tiene timeout nativo) |
| **Overcurrent SW** | No implementado | Medido en ISR, saturación inmediata |
| **Overtemperature** | No implementado | LPF + derating de corriente máxima |

---

## 10. Problemas Críticos

### CRÍTICO-1: TIM2 ARR = 0xFFFFFFFF (PWM Cuasi-DC)

**Severidad:** 🔴 CRÍTICO — El motor no conmuta realmente.

**Descripción:** El auto-reload de TIM2 está configurado en 0xFFFFFFFF (máximo 32-bit). Con center-aligned mode 2, la frecuencia PWM efectiva es:

```
f_pwm = TIM2CLK / (2 × ARR) = 170 MHz / (2 × 4,294,967,295) ≈ 0.02 Hz
```

Los duty cycles se actualizan a 20 kHz desde la ISR de TIM6 (vía `__HAL_TIM_SET_COMPARE`), pero el timer tarda ~50 segundos en completar un ciclo de conteo. Las transiciones físicas PWM ocurren en escala de segundos, no microsegundos.

**Impacto:** El inversor opera en modo cuasi-DC. Las fases ven corriente continua con pequeñas modulaciones. El motor puede girar (se ha observado que cierta conmutación ocurre por la actualización periódica de CCR), pero con:
- Eficiencia extremadamente reducida (pérdidas resistivas altas)
- Rizado de corriente máximo (sin filtrado por inductancia de motor)
- Potencial sobrecalentamiento de FETs por conducción DC prolongada
- Sin beneficio de conmutación PWM (sin reducción de pérdidas de conmutación vs conducción)

**Causa raíz:** Error de configuración en CubeMX o en la migración del IOC. El valor esperado es **4250** (para 20 kHz PWM center-aligned con TIM2CLK=170 MHz).

**Solución:** Cambiar `TIM2.Period` de `4294967295` (0xFFFFFFFF) a **4250** (20 kHz, decisión actual). Asegurar que el cambio esté en sección USER CODE para que CubeMX no lo sobrescriba al regenerar código.

### CRÍTICO-2: PA9 en Conflicto (Encoder vs VBUS)

**Severidad:** 🔴 CRÍTICO — Sin VBUS real, el lazo de corriente no compensa variaciones de tensión de batería.

**Descripción:** El pin PA9 no puede ser simultáneamente TIM1_CH2 (encoder B, necesario para encoder incremental) y ADC2_CH5 (VBUS sense, necesario para SVM). Se eligió mantener encoder.

**Impacto:** VBUS hardcodeado a 24.0V en `main.c`. La variación típica de batería LiPo (9.6-12.6 V para 3S, o 22.2-29.4 V para 6S) produce un error en el SVM que escala linealmente con la desviación de VBUS:
- A 12 V real con VBUS=24.0V: el SVM aplica 2× la tensión debida → saturación de PI y sobrecorriente
- A 36 V real con VBUS=24.0V: el SVM aplica 0.67× la tensión debida → torque limitado

**Solución arquitectónica:** Recibir VBUS por SPI2 desde la placa master. La master mide VBUS con su ADC y lo envía en cada trama de comando.

### CRÍTICO-3: Ic en ADC1 (Scan) vs ADC3 (Independiente)

**Severidad:** 🟠 ALTO — Skew temporal entre Ia e Ic de ~353 ns.

**Descripción:** En Phoenix, Ia (ADC1_IN4) e Ic (ADC1_IN12) comparten ADC1 en modo scan. Se convierten secuencialmente: primero Ia, luego Ic. Moteus-c1 asigna Ic a ADC3_IN12 (mismo pin PB1), dando a cada fase su propio ADC.

**Impacto:** El skew de ~353 ns entre Ia e Ic introduce un error en la transformada de Park que escala con la velocidad del motor. A 10,000 RPM (7 pares de polos → 1167 Hz eléctricos): 353 ns × 1167 Hz × 360° ≈ 0.15° de error eléctrico. Bajo pero no despreciable.

**Solución:** Reconfigurar PB1 como ADC3_IN12 (el pin PB1 tiene mux a ambos ADCs) y usar tres ADC independientes.

---

## 11. Problemas Medios

### MEDIO-1: Corrientes de Gate Drive Excesivas

**Severidad:** 🟡 MEDIO — Potencial ringing, EMI, y estrés en FETs.

Las corrientes IDRIVEP=370 mA e IDRIVEN=740 mA son 6× y 37× mayores que las de moteus-c1 (60/20 mA). Verificar que los FETs instalados en Phoenix tengan gate charge compatible. Si los FETs son pequeños (típicos de c1: ~10-20 nC Q_g), 740 mA de sink pueden causar:
- Ringing de alta frecuencia en el gate (>50 MHz)
- Overshoot de Vgs que puede exceder el rating máximo (±20V típico)
- EMI radiada que afecta al encoder AS5600 (I2C sensible)

**Recomendación:** Medir la forma de onda de Vgs con osciloscopio. Si se observa ringing, reducir IDRIVEN a 20-100 mA (como moteus-c1).

### MEDIO-2: VDS_LVL más Sensible (260 mV vs 940 mV)

**Severidad:** 🟡 MEDIO — Falsos positivos de OCP durante transitorios normales.

Phoenix usa VDS_LVL=260 mV, mientras moteus-c1 usa 940 mV para la misma familia. Si los FETs tienen R_ds(on) similar, el OCP de Phoenix disparará a ~1/3 de la corriente que moteus. Esto puede causar:
- Disparos espurios durante el arranque (cuando la corriente es alta por la saturación del PI)
- Falsa detección de sobrecorriente en transitorios de carga

**Recomendación:** Verificar la corriente de disparo OCP real: I_ocp = VDS_LVL / R_ds(on). Para un FET típico con R_ds(on)=3 mΩ: 260 mV / 3 mΩ = 87 A. Si es adecuado, mantener. Si hay falsos positivos, incrementar a 520 mV o 940 mV.

### MEDIO-3: TIM1 en Modo TI1 (Solo 2× Resolución)

**Severidad:** 🟡 MEDIO — Resolución de encoder reducida a la mitad.

TIM1 está configurado como `TIM_ENCODERMODE_TI1` (cuenta solo en flancos de CH1, usa CH2 solo para dirección). Esto da **2048 pulsos/rev** (1024 CPR × 2). En modo TI12 daría **4096 pulsos/rev** (1024 CPR × 4).

**Impacto:** A baja velocidad, la resolución de velocidad es más gruesa, empeorando el control de baja velocidad.

**Solución:** Cambiar a `TIM_ENCODERMODE_TI12` en `tim.c`.

### MEDIO-4: ADC2 No Calibrado

**Severidad:** 🟡 MEDIO — Offset de ±0.5 A en medición de Ib.

Solo ADC1 recibe calibración (`HAL_ADCEx_Calibration_Start`). ADC2 (Ib) opera con offset sin calibrar, que puede ser de hasta ±4 LSB (±4.9 mV). A 40 V/V de ganancia CSA: ±4.9 mV / (40 × 0.5 mΩ) ≈ ±0.25 A de error solo en Ib.

**Impacto:** Error diferencial en la transformada de Clarke, que produce Id/Iq erróneos incluso con corriente real cero.

**Solución:** Añadir `HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED)`.

### MEDIO-5: Protecciones Incompletas

**Severidad:** 🟡 MEDIO — Riesgo de daño en condiciones de fallo no cubiertas.

| Protección | Phoenix | moteus-c1 |
|------------|---------|-----------|
| Overcurrent HW | ✅ EXTI nFAULT (DRV8323) | ✅ DRV8323 HW + nFAULT |
| Overcurrent SW | ❌ No implementado | ✅ En ISR, satura inmediata |
| Overtemperature FET | ❌ No implementado | ✅ ADC4 + LPF + derating |
| Overtemperature motor | ❌ No implementado | ✅ ADC5 + LPF + derating |
| UVLO (bajo voltaje) | ❌ No implementado (sin VBUS real) | ✅ Comparación en ISR |
| Stall detection | ❌ No implementado | ✅ Watchdog de velocidad/posición |
| Watchdog SPI2 | ❌ No implementado | ✅ CAN timeout nativo |

### MEDIO-6: Calibración de Motor Pendiente

**Severidad:** 🟡 MEDIO — Parámetros de motor no identificados, control sub-óptimo.

| Calibración | Phoenix | moteus-c1 |
|-------------|---------|-----------|
| Offset ADC | Esqueleto (estructura preparada) | Implementado |
| Rs (resistencia fase) | No implementado | Medición automática |
| Ld, Lq (inductancia) | No implementado | Inyección de tensión |
| Ke (constante BEMF) | No implementado | Identificación automática |
| Offset encoder | No implementado | Alineación automática |
| Conmutación | No implementado | Tablas de compensación |

---

## 12. Tabla de Correspondencias de Archivos

### 12.1 Archivos de Código Fuente

| Función | Phoenix (ruta en foc-firmware/) | moteus-c1 (ruta en fw/) |
|---------|--------------------------------|------------------------|
| Aplicación principal | `Core/Src/main.c` | `fw/moteus.cc` |
| Config hardware | `HARDWARE.md`, `cubemx/FOCTest.ioc` | `fw/moteus_hw.cc`, `fw/moteus_hw.h` |
| HAL GPIO | `Core/Src/gpio.c` | `fw/moteus_hw.cc` (mbed DigitalOut) |
| HAL ADC | `Core/Src/adc.c` | `fw/stm32g4_adc.h`, `fw/stm32g4_adc.cc` |
| HAL TIM | `Core/Src/tim.c` | `fw/bldc_servo.cc` (direct register) |
| HAL SPI | `Core/Src/spi.c` | `fw/stm32_bitbang_spi.h` |
| HAL I2C | `Core/Src/i2c.c` | (no usado en c1) |
| HAL CORDIC | `Core/Src/cordic.c` | `fw/bldc_servo_control.h` (inline) |
| ISRs | `Core/Src/stm32g4xx_it.c` | (en varios archivos, mbed gestiona) |
| Driver DRV8323 | `Tools/drv8323.c`, `Tools/drv8323.h` | `fw/drv8323.cc`, `fw/drv8323.h` |
| Driver AS5600 | `Tools/as5600.c`, `Tools/as5600.h` | (no usado, usa AS5047) |
| FOC algorithm | `Tools/foc.c`, `Tools/foc.h` | `fw/foc.cc`, `fw/foc.h` |
| Trig lookup table | `Tools/trig_lut.h` | — (usa CORDIC) |
| PID control | `Tools/pid.h` | `fw/bldc_servo_control.h` |
| Servo control | `Tools/servo_control.c`, `Tools/servo_control.h` | `fw/bldc_servo.cc`, `fw/bldc_servo.h`, `fw/bldc_servo_control.h` |
| Comunicación SPI | `Tools/comm_spi.c`, `Tools/comm_spi.h` | — (no usa SPI host) |
| Comunicación CAN | — (no implementado) | `fw/fdcan.h`, `fw/fdcan.cc` |
| Encoder incremental | `Tools/encoder.c`, `Tools/encoder.h` | `fw/aux_mbed.h` (Stm32Quadrature) |
| Encoder SPI (AS5047) | — (usa AS5600 I2C) | `fw/aux_spi.h` |
| Motor driver interface | — (no hay interfaz abstracta) | `fw/motor_driver.h` |
| Estado firmware | `docs/FIRMWARE_STATUS.md` | `README.md` + `CHANGELOG.md` |

### 12.2 Archivos de Documentación

| Documento | Propósito |
|-----------|-----------|
| `README.md` (raíz) | Descripción general del repositorio moteus |
| `foc-firmware/HARDWARE.md` | Config detallada de hardware Phoenix |
| `foc-firmware/docs/FIRMWARE_STATUS.md` | Estado de cada componente del firmware Phoenix |
| `foc-firmware/docs/MOTEUS_REFERENCE.md` | Comparación periférico a periférico con moteus |
| `foc-firmware/docs/DISCREPANCIAS.md` | Discrepancias entre documentación y código |
| `foc-firmware/docs/CONCLUSIONES.md` | Conclusiones del proyecto Phoenix |
| `foc-firmware/docs/INTEGRATION_REVIEW.md` | Review de integración vs moteus (C1-C6, M1-M5) |
| `foc-firmware/docs/PLAN_AVANCE.md` | Plan de trabajo y roadmaps |
| `foc-firmware/docs/REVISIÓN DE INTEGRACIÓN CORREGIDA — Proyecto Phoenix.md` | Revisión corregida post-fixes |
| **`foc-firmware/docs/SINTESIS_COMPARATIVA.md`** | **Este documento — síntesis comparativa global** |

---

## 13. Tabla Resumen de Diferencias

| Dimensión | Phoenix | moteus-c1 | Gap |
|-----------|---------|-----------|:---:|
| **Frecuencia FOC** | 20 kHz | 15-30 kHz | ✅ comparable |
| **PWM real** | ✅ 20 kHz (ARR=4250, center-aligned) | 30 kHz | 🟢 **corregido** |
| **Trig ADC** | TIM2 TRGO (2×/ciclo) | LPTIM1 (1×/ciclo) | ⚠️ subóptimo |
| **Sin/Cos** | LUT 1024 (±0.35°) | CORDIC HW (<0.01°) | ⚠️ menor precisión |
| **VBUS** | Hardcodeado 24.0V | ADC PA9 medido | 🔴 **crítico** |
| **Encoder** | AS5600 I2C + incr. | AS5047 SPI monolítico | ⚠️ más complejo |
| **Resolución encoder** | 4096 CPR (incr.) | 16384 CPR (absoluto) | ⚠️ 4× menor |
| **Comunicación** | SPI2 slave (punto a punto) | CAN-FD (multi-drop) | ❌ diferentes |
| **Gate IDRIVEP_HS** | 370 mA | 60 mA | ⚠️ 6× mayor |
| **Gate IDRIVEN_HS** | 740 mA | 20 mA | ⚠️ **37× mayor** |
| **VDS_LVL (OCP)** | 260 mV | 940 mV | ⚠️ 3.6× más sensible |
| **Calibración ADC** | Solo ADC1 | Todos | ⚠️ parcial |
| **Toolchain** | CubeMX + HAL C | Bazel + mbed C++ | ❌ diferentes |
| **Tests** | No | Sí (Bazel host tests) | ❌ ausente |
| **Protecciones** | Parciales (HW nFAULT) | Completas (HW+SW) | ⚠️ incompletas |
| **Calibración motor** | Esqueleto | Automática completa | ❌ pendiente |

---

## 14. Recomendaciones Priorizadas

### Inmediatas (Riesgo de funcionamiento incorrecto)

1. **🔴 TIM2 ARR** — Cambiar de 0xFFFFFFFF a 4250 (20 kHz PWM). Verificar que el código generado no sobrescriba.
2. **🔴 VBUS** — Implementar recepción de VBUS por SPI2 desde master. Mientras tanto, documentar el error esperado en el control.
3. **🟠 Ic a ADC3** — Reconfigurar PB1 como ADC3_IN12 para muestreo simultáneo de 3 fases.
4. **🟡 ADC2 cal** — Añadir calibración de ADC2 para eliminar offset en Ib.
5. **🟡 TIM1 TI12** — Cambiar a modo TI12 para duplicar resolución de encoder.

### Corto Plazo (Robustez)

6. **🟡 Gate currents** — Verificar FETs y ajustar IDRIVEP/IDRIVEN a valores seguros. Medir Vgs con osciloscopio.
7. **🟡 VDS_LVL** — Evaluar si 260 mV causa falsos OCP. Si es así, incrementar a 520-940 mV.
8. **🟡 Overcurrent SW** — Implementar comparación rápida en ISR con shutdown inmediato.
9. **🟡 Watchdog SPI2** — Detectar timeout de comunicación y entrar a STOPPED.

### Mediano Plazo (Rendimiento)

10. **🟡 CORDIC** — Habilitar CORDIC hardware en lugar de LUT trig para precisión completa.
11. **🟡 Calibración motor** — Implementar Rs/Ls/Ke measurement y offset encoder.
12. **🟡 Protecciones** — Temperatura FET, temperatura motor, UVLO, stall detection.
13. **🟢 SVM con VBUS real** — Una vez que VBUS llegue por SPI2, integrar en SVM.

---

## 15. Referencias

- **moteus-c1 pinout:** `fw/moteus_hw.cc:322-381` (familia 2)
- **moteus PWM config:** `fw/bldc_servo.cc` — `ConfigurePwmTimer()` (línea 458)
- **moteus DRV8323 config:** `fw/drv8323.h:236-311` — valores por defecto
- **moteus ADC:** `fw/stm32g4_adc.h`, `fw/stm32g4_adc.cc`
- **moteus FOC:** `fw/foc.cc`, `fw/bldc_servo_control.h` — `ISR_DoCurrent()`
- **moteus CORDIC:** `fw/bldc_servo_control.h:511` — `self().cordic(RadiansToQ31(theta))`
- **Phoenix hardware:** `foc-firmware/HARDWARE.md`
- **Phoenix firmware status:** `foc-firmware/docs/FIRMWARE_STATUS.md`
- **Phoenix reference vs moteus:** `foc-firmware/docs/MOTEUS_REFERENCE.md`
- **Phoenix discrepancias:** `foc-firmware/docs/DISCREPANCIAS.md`
- **Phoenix integración:** `foc-firmware/docs/INTEGRATION_REVIEW.md`
- **Datasheet DRV8323S:** SLVSF44 — register map §7.5
- **Datasheet STM32G431:** RM0440
- **Datasheet AS5600:** AMS-AS5600-DS000
