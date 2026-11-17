# Plan de Avance — Firmware Phoenix (STM32G431)

> Basado en: moteus-c1 (familia 2) + modificaciones phoenix
> Estado firmware: FOC loop @20kHz + servo control cascade (PID position/velocity/current) + SPI2 comunicación
> Versión plan: 1.1

> ⚠️ **CRÍTICO — TIM2 ARR:** El auto-reload de TIM2 está en 0xFFFFFFFF. El PWM NO está commutando a frecuencia real. Ver Fase 0.

---

## Log de Cambios

### 2026-07-08 — Corrección Crítica: TIM2 ARR identificado
- ARR=0xFFFFFFFF produce PWM cuasi-DC (~0.02 Hz, período ~50 s)
- Pendiente cambiar ARR a ~4250 (20 kHz) para PWM real

### 2026-07-08 — Decisión: PWM a 20 kHz
- Elegido **20 kHz (ARR=4250)** para coherencia con FOC rate de TIM6
- `.ioc` actualizado a `TIM2.PeriodNoDither=4250`
- Síntesis comparativa con moteus-c1 completada en SINTESIS_COMPARATIVA.md

### 2026-07-07 — FOC Loop Implementado
- FOC loop movido de main loop (1 kHz) a ISR de TIM6 @ 20 kHz
- ADC configurado con HW trigger (TIM2_TRGO) y sampling time 47.5 ciclos
- VBUS sensing implementado en ADC2_CH5 (PA9), reemplazando hardcode 24.0 V
- Lookup table de seno/coseno (1024 entries) reemplaza sinf/cosf de math.h
- DRV8323 CSA offset calibration implementado en secuencia de enable
- Control HiZ pin (PC15) para power-on/off seguro

### 2026-07-08 — Servo Control + SPI Integration
- Port completo de PID.h desde moteus (PID + SimplePI + ExponentialFilter)
- ServoControl state machine (servo_control.h/.c): modos STOPPED, BRAKE, CURRENT, VELOCITY, POSITION, ZERO_VELOCITY
- Cascade de control: position PID → velocity PID → current limits, ejecutado en TIM6 ISR @20kHz
- `FOC_RunWithRefs()` añadido para aceptar Id/Iq ref del servo control en vez de usar refs internas
- SPI2 handler traduce setpoints del master → ServoCommandData → ServoControl_SetCommand
- Telemetría SPI2 ahora reporta servo status (velocidad, Iq, fault, state)
- PID gains configurables en ServoConfig, escalables por comando SPI (kp_scale/kd_scale/ilimit_scale)

### 2026-07-08 — EXTI PB13 nFAULT (HW fault detection)
- PB13 (nFAULT) configurado como EXTI falling edge con pull-up en vez de GPIO input polling
- EXTI15_10_IRQHandler implementado: desactiva TIM2 PWM inmediatamente (clear MOE), setea flag `g_drv8323_hw_fault`
- NVIC prioridad 0 (más alta que TIM6 prio 1 y SPI2 prio 2)
- Main loop fault handler actualizado: usa HW flag, setea ServoFault en servo control, auto-retry con debounce
- Matching moteus: moteus revisa `fault()` al inicio de cada ISR — acá EXTI provee respuesta inmediata, complementado por polling en loop

---

## Fase 0 — Corrección Crítica: TIM2 ARR (PWM)

### Objetivo
Corregir el auto-reload de TIM2 para obtener PWM switching real a 20 kHz (center-aligned).

**Contexto:** TIM2 está configurado con ARR=0xFFFFFFFF en center-aligned mode 2. Con TIM2CLK=170 MHz, la frecuencia PWM es f_pwm = 170 MHz / (2 × 0xFFFFFFFF) ≈ 0.02 Hz (período de ~50 segundos). Las actualizaciones de duty cycle desde TIM6 ISR (20 kHz) modifican CCR1/CCR2/CCR3, pero el timer base no commuta a velocidad PWM. El motor puede funcionar en modo cuasi-DC con eficiencia reducida, alto rizado de corriente y riesgo de sobrecalentamiento.

**Referencia:** MOTEUS_REFERENCE.md sección 1. moteus-c1 usa `pwm_rate_hz=30000` → ARR=2833 (30 kHz).

### Tasks

#### 0.1 Calcular ARR correcto
- Decisión: usar **20 kHz** (coherente con FOC rate de TIM6)
- ARR = 170 MHz / (2 × 20000) = **4250**

#### 0.2 Modificar ARR en `Core/Src/tim.c`
- Localizar `MX_TIM2_Init()` y cambiar `htim2.Init.Period = 0xFFFFFFFF` → `4250`
- Mover a sección `USER CODE BEGIN TIM2_INIT` para que CubeMX no sobrescriba

#### 0.3 Actualizar `cubemx/FOCTest.ioc`
- Abrir CubeMX, confirmar TIM2 Period=4250 (ya actualizado en `.ioc`), regenerar código
- Verificar que USER CODE sections se mantengan intactas

#### 0.4 Revisar trigger de ADC (TIM2_TRGO)
- Con ARR=4250, el update event de TIM2 ocurre a ~20 kHz (coincide con FOC rate de TIM6)
- Verificar que el trigger de ADC (TIM2_TRGO) sigue siendo el correcto
- Ajustar sampling time si es necesario para CSA settling time

#### 0.5 Ajustar clamp de duty cycle
- Revisar duty cycle mínimo/máximo con el nuevo ARR
- Asegurar tiempo muerto suficiente para half-bridges

### Dependencias
Ninguna (es independiente de otras fases).

### Esfuerzo estimado
1 día.

---

## Fase 1 — Comunicación SPI2 Slave con Placa Master

### Objetivo
Establecer canal de comunicación bidireccional con la placa master mediante SPI2 (slave, 16-bit) para recibir setpoints y enviar telemetría.

### Tasks

#### 1.1 Definir protocolo
- **Formato**: Trama de N palabras de 16 bits
- **Header**: 1 palabra — identificador de comando + longitud
- **Payload**: setpoints (posición, velocidad, torque, modo)
- **Telemetría**: datos devueltos en la misma transacción SPI
- **Frecuencia**: esperada 1-10 kHz (depende de master)
- **Referencia**: protocolo multiplex de moteus como inspiración

Protocolo sugerido (mínimo viable):

```
TX desde master → Phoenix (cada transacción SPI):
  Word 0:  [CMD_ID:4]  [RESERVED:4]  [MODE:8]
  Word 1:  setpoint posición (int16_t escalado)
  Word 2:  setpoint velocidad (int16_t escalado)
  Word 3:  setpoint torque (int16_t escalado)
  Word 4:  checksum/CRC opcional

RX desde Phoenix → Master (respuesta simultánea):
  Word 0:  [STATUS:8]  [FAULT:8]
  Word 1:  posición actual (int16_t escalado)
  Word 2:  velocidad actual (int16_t escalado)
  Word 3:  corriente Iq (int16_t escalado)
  Word 4:  tensión bus (uint16_t)
```

#### 1.2 Implementar driver SPI slave
- Habilitar interrupción SPI2 en `stm32g4xx_it.c` (`SPI2_IRQHandler`)
- Usar HAL_SPI_Receive_IT / HAL_SPI_Transmit_IT para RX/TX
- Buffer circular doble para no perder tramas
- Implementar callback `HAL_SPI_TxRxCpltCallback` para procesar trama completa
- Timeout / watchdog: si no hay transacción en X ms, entrar en safe state

#### 1.3 Procesamiento de comandos
- Parsear header + payload
- Actualizar setpoints del controlador (posición/velocidad/torque)
- Responder con telemetría del estado actual
- Implementar comando de emergencia (STOP / disable driver)

#### 1.4 Testing
- Conectar a master simulado (otra STM32 o Raspberry Pi con SPI)
- Verificar integridad de datos con patrones conocidos
- Medir latencia y throughput
- Verificar recovery tras error de comunicación

---

## Fase 2 — FOC Control Loop ✅ COMPLETED

### Objetivo
Implementar Field Oriented Control para motor BLDC con encoder incremental, ejecutándose desde una interrupción de timer a 10-40 kHz.

### Tasks (implementado — mantenido como referencia)

#### 2.1 Timer ISR para FOC
- Configurar **TIM6** o **TIM7** (timers básicos, sin pines externos) para generar interrupción periódica
- Periodo: 25-100 µs (10-40 kHz)
- En ISR: leer ADC → encoder → FOC → actualizar PWM
- Prioridad: más alta que SPI, más baja que SysTick

#### 2.2 Lectura sincronizada de ADC
- TIM1_TRGO (encoder) o TIM2_TRGO (PWM) como trigger para ADC1/ADC2
- Leer ADC1: Ia (PA3), Ic (PB1) — muestreo simultáneo con inyectado
- Leer ADC2: Ib (PA6)
- Convertir raw ADC a corriente en amperios: `I = (raw - offset) * Vref / (4096 * Rshunt * gain)`
- Calibrar offset al iniciar (muestreo con motor en reposo, driver en high-Z)

#### 2.3 Clarke Transform (Ia, Ib, Ic → Iα, Iβ)
```c
Iα = Ia
Iβ = (Ia + 2*Ib) / √3   // asumiendo Ia + Ib + Ic = 0
```

#### 2.4 Park Transform (Iα, Iβ → Id, Iq)
```c
// θ_e = ángulo eléctrico del encoder
Id =  Iα * cos(θ_e) + Iβ * sin(θ_e)
Iq = -Iα * sin(θ_e) + Iβ * cos(θ_e)
```
- Usar tabla lookup o `arm_sin_f32`/`arm_cos_f32` de CMSIS-DSP
- θ_e de `Encoder_GetElectricalAngle()`

#### 2.5 PI Controllers (Id, Iq)
```
Vd_error = Id_ref - Id
Vd += Kp_d * Vd_error + Ki_d * Vd_error * dt
(similar para Iq)
```
- Anti-windup condicional
- Límites de saturación de tensión (Vd² + Vq² ≤ Vmax²)
- Desacople: Vd_ff = -ω_e * Lq * Iq, Vq_ff = ω_e * (Ld * Id + λ_pm)
- **Referencia**: parámetros de motor (Ld, Lq, Rs, λpm) por configurar

#### 2.6 Inverse Park (Vd, Vq → Vα, Vβ)
```
Vα = Vd * cos(θ_e) - Vq * sin(θ_e)
Vβ = Vd * sin(θ_e) + Vq * cos(θ_e)
```

#### 2.7 Space Vector Modulation (SVM)
- Vα, Vβ → duty ratios Ta, Tb, Tc para los 3 half-bridges
- Implementar SVPWM (centrado, min/max injection) para mejor DC bus utilization
- Actualizar TIM2 CCR1/CCR2/CCR3 con los duty cycles
- Aplicar dead-time compensation si es necesario

#### 2.8 Secuencia de arranque
1. Driver enable + esperar charge pump (1 ms)
2. Corriente cero (Vd = Vq = 0), leer offset ADC
3. Alineación: aplicar corriente en eje D fijo por 50 ms
4. Transición suave a control conmutado (θ_e tracking)
5. Rampa de velocidad o posición controlada

#### 2.9 Feedforward BEMF
- Compensar tensión contraelectromotriz: Vq_ff = ω_e * λ_pm
- Mejora respuesta transitoria

---

## Fase 3 — Control de Alto Nivel

### Objetivo
Implementar modos de control: posición, velocidad, torque, corriente.

### Tasks

#### 3.1 Modo Torque
- Setpoint Iq_ref directamente
- Id_ref = 0 (o MTPA si aplica)
- Límite: `sqrt(Iq² + Id²) ≤ Imax`

#### 3.2 Modo Velocidad
- PI externo: `ω_ref - ω_meas → Iq_ref`
- Anti-windup + límite de corriente
- Rama de feedforward: aceleración * inercia

#### 3.3 Modo Posición
- P cascada: `θ_ref - θ_meas → ω_ref`
- O PID: posición → torque directo (PIV)
- Perfil trapezoidal o S-curve para movimientos suaves

#### 3.4 Modo Corriente Directa
- Modo debug/calibración: aplicar Vd, Vq directamente

---

## Fase 4 — Calibración y Protecciones

### Objetivo
Calibrar parámetros del motor y proteger el hardware.

### Tasks

#### 4.1 Calibración offset ADC
- Motor detenido, high-Z, promediar 1000+ muestras
- Almacenar offset por canal

#### 4.2 Medición Rs (resistencia de fase)
- Aplicar Vd fijo, medir Id estabilizado
- Rs = Vd / Id

#### 4.3 Medición Ld, Lq (inductancia)
- Método de inyección de tensión con respuesta de corriente
- O método de decaimiento (L/R)

#### 4.4 Calibración offset encoder (θ_offset)
- Alinear eje D con fase A aplicando corriente y observando posición
- `θ_e_real = θ_e_raw - θ_offset`
- Almacenar en memoria no volátil

#### 4.5 Protecciones
- **Overcurrent**: si |I| > Imax, disable driver inmediatamente
- **Overvoltage**: si Vbus > Vmax, activar frenado dinámico
- **Undervoltage**: si Vbus < Vmin, disable driver
- **Overtemperature**: si T > Tmax, reducir corriente
- **Stall detection**: si ω_meas ≈ 0 con Iq alto por mucho tiempo
- **Encoder fault**: si delta encoder es errático o nulo con corriente alta

---

## Fase 5 — Optimización y Refinamiento

### Objetivo
Mejorar rendimiento y robustez.

### Tasks

#### 5.1 Compensación de conmutación
- Tabla lookup de corrección de ángulo eléctrico vs corriente
- Compensa no-linealidades del inversor (dead-time, VCEsat)

#### 5.2 MTPA (Maximum Torque Per Ampere)
- Para motores IPMSM: inyectar Id negativo para aprovechar reluctance torque

#### 5.3 Flux Weakening
- Por encima de velocidad base: reducir Id para extender rango de velocidad

#### 5.4 Observador de posición (sensorless)
- Observador Luenberger o Sliding Mode para respaldo si encoder falla

#### 5.5 Filtro PLL en encoder
- Mejorar calidad de velocidad a baja velocidad (M/T method)

---

## Hitos y Prioridades

| Hito | Dependencias | Esfuerzo estimado |
|---|---|---|
| **P0** Corrección TIM2 ARR (PWM) | — | 1 día |
| **P1** SPI2 funcional (comandos básicos) | — | 2-3 días |
| **P2** FOC loop en ISR (corriente) | P1 (para monitoreo) | 4-6 días |
| **P3** Control de velocidad | P2 | 1-2 días |
| **P4** Control de posición | P3 | 1 día |
| **P5** Calibración automática | P2 | 2-3 días |
| **P6** Protecciones | P2 | 1 día |
| **P7** Comunicación completa | P1 + P2 | 2 días |

**Total estimado**: 14-19 días hábiles para MVP funcional con FOC + comunicación.

---

## Referencias

- **moteus-c1 pinout**: `/home/sysmic/Desktop/SysmicProyect/robot/moteus-main/fw/moteus_hw.cc` (family 2)
- **moteus FOC**: `/home/sysmic/Desktop/SysmicProyect/robot/moteus-main/fw/foc.cc`
- **moteus BldcServo**: `/home/sysmic/Desktop/SysmicProyect/robot/moteus-main/fw/bldc_servo.h`
- **moteus encoder**: `/home/sysmic/Desktop/SysmicProyect/robot/moteus-main/fw/aux_mbed.h` (Stm32Quadrature)
- **Config IOC**: `/home/sysmic/Desktop/SysmicProyect/robot/moteus-main/foc-firmware/cubemx/FOCTest.ioc`
- **Documentación estado**: `docs/FIRMWARE_STATUS.md`
- **Pinout/HW (fuente de verdad)**: `HARDWARE.md`
- **Corrección de discrepancias**: `docs/DISCREPANCIAS.md`

> ⚠️ **I2C2:** Pines correctos según CubeMX = **PC4 (SCL), PF0 (SDA)**. Ver `DISCREPANCIAS.md` DC-1.
> ⚠️ **VBUS:** PA9 es TIM1_CH2 (encoder). No puede usarse como ADC2_CH5. Pendiente recibir VBUS por SPI2. Ver `DISCREPANCIAS.md` DC-2.
