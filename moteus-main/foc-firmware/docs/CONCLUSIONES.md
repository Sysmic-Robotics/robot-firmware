# Proyecto Phoenix — Conclusiones y Estado del Sistema

> **Sistema**: Controlador de motor brushless FOC (Field-Oriented Control)
> **MCU**: STM32G431CBU6 @ 170 MHz
> **Placa**: Phoenix (basada en moteus-c1, modificada)
> **Fecha**: 2026-07-07

---

## Resumen Ejecutivo

El firmware Phoenix implementa un controlador de motor brushless con FOC (Field-Oriented Control) completo sobre un STM32G431CBU6, operando a 170 MHz con aceleración CORDIC hardware. El sistema incluye sensado de corriente en 3 fases, encoder incremental + absoluto (AS5600), control PID en cascada (posición → velocidad → corriente), y comunicación SPI2 con una placa master externa.

**Estado general: FUNCIONAL — FOC loop operativo a 20 kHz, control PID cascade completo, SPI2 protocol operativo, protegido contra faults del gate driver.**

---

## Arquitectura del Sistema

### Componentes del Lazo de Control

```
SPI2 (host) ──► ServoControl ──► PID/PI cascade ──► FOC (Clarke/Park/PI/InvPark/SVM) ──► TIM2 PWM ──► DRV8323 ──► Motor
   ▲                       (state machine)                    ▲                               │
   │                                                            │                               │
   └────── Telemetría (posición, vel, corriente) ◄──────────────┘ ◄────── TIM6 ISR @ 20 kHz ◄──┘
                                                                          │
                                                                    TIM1 Encoder (posición)
                                                                    ADC1+ADC2 (Ia, Ib, Ic)
                                                                    AS5600 (absoluto, startup)
```

### Flujo de Inicialización

```
Power On
  │
  ├─ HAL_Init / SystemClock_Config (170 MHz)
  ├─ MX_GPIO_Init (pines + EXTI PB13 nFAULT)
  ├─ MX_ADC1_Init (master, CH4+CH12)
  ├─ MX_ADC2_Init (slave, CH3)
  ├─ MX_I2C2_Init (AS5600)
  ├─ MX_SPI2_Init (slave, host comm)
  ├─ MX_SPI3_Init (master, DRV8323)
  ├─ MX_TIM1_Init (encoder)
  ├─ MX_TIM2_Init (PWM)
  ├─ MX_CORDIC_Init
  │
  ├─ AS5600_Init → DRV8323_Init → Encoder_Init (seed posición absoluta)
  ├─ FOC_Init (TIM6 @ 20kHz + TIM2 PWM)
  ├─ COMM_SPI_Init (slave IRQ)
  │
  └─ Main loop (fault monitoreo + telemetría)
```

---

## Lo Construido

### 1. FOC Loop Completo (20 kHz)
- **Clarke transform**: Ia, Ib, Ic → Iα, Iβ
- **Park transform**: Iα, Iβ → Id, Iq (con ángulo eléctrico del encoder)
- **PI Id/Iq**: control de corriente con feedforward (decoupling + BEMF)
- **Inverse Park**: Vd, Vq → Vα, Vβ
- **SVM (Space Vector Modulation)**: implementación min-max con duty cycles normalizados
- **Timing**: ~8 µs/ciclo en ISR de TIM6 (20 kHz)
- **Trigonometría**: LUT de 1024 entradas sin/cos; CORDIC inicializado

### 2. Control PID en Cascada
- **PID posición**: Kp=200.0, Ki=0.0, Kd=0.0 (sintonizable vía defines)
- **PI velocidad**: Kp=2.0, Ki=0.5, límites configurables
- **PI corriente (Id/Iq)**: Kp=0.5, Ki=0.1, límite 20 A
- Anti-windup en todos los integradores
- Modos: POSITION, VELOCITY, CURRENT, ZERO_VEL, BRAKE, STOPPED

### 3. ServoControl State Machine
- 7 estados: STOPPED, BRAKE, CURRENT, VELOCITY, POSITION, ZERO_VEL, CALIBRATING
- Transiciones robustas entre modos
- Manejo de faults con auto-retry (hasta 3 intentos)
- Watchdog interno de velocidad cero

### 4. Sensado de Posición
- **Encoder incremental** (TIM1, modo TI12): 1024 CPR → 4096 pulsos/rev (4x)
- **Encoder absoluto** (AS5600, I2C2): 12-bit, 0.5° resolución, usado al startup para seed
- Velocidad con LPF exponencial (alpha configurable, default 0.85)
- Ángulo eléctrico: `posición_encoder * pole_pairs % 2π`

### 5. Sensado de Corriente
- ADC1 en modo master dual con ADC2, T2_TRGO trigger
- 3 fases: Ia (PA3 CH4), Ib (PA6 CH3), Ic (PB1 CH12)
- Sampling time: 47.5 ciclos
- Conversión por inyección (HAL_ADCEx_InjectedStart)
- Escala de corriente: raw ADC (12-bit) → amperios (con ganancia CSA 40 V/V)

### 6. Gate Driver DRV8323
- SPI3 a 5.3125 MBit/s, modo 3X PWM
- CSA gain 40 V/V, dead time 50 ns, OCP retry 4 ms
- Calibración CSA offset (CAL_A/B/C)
- Fault detection por EXTI (PB13, falling-edge) + polling periódico
- Auto-retry con reconexión SPI (hasta 3 intentos)
- Control HiZ para power-on/off seguro

### 7. Comunicación SPI2 con Host
- Protocolo 5 palabras × 16-bit (80 bits por frame)
- IRQ-driven con doble buffer TX/RX
- Comandos: STOP, SETPOINT_POS/VEL/TORQUE, CALIBRATE, READ_STATUS
- Telemetría en respuesta: posición, velocidad, Iq, status/faults

### 8. Documentación
- HARDWARE.md: configuración de pines, periféricos, clock tree (actualizado al código)
- FIRMWARE_STATUS.md: estado detallado de cada componente
- DISCREPANCIAS.md: discrepancias entre documentación y código
- PLAN_AVANCE.md: plan de trabajo

---

## Problemas Conocidos y Riesgos

### ⚠️ Crítico: ARR de TIM2 en 0xFFFFFFFF
**El auto-reload de TIM2 está configurado en 0xFFFFFFFF (máximo 32-bit).** Con center-aligned mode, la frecuencia PWM resultante es ~0.02 Hz — extremadamente baja. Aunque los duty cycles se actualizan a 20 kHz desde la ISR de TIM6, las transiciones físicas de PWM ocurren en una escala de tiempo de segundos, no microsegundos.

**Impacto**: El inversor opera en modo cuasi-DC en lugar de PWM switching real. El motor puede funcionar pero con eficiencia reducida, mayor rizado de corriente, y potencial sobrecalentamiento de los FETs.

**Solución propuesta**: Cambiar TIM2.Period a 4250 (→ 20 kHz PWM en center-aligned) o 8500 (→ 10 kHz). Verificar que la generación de código de CubeMX no sobrescriba este cambio, o moverlo a USER CODE.

### ⚠️ VBUS Hardcodeado
La tensión del bus DC está fijada en 24.0V en el código. PA9 (que sería ADC2_CH5 para VBUS) está ocupado por TIM1_CH2 (encoder). Sin VBUS real, el control de corriente no puede compensar variaciones de tensión de bus (feedforward necesario para operación precisa a alta velocidad).

### ⚠️ ADC2 Dual-Rank Redundante
ADC2 tiene ambas conversiones regulares en CH3 (PA6/Ib). Solo se usa la segunda (`InjectionData[1]`). Sin impacto funcional pero consumo de ciclos ADC innecesario.

### ⚠️ Protecciones Incompletas
- ❌ Overcurrent software (falta comparación rápida en ISR)
- ❌ Overtemperature (sin sensor)
- ❌ UVLO (sin VBUS real)
- ❌ Stall detection
- ❌ Desaturation protection
- ❌ Watchdog de comunicación SPI2

### ⚠️ Calibración Pendiente
- Rs/Ls (resistencia/inductancia de fase) no medidas
- Ke (constante BEMF) no identificada
- Offset de conmutación del encoder no calibrado
- CALIBRATING state existe pero la secuencia no está implementada

---

## Recomendaciones

### Prioridad 1 (Funcionamiento Básico del Motor)
1. **Corregir TIM2 ARR** — Poner Period = 4250 para PWM real a 20 kHz
2. **Implementar calibración** — Offset de encoder (alineación), corriente offset, Rs/Ls
3. **Agregar VBUS real** — Reasignar entrada ADC o recibir por SPI2 desde master

### Prioridad 2 (Robustez)
4. **Overcurrent software** — Comparación rápida en ISR con shutdown inmediato
5. **Watchdog SPI2** — Detectar timeout de comunicación y entrar a STOPPED
6. **Protección UVLO** — Monitorear VBUS y detener si cae

### Prioridad 3 (Rendimiento)
7. **Habilitar CORDIC** — Reemplazar LUT trig por CORDIC hardware para precisión completa
8. **DMA para ADC** — Reemplazar polling de ADC por lectura DMA para reducir latencia en ISR
9. **DMA para SPI2** — Transferencias DMA para frames largos de telemetría
10. **SVM con inyección de armónico** — Técnicas de utilización de bus mejorada

---

## Estadísticas del Proyecto

| Métrica | Valor |
|---|---|
| Archivos fuente (Core) | 12 (main, adc, cordic, dma, gpio, i2c, spi, tim, comm_spi, stm32g4xx_hal_msp, stm32g4xx_it, system_stm32g4xx) |
| Archivos Tools | 8 (as5600, drv8323, encoder, foc, pid, servo_control, trig_lut, comm_spi) + headers |
| LOC estimado (Tools) | ~1500 líneas |
| LOC estimado (Core) | ~2500 líneas (incluyendo HAL generado) |
| Periféricos activos | 10 (TIM1, TIM2, TIM6, ADC1, ADC2, SPI2, SPI3, I2C2, CORDIC, GPIO+EXTI) |
| Frecuencia FOC | 20,000 Hz (50 µs/ciclo) |
| Ciclo FOC | ~8 µs (16% de CPU en ISR) |
| Resolución PWM teórica | 32-bit (limitada por ARR real) |
| Resolución ADC | 12-bit |
| Comunicación host | SPI2 slave, 16-bit, full-duplex, 80-bit frames |
| Gate driver | DRV8323S, 3X PWM mode, 5.3 MBit/s SPI |

---

## Referencias

- HARDWARE.md — Configuración detallada de hardware
- FIRMWARE_STATUS.md — Estado de cada componente del firmware
- DISCREPANCIAS.md — Discrepancias detectadas
- Datasheet STM32G431 (RM0440)
- Datasheet DRV8323S (SLVSF44)
- Datasheet AS5600
- Código fuente: `Core/` (HAL + aplicación), `Tools/` (drivers + algoritmos)
- Proyecto CubeMX: `cubemx/FOCTest.ioc`
- Inspirado en: moteus (mjbots) — servo brushless de código abierto
