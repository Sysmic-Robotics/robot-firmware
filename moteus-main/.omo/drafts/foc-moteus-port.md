---
slug: foc-moteus-port
status: awaiting-approval
intent: clear
pending-action: write .omo/plans/foc-moteus-port.md
approach: Portar el firmware FOC de moteus a un PCB personalizado con STM32G431CBU6 + DRV8323S (SPI3) + AS5600 (I2C2), eliminando dependencias de CAN-FD y reemplazándolas por SPI2.
---

# Draft: foc-moteus-port

## Components (topology ledger)
| id | outcome (one line) | status | evidence path |
|---|---|---|---|
| c1 | FOC core (foc.h, pid.h, math.h) port | active | .omo/evidence/phase-1/ |
| c2 | DRV8323S driver (SPI3, subclase de MotorDriver) | active | .omo/evidence/phase-1/ |
| c3 | PWM+ADC config para G431 (TIM2 PWM, ADC1/2 TIM2_TRGO, SIN DMA) | active | .omo/evidence/phase-1/ |
| c4 | Raw FOC current loop (ISR 20kHz, Clarke/Park/SVPWM) | active | .omo/evidence/phase-1/ |
| c5 | AS5600 I2C2 driver | active | .omo/evidence/phase-2/ |
| c6 | Quadrature encoder timer (TIM1 encoder mode) | active | .omo/evidence/phase-2/ |
| c7 | MotorPosition hybrid source (I2C abs + quad inc) | active | .omo/evidence/phase-2/ |
| c8 | Position/velocity control FSM (BldcServoControl) | active | .omo/evidence/phase-3/ |
| c9 | SPI2 transport (reemplazo de CAN-FD, slave) | active | .omo/evidence/phase-4/ |
| c10 | Comms protocol (multiplex/micro_server adaptado) | active | .omo/evidence/phase-4/ |
| c11 | Firmware main processor multi-motor | deferred | — |

## Open assumptions (announced defaults)
| assumption | adopted default | rationale | reversible? |
|---|---|---|---|
| FOC loop rate | 20kHz (50µs) | Balance entre resolución de control y tiempo para I2C+SPI | Sí, configurable por timer |
| AS5600 I2C speed | 400kHz fast mode | Máximo estándar del AS5600 | Sí |
| PWM frequency | 20kHz (matching FOC loop) | Típico en FOC audibles | Sí |
| Dead time | 50ns (DRV8323S DTC en reg5, SPI3) | Configurado vía SPI, driver existente | Sí |
| SPI baud rate | 10MHz | Balance velocidad/integridad señal | Sí |
| Position PLL bandwidth | 100Hz | Filtrado para combinación I2C+quad | Sí |

## Findings (cited - path:lines)
- **foc.h** no tiene dependencias de hardware. 100% reutilizable. (fw/foc.h)
- **MotorDriver** clase abstracta con interfaz limpia. DRV8323 es referencia SPI — usamos el driver existente. (fw/motor_driver.h, fw/drv8323.h)
- **BldcServo** ISR pipeline: ISR_HandleTimer→ISR_DoTimer→ISR_DoSenseCritical + ISR_DoSense→ISR_CalculateCurrentState→ISR_DoControl→DoPwmControl. (fw/bldc_servo.cc)
- **MotorPosition** soporta kI2C + kQuadrature como fuentes combinables con PLL. (fw/motor_position.h)
- I2C a 400kHz ≈ 200-300µs por lectura → 6-9 ciclos FOC a 20kHz. Necesario híbrido I2C+quadrature.
- El protocolo CAN-FD usa multiplex/micro_server → capa serializable adaptable a SPI. (fw/fdcan.cc/h)
- Existen stm32_i2c.h y stm32_spi.h como abstracciones HAL.

## Decisions (with rationale)
1. **DRV8323S por SPI3**: El PCB usa DRV8323SRTAR con SPI (no DRV8328S GPIO). Ya tenemos el driver en Tools/drv8323.c/.h con todas las configuraciones SPI: gain, dead time, PWM 3X, CSA offset cal, HiZ pin. La configuración SPI3 ya existe en HARDWARE.md (PC10=SCK, PC11=MISO, PB5=MOSI, 10.625 MBd).
2. **Hybrid AS5600 + Quadrature**: I2C solo es demasiado lento para FOC loop. AS5600 da posición absoluta a ~200-300µs vía I2C, un timer en encoder mode da incrementos rápidos (~1µs). MotorPosition ya soporta esta arquitectura.
3. **SPI en vez de CAN-FD**: El STM32G431 no tiene CAN-FD (solo CAN 2.0 básico con 1 filtro). SPI+DMA es más flexible para multi-motor futuro. Se adapta el protocolo capa multiplex.
4. **TIM2 para PWM, TIM1 para encoder**: El PCB asigna TIM2_CH1-3 (PA0, PA1, PA2) como PWM 3-fases y TIM1_CH1-2 (PA8, PA9) como encoder quadrature. Esto es al revés de lo típico, pero funcional. TIM2 tiene 32-bit counter, suficiente para PWM 20kHz. TIM1 en encoder mode da posición incremental.
5. **ADC sincronizado por TIM2_TRGO (sin DMA)**: El PCB no tiene DMA configurado para ADC. Usamos TIM2_TRGO (update event) como trigger hardware de ADC1+ADC2, con muestreo simultáneo por TIM2. Sampling time ya configurado a 47.5 ciclos (vs 2.5 original) para cumplir con settle time del DRV8323 a gain=20. Sin DMA — las conversiones se leen en el ISR de TIM6 (FOC @ 20kHz) por polling rápido.

## Scope IN
- FOC current loop (Clarke, Park, inverse Park, SVPWM)
- DRV8323S SPI3 driver (Tools/drv8323.c existente, conectar SPI3)
- TIM2 PWM 3-fases + ADC1/2 TIM2_TRGO (sin DMA, sin LPTIM)
- AS5600 I2C2 + TIM1 Quadrature encoder
- VBUS sensing ADC2_CH5 (PA9)
- Position/velocity servo control
- SPI2 slave transport (reemplazo CAN-FD)
- Protocolo serialización (multiplex adaptado)
- Documentación hardware y periféricos

## Scope OUT (Must NOT have)
- NO CAN-FD (no hay soporte hw en G431)
- NO multi-motor main processor firmware (fase futura)
- NO USB protocol stack
- NO HTTP/WiFi/BLE
- NO GUI ni telemetry viewer
- NO RTOS (bare-metal, como moteus original)

## Open questions
- ~~¿Pines exactos de quadrature timer (TIM2, TIM3 o TIM4)?~~ → **TIM1 (PA8, PA9)** según HARDWARE.md. Resuelto.
- ~~¿SPI slave o multi-master?~~ → **SPI2 slave** según HARDWARE.md. Resuelto.
- ¿Formato de trama SPI? Propuesta: 8B header + N*8B payload (como CAN-FD pero sobre SPI). Pendiente decidir.
- ¿ADC CH12 (PB1, Ic) no está en secuencia regular — agregar o dejar como está? Actualmente solo leemos Ia (ADC1 CH4) + Ib (ADC2 CH3) para FOC.
- ¿TIM2_TRGO para ADC está bien a 20kHz o necesitamos trigger separado? Actualmente configurado y funcional.

## Approval gate
status: awaiting-approval

## Changelog
- **2026-07-07**: Corrección mayor de supuestos de hardware tras revisión del PCB real (HARDWARE.md). DRV8328S→DRV8323S SPI3, TIM1 PWM→TIM1 encoder+ TIM2 PWM, DMA+LPTIM→TIM2_TRGO sin DMA, SPI1→SPI2 slave. Plan y draft actualizados.
