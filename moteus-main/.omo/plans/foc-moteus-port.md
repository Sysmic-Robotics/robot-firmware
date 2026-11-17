# foc-moteus-port - Work Plan

## TL;DR (For humans)

**What you'll get:** Un firmware FOC (Field-Oriented Control) para controlar motores brushless sin escobillas usando un STM32G431CBU6 con driver DRV8323S (SPI3) y encoder magnético AS5600 (I2C2). El sistema controla corriente, velocidad y posición del motor a 20kHz, y se comunica por SPI2 slave en lugar de CAN-FD.

**Why this approach:** Reutilizamos el núcleo FOC maduro de moteus (miles de horas de producción) en vez de escribirlo desde cero. El PCB ya asigna TIM2_CH1-3 como PWM 3-fases, TIM1 como encoder, ADC1/2 con trigger por TIM2_TRGO (sin DMA), y DRV8323S por SPI3. Reutilizamos el driver DRV8323 existente (Tools/drv8323.c). El AS5600 va por I2C2 y la comunicación externa por SPI2 slave.

**What it will NOT do:** No usará CAN-FD (el G431 no lo soporta). No tendrá RTOS. No soportará múltiples motores desde un solo G431 (eso vendrá en una fase futura con un procesador principal). No tendrá interfaz USB, WiFi, ni GUI.

**Effort:** Large — 5 fases, cada fase es ~1-2 semanas de trabajo
**Risk:** Medium — El FOC math está probado, pero la integración HW específica (DRV8323S SPI3, AS5600 I2C2, SPI2 protocol) requiere validación
**Decisions to sanity-check:** DRV8323S SPI3 config (gain=20, PWM 3X, dead-time 50ns), hybrid AS5600+quadrature (I2C2 lento + TIM1 encoder rápido), SPI2 slave en vez de CAN-FD

Your next move: Aprobar el plan para empezar la Fase 1 (port del FOC core). Full execution detail follows below.

---

> TL;DR (machine): Large effort, Medium risk. Port moteus FOC firmware to STM32G431CBU6 + DRV8323S (SPI3) + AS5600 (I2C2). Hardware: TIM2 PWM, TIM1 encoder, ADC1/2 TIM2_TRGO, SPI3 DRV8323, SPI2 slave comms. 5 phases: (1) FOC core + PWM/ADC, (2) Encoder hybrid, (3) Position control, (4) SPI2 transport, (5) Multi-motor main proc.

## Scope
### Must have
- FOC current loop (Clarke/Park/SVPWM) a 20kHz
- DRV8323S gate driver control (SPI3: PC10=SCK, PC11=MISO, PB5=MOSI — 10.625 MBd)
- PWM 3-phase generation (TIM2_CH1/2/3: PA0/PA1/PA2, PWM1 mode)
- ADC 2-shunt current sensing (ADC1 CH4=Ia PA3, ADC2 CH3=Ib PA6, TIM2_TRGO trigger, sin DMA, 47.5 ciclos sampling)
- VBUS sensing (ADC2 CH5 PA9)
- AS5600 absolute position via I2C2 (400kHz, SCL=PC4, SDA=PF0)
- Quadrature encoder incremental position (TIM1 encoder mode, PA8/PA9)
- Hybrid position: AS5600 I2C2 (abs, slow) + TIM1 quadrature (inc, fast) via PLL
- Position/velocity PID control (BldcServoControl FSM)
- SPI2 slave transport replacing CAN-FD (PF1=SCK, PB14=MISO, PA11=MOSI, 16-bit)
- Protocol serialization (multiplex frame format over SPI)
- Hardware documentation (HARDWARE.md, PERIPHERALS.md)
- Peripheral configuration for CubeMX generation

### Must NOT have (guardrails, anti-slop, scope boundaries)
- NO CAN-FD (no hay soporte hw en G431)
- NO multi-motor main processor firmware (fase futura)
- NO USB protocol stack
- NO HTTP/WiFi/BLE
- NO GUI ni telemetry viewer (tview)
- NO RTOS (bare-metal, como moteus original)
- NO HAL overhead en ISR (register directo en runtime)
- NO cambios al código moteus original (todo nuevo en foc-firmware/)
- NO dependencias externas (solo CMSIS + CubeMX HAL init)
- NO DMA para ADC (el PCB no tiene configurado)
- NO LPTIM1 (trigger es TIM2_TRGO directo)
- NO CORDIC para trigonometría (usamos LUT — ya implementado en trig_lut.h)
- NO DRV8328S (el PCB usa DRV8323S por SPI3)

## Verification strategy
> Zero human intervention - all verification is agent-executed.
- Test decision: tests-after (tests por fase, compilación + lógica + simulación)
- Evidence: .omo/evidence/task-<N>-foc-moteus-port.<ext>
- Tools: gcc-arm-none-eabi, pyelftools (hex analysis), Python (simulación FOC math)

## Execution strategy
### Parallel execution waves
Wave 1 (Foundation): FOC math port + DRV8323S SPI3 driver (ya existe, integrar) + PWM/ADC HAL (TIM2, ADC1/2 TIM2_TRGO)
Wave 2 (Current Loop): Full current sensing ISR pipeline + Clarke/Park/SVPWM (TIM6 ISR @20kHz, ya funcional)
Wave 3 (Encoder): AS5600 I2C2 + Quadrature TIM1 + MotorPosition hybrid
Wave 4 (Position Control): BldcServoControl FSM + position/velocity PID
Wave 5 (Communication): SPI2 slave transport + protocol adaptation

### Dependency matrix
| Todo | Depends on | Blocks | Can parallelize with |
| --- | --- | --- | --- |
| W1-D1: FOC math port | — | W1-D3 | W1-D2 |
| W1-D2: DRV8323S SPI3 driver integration | — | W2 | W1-D1 |
| W1-D3: TIM2 PWM + ADC TIM2_TRGO HAL config | W1-D1 | W2 | W1-D2 |
| W2: Current loop ISR | W1-D1, W1-D2, W1-D3 | W3 | — |
| W3-A: AS5600 I2C | — | W3-C | W3-B |
| W3-B: Quadrature TIM1 (PA8/PA9 encoder mode) | — | W3-C | W3-A |
| W3-C: MotorPosition hybrid | W3-A, W3-B | W4 | — |
| W4: Position control FSM | W2, W3-C | W5 | — |
| W5-A: SPI2 slave transport (PF1=SCK, PB14=MISO, PA11=MOSI) | — | W5-B | — |
| W5-B: Protocol adaptation | W5-A | Final | — |

## Todos
> Implementation + Test = ONE todo. Never separate.

- [x] 1. [HW-DOC] Hardware documentation (HARDWARE.md + PERIPHERALS.md)
  What to do / Must NOT do: Create exhaustive hardware documentation and CubeMX peripheral configuration guides
  Parallelization: Wave 0 | Blocked by: — | Blocks: W1
  References: Hardware specs: STM32G431CBU6 DS12500, DRV8323S datasheet, AS5600 datasheet, foc-firmware/HARDWARE.md
  Acceptance criteria (agent-executable): Files exist at foc-firmware/HARDWARE.md and foc-firmware/PERIPHERALS.md, both >100 lines
  QA scenarios: READ files and verify all pin mappings, peripheral settings match datasheet specs
  Commit: Y | docs(hardware): hardware documentation and CubeMX peripheral configuration

- [ ] 2. [W1-D1] Port FOC math library (foc.h, pid.h, math_util.h, etc.)
  What to do / Must NOT do: Copy the pure-math FOC files from moteus fw/ into foc-firmware/lib/foc/. These files have ZERO hardware dependencies and can compile stand-alone. Do NOT copy hardware-dependent files. Create a test harness that exercises Clarke/Park/SVPWM with known inputs.
  Parallelization: Wave 1 | Blocked by: — | Blocks: W1-D3, W2
  References: fw/foc.h, fw/pid.h, fw/math_util.h, fw/bldc_servo_structs.h, fw/math/ in moteus tree
  Acceptance criteria: foc-firmware/lib/foc/*.h compiles stand-alone with gcc-arm-none-eabi. Test harness verifies: Clarke(ia,ib) → (alpha,beta), Park(alpha,beta,theta) → (d,q), SVPWM(d,q,theta) → duty_U,V,W
  QA scenarios: happy: compile + run test harness with known-good values, compare to python numpy reference
  Commit: Y | feat(foc): port moteus FOC math library (Clarke/Park/SVPWM/PID)

- [ ] 3. [W1-D2] Integrate DRV8323S SPI3 driver
  What to do / Must NOT do: Conectar el driver DRV8323 existente (Tools/drv8323.c/.h) al SPI3 del PCB (PC10=SCK, PC11=MISO, PB5=MOSI, 10.625 MBd). Verificar config: gain=20V/V, VREF/2 bidirectional, PWM 3X, dead-time 50ns, CSA cal en Enable(), HiZ pin control (PC15). El driver YA existe y está verificado — solo falta cablear SPI3 handle y pinout. NO reescribir el driver, NO cambiar la API DRV8323_Init/Enable/Disable. NO asumir DRV8328S (no existe en este PCB).
  Parallelization: Wave 1 | Blocked by: — | Blocks: W2
  References: Tools/drv8323.h/.c, Tools/drv8323_regs.h, foc-firmware/HARDWARE.md (SPI3 section), fw/motor_driver.h
  Acceptance criteria: DRV8323_Init() usa SPI3 (hspi3). Compila. ReadReg/WriteReg funcionan sobre SPI3. CSA cal + HiZ sequence intactos.
  QA scenarios: happy: compile + link + verify SPI3 pinout en map file. failure: SPI3 handle NULL → error
  Commit: Y | feat(driver): integrate DRV8323S SPI3 driver (existing) with PCB pinout

- [ ] 4. [W1-D3] Configure TIM2 PWM + ADC1/2 TIM2_TRGO for current sensing
  What to do / Must NOT do: Verificar/configurar: TIM2_CH1/2/3 (PA0/PA1/PA2) como PWM 3-fases 20kHz (modo PWM1, prescaler=0, period=8499 para 20kHz desde 170MHz). ADC1 CH4=Ia (PA3) y ADC2 CH3=Ib (PA6) con trigger TIM2_TRGO rising edge, sampling time 47.5 ciclos, scan mode ENABLE (2 conv each). NO DMA — las lecturas se hacen por polling en ISR FOC. NO LPTIM1 — el trigger es directo por TIM2. NO tocar config TIM1 (es encoder, no PWM).
  Parallelization: Wave 1 | Blocked by: W1-D1 | Blocks: W2
  References: foc-firmware/HARDWARE.md (TIM2, ADC1, ADC2), foc-firmware/Core/Src/adc.c (ya modificado), foc-firmware/Core/Src/tim.c (TIM2 TRGO ya configurado), RM0440 (TIM2, ADC chapters)
  Acceptance criteria: TIM2 genera PWM 20kHz en PA0/PA1/PA2. ADC1+ADC2 con TIM2_TRGO rising edge, sampling 47.5 ciclos. Sin DMA, sin LPTIM1. Compila.
  QA scenarios: happy: compile + link + verify TIM2_ARR=8499, verify ADC_CFGR trigger source. failure: TIM2_CH1/2/3 verify pin mux en map file
  Commit: Y | feat(hal): TIM2 PWM + ADC1/2 TIM2_TRGO for 2-shunt FOC

- [ ] 5. [W2] Implement current loop ISR pipeline
  What to do / Must NOT do: Create the FOC ISR at 20kHz: read adc_buffer[3], calculate phase currents, Clarke transform, Park transform, PI current control, inverse Park, SVPWM duty calculation, update TIM1 CCRx. Reference bldc_servo.cc ISR pipeline. ADC offset calibration, overcurrent protection. Do NOT implement position/velocity control yet (just current).
  Parallelization: Wave 2 | Blocked by: W1-D1, W1-D2, W1-D3 | Blocks: W4
  References: fw/bldc_servo.cc (ISR_DoSenseCritical, ISR_DoSense, ISR_CalculateCurrentState, ISR_DoControl, DoPwmControl)
  Acceptance criteria: foc-firmware/src/foc_isr.c compiles. ISR reads ADC, applies transforms, updates PWM. Overcurrent threshold triggers protection. Compile-tested with gcc-arm-none-eabi.
  QA scenarios: happy: compile + link + verify symbol map. failure: simulate overcurrent, confirm PWM kill path
  Commit: Y | feat(foc): 20kHz current loop ISR with Clarke/Park/SVPWM

- [ ] 6. [W3-A] Implement AS5600 I2C driver
  What to do / Must NOT do: Create AS5600 I2C read function: read angle register (0x0C-0x0D) at 400kHz. Return 12-bit position (0-4095). Polling mode (not interrupt — called at ~500Hz from main loop). Handle I2C NACK/timeout gracefully. Do NOT integrate with MotorPosition yet.
  Parallelization: Wave 3 | Blocked by: — | Blocks: W3-C
  References: AS5600 datasheet, fw/stm32_i2c.h (I2C abstraction)
  Acceptance criteria: foc-firmware/src/as5600.c/.h compiles. as5600_read_angle() returns uint16_t 0-4095. Simulated I2C (mock) returns expected values.
  QA scenarios: happy: compile + mock test with known register values. failure: NACK returns error code
  Commit: Y | feat(encoder): AS5600 I2C angle read at 400kHz

- [ ] 7. [W3-B] Implement quadrature encoder timer (TIM1)
  What to do / Must NOT do: Configure TIM1 in encoder mode TI1+TI2 on PA8/PA9. Crear función para leer delta posición cada ciclo FOC (20kHz). Manejar overflow/underflow (16-bit counter). TIM1 no se usa como PWM en este PCB — solo encoder. NO cambiar configuración de TIM2 (es PWM). NO integrar con MotorPosition todavía.
  Parallelization: Wave 3 | Blocked by: — | Blocks: W3-C
  References: foc-firmware/HARDWARE.md (TIM1 encoder mode), RM0440 (TIM1 encoder mode), fw/motor_position.h (quadrature source pattern)
  Acceptance criteria: foc-firmware/src/quadrature.c/.h compila. quad_read_delta() retorna int16_t. Valores simulados producen deltas correctos con overflow handling.
  QA scenarios: happy: compile + overflow wrap test. failure: delta cero cuando motor detenido
  Commit: Y | feat(encoder): TIM1 quadrature encoder mode on PA8/PA9

- [ ] 8. [W3-C] Implement MotorPosition hybrid (I2C absolute + quadrature incremental)
  What to do / Must NOT do: Create hybrid position source: AS5600 provides absolute reference (~500Hz), quadrature provides fast increments (20kHz). PLL filter: error = abs_pos - (quad_pos * scale + offset), correction applied with configurable bandwidth. Reference moteus MotorPosition::CombineSources(). Initialize scale from AS5600 on first boot. Do NOT include other position source types (Hall, BiSS, etc.) — only I2C and quadrature.
  Parallelization: Wave 3 | Blocked by: W3-A, W3-B | Blocks: W4
  References: fw/motor_position.h (CombineSources architecture, PLL filtering)
  Acceptance criteria: foc-firmware/src/motor_position.c/.h compiles. pos_estimate = hybrid_update(as5600_angle, quad_delta). Drift corrected when AS5600 updates (~500Hz). Smooth transitions between updates.
  QA scenarios: happy: compile + simulate constant velocity (quad ramps, as5600 catches up). failure: as5600 stops updating, quad alone maintains position with drift
  Commit: Y | feat(position): MotorPosition hybrid: AS5600 I2C abs + quad incremental

- [ ] 9. [W4] Implement position/velocity control loop (BldcServoControl FSM)
  What to do / Must NOT do: Implement the control FSM: kVoltageFoc (open-loop voltage), kCurrent (IQ/ID control), kVelocity (velocity PID with feed-forward), kPosition (position PID + velocity loop). Reference bldc_servo_control.h template. Integrate with current loop from W2 and position from W3-C. Do NOT implement kStopped, kCalibration, or kTimeout states initially.
  Parallelization: Wave 4 | Blocked by: W2, W3-C | Blocks: W5
  References: fw/bldc_servo_control.h (template class), fw/bldc_servo.cc (control dispatch in ISR) 
  Acceptance criteria: foc-firmware/src/servo_control.c/.h compiles. FSM transitions: kVoltageFoc→kCurrent→kVelocity→kPosition. PID gains configurable. Each state produces correct (Vd, Vq) for current loop.
  QA scenarios: happy: compile + simulate state transitions with mock current/position. verify outputs
  Commit: Y | feat(servo): position/velocity control FSM with PID loops

- [ ] 10. [W5-A] Implement SPI2 slave transport layer
  What to do / Must NOT do: Configurar SPI2 en slave mode (PF1=SCK, PB14=MISO, PA11=MOSI, 16-bit, CPOL=LOW, CPHA=1EDGE, NSS software). Sin DMA por ahora — polling o IRQ. Frame format: 8B header (dest, src, type, len, crc8, reserved) + N*8B payload. CRC-8 validation. Timeout detection. Reference moteus fdcan.cc for frame structure patterns. NO usar SPI1 (no existe en este PCB). NO implementar protocolo de comandos todavía — solo transporte de bytes.
  Parallelization: Wave 5 | Blocked by: W4 | Blocks: W5-B
  References: foc-firmware/HARDWARE.md (SPI2 slave config), fw/fdcan.h/cc (CAN-FD frame structure as reference), RM0440 (SPI2 chapters)
  Acceptance criteria: foc-firmware/src/spi_transport.c/.h compila. spi_send_frame() y spi_recv_frame() con CRC validation. Slave mode funcional. Loopback test con maestro externo.
  QA scenarios: happy: compile + pin verify in map file. failure: CRC mismatch returns error code
  Commit: Y | feat(spi): SPI2 slave transport layer for external comms

- [ ] 11. [W5-B] Adapt moteus protocol over SPI
  What to do / Must NOT do: Adapt the moteus command/response protocol (multiplex/micro_server) to run over SPI instead of CAN-FD. Map CAN-FD frame fields (arbitration_id, data) to SPI frame header. Implement command dispatch table: kStop, kSetOutputPosition, kSetOutputVelocity, kSetTorque, kRezero, kReadConfig. Reference moteus micro_server.cc for command handling. Do NOT implement all moteus commands — only essential subset.
  Parallelization: Wave 5 | Blocked by: W5-A | Blocks: Final
  References: fw/moteus_controller.cc (command handling), fw/micro_server.h/cc (protocol dispatch)
  Acceptance criteria: foc-firmware/src/spi_protocol.c/.h compiles. Commands dispatched correctly via SPI frames. Position/velocity/torque setpoints reach control loop.
  QA scenarios: happy: compile + simulate command/response cycle
  Commit: Y | feat(protocol): adapt moteus command protocol over SPI transport

## Final verification wave
> Runs in parallel after ALL todos. ALL must APPROVE. Surface results and wait for the user's explicit okay before declaring complete.
- [ ] F1. Plan compliance audit
- [ ] F2. Code quality review
- [ ] F3. Real manual QA
- [ ] F4. Scope fidelity

## Commit strategy
- docs(hardware): hardware documentation
- feat(foc): port FOC math library
- feat(driver): DRV8323S SPI3 driver integration
- feat(hal): TIM2 PWM + ADC1/2 TIM2_TRGO config
- feat(foc): current loop ISR
- feat(encoder): AS5600 I2C2
- feat(encoder): quadrature TIM1 on PA8/PA9
- feat(position): MotorPosition hybrid
- feat(servo): position/velocity control FSM
- feat(spi): SPI2 slave transport layer
- feat(protocol): command protocol over SPI

## Changelog
- **2026-07-07**: Corrección mayor de supuestos de hardware tras revisión HARDWARE.md del PCB real. DRV8328S→DRV8323S SPI3, TIM1 PWM→TIM1 encoder + TIM2 PWM, DMA+LPTIM→TIM2_TRGO sin DMA, SPI1→SPI2 slave. TL;DR, scope, dependency matrix, todos W1-D2/W1-D3/W3-B/W5-A, commit strategy actualizados.

## Success criteria
1. Firmware compiles for STM32G431CBU6 target
2. FOC current loop runs at 20kHz (measured via TIM6_IRQ toggle)
3. AS5600 reads position at ~500Hz via I2C2, TIM1 quadrature tracks at 20kHz
4. Position error < 1 electrical degree in steady state
5. SPI frames transmit/receive with CRC validation
6. Motor responds to position/velocity/torque commands via SPI
