# foc-firmware-integration-review - Work Plan

## TL;DR (For humans)

**What you'll get:** Un documento `docs/INTEGRATION_REVIEW.md` que analiza tu firmware contra el de referencia (moteus fw/) en 3 áreas: ADC (medición de corriente), driver DRV8323, e integración encoder→FOC. Incluye 6 hallazgos críticos con soluciones, y un plan de correcciones en 4 fases.

**Why this approach:** La configuración actual de ADC (sampling time 2.5 ciclos = 59 ns) es incompatible con el tiempo de establecimiento del DRV8323 CSA (1500 ns). Sin arreglar esto, el lazo FOC leerá corrientes incorrectas. El plan prioriza: ADC → DRV8323 → FOC en ISR → refinamiento.

**What it will NOT do:** No implementa CORDIC, LPTIM1, DMA, field weakening, ni MTPA. Esa es una fase futura. Tampoco cambia SPI2, I2C2, AS5600, ni la comunicación con la placa master.

**Effort:** Medium (4 fases, ~8-11 días hábiles)
**Risk:** High - la configuración ADC actual impide FOC funcional; los cambios tocan periféricos críticos
**Decisions to sanity-check:** Sampling time ADC (47.5 vs 640.5 ciclos), trigger source (TIM2_TRGO vs LPTIM1), método sin/cos (LUT vs CORDIC)

Your next move: Revisa el plan detallado abajo. Si estás de acuerdo, dime `dale` para aprobar y empezar.

---

> TL;DR (machine): Medium | High | 4 fases: ADC→DRV8323→FOC ISR→refinamiento. Documento de análisis + 6 fixes críticos.

## Scope
### Must have
- Documento `docs/INTEGRATION_REVIEW.md` con análisis completo de ADC, DRV8323, encoder→FOC
- Fix sampling time ADC: 2.5 → 47.5+ ciclos
- Fix trigger ADC: software → TIM2_TRGO hardware (simultáneo)
- Fix VBUS: hardcodeado → ADC2_IN5 (PA9)
- Fix CSA cal: agregar calibración DRV8323
- Fix HiZ: agregar pin de high-Z
- Fix FOC rate: 1 kHz main loop → 20 kHz TIM6/7 ISR
- Fix sin/cos: math.h → lookup table + interpolación

### Must NOT have (guardrails, anti-slop, scope boundaries)
- NO CORDIC (v2), NO LPTIM1 (v2), NO DMA (v2)
- NO field weakening, MTPA, flux braking
- NO cambios en SPI2, I2C2, AS5600, COMM_SPI
- NO refactor de PLAN_AVANCE.md o FIRMWARE_STATUS.md
- NO cambios en el protocolo de comunicación con master
- NO tocar la capa de CubeMX (.ioc) — solo código usuario

## Verification strategy
- Test decision: tests-after (verificación en hardware + simulación)
- Evidence: .omo/evidence/task-<N>-foc-firmware-integration-review.<ext>

## Execution strategy
### Parallel execution waves
Wave 1 — Documentación y Diagnóstico (1 todo)
Wave 2 — ADC Fixes (3 todos, pueden paralelizarse)
Wave 3 — DRV8323 Fixes (2 todos)
Wave 4 — FOC en ISR (2 todos, requiere Wave 2+3)

### Dependency matrix
| Todo | Depends on | Blocks | Can parallelize with |
| --- | --- | --- | --- |
| 1. Analysis doc | — | — | — |
| 2. ADC sampling time | — | 5,6 | 3,4 |
| 3. ADC trigger HW | — | 5,6 | 2,4 |
| 4. VBUS sensing | — | 5,6 | 2,3 |
| 5. DRV8323 CSA cal | 2,3 | 7 | 6 |
| 6. DRV8323 HiZ | 2,3 | 7 | 5 |
| 7. FOC ISR + LUT | 5,6 | — | — |

## Todos

### Wave 1 — Analysis Document

- [x] 1. Write `docs/INTEGRATION_REVIEW.md`
  What to do / Must NOT do:
  Crear `/home/sysmic/Desktop/SysmicProyect/robot/moteus-main/foc-firmware/docs/INTEGRATION_REVIEW.md` con las siguientes secciones basadas EXACTAMENTE en el código real de foc-firmware y moteus fw/:
  
  **Sección 1 — ADC Configuration Comparison:**
  - Tabla comparativa foc-firmware vs moteus fw/ (clock, resolución, sampling time, trigger, sincronización, canales)
  - Mapeo de pines (Ia=PA3, Ib=PA6, Ic=PB1) con verificación contra moteus
  - **Problemas identificados (6 críticos, 5 medios, 3 mejoras futuras)**
  - Para C1: explicar CSA settling time con tabla de ganancias (DRV8323 datasheet §7.5)
  - Para C2: explicar por qué el trigger software causa error de fase
  - Citas precisas a archivos: `adc.c:80,92,139`, `drv8323.h:58-67`, `stm32g4_adc.cc:137-148`
  - Configuración recomendada con código de ejemplo
  - Referencia: `fw/drv8323.h:58-67`, `fw/stm32g4_adc.cc:137-151`, `fw/bldc_servo_control.h:817-1124`
  
  **Sección 2 — DRV8323 Driver Review:**
  - Análisis de registros (reg2-reg6) con valores decodificados
  - Tabla comparativa foc-firmware vs moteus fw/
  - Diferencias: SPI (HAL vs bitbang), CS pin, HiZ, calibración CSA, enable sequence
  - Referencia: `Tools/drv8323.c:126-181`, `fw/drv8323.cc:237-268`, `fw/drv8323.h`
  
  **Sección 3 — Encoder → FOC Integration:**
  - Flujo actual (main loop @ 1 kHz)
  - Comparación con moteus (ISR @ 15-30 kHz)
  - Análisis de TIM1 (encoder mode TI1, overflow, velocidad, θ_eléc)
  - Análisis de FOC (Clarke, Park, PI, SVM)
  - Referencia: `Tools/encoder.c`, `Tools/foc.c`, `fw/bldc_servo_control.h:464-627`
  
  **Sección 4 — Resumen:**
  - Tabla de 6 hallazgos críticos con archivo/línea y solución
  - Tabla de 5 hallazgos medios
  - Tabla de mejoras futuras (CORDIC, CCM, LPTIM1, decoupling, BEMF ff)
  
  **Sección 5 — Roadmap (4 fases):**
  - Fase 0: Diagnóstico (✓ completado en este documento)
  - Fase 1: ADC (2-3 días): sampling time, trigger HW, VBUS, calibración
  - Fase 2: DRV8323 (1 día): calibración CSA, HiZ
  - Fase 3: FOC ISR (3-4 días): TIM6/7, LUT sin/cos, encoder en ISR
  - Fase 4: Refinamiento (2-3 días): feedforward, BEMF, arranque, PI tuning
  
  Must NOT do:
  - NO documentar periféricos no usados (DMA, LPTIM1, SPI1, I2C1, TIM3, UART, CAN)
  - NO incluir código de implementación (solo diagnóstico y recomendaciones)
  - NO especular sobre hardware que no está en el código
  - NO reemplazar ni modificar PLAN_AVANCE.md, FIRMWARE_STATUS.md, HARDWARE.md
  
  Parallelization: Wave 1 | Blocked by: (none) | Blocks: (none)
  References:
  - `foc-firmware/Core/Src/adc.c` (all lines) — ADC config actual
  - `foc-firmware/Core/Src/tim.c` (all lines) — TIM1+2 config real
  - `foc-firmware/Tools/drv8323.c` (all lines) — driver actual
  - `foc-firmware/Tools/drv8323.h` (all lines) — driver header
  - `foc-firmware/Tools/foc.c` (all lines) — FOC implementation
  - `foc-firmware/Tools/foc.h` (all lines) — FOC header
  - `foc-firmware/Tools/encoder.c` (all lines) — encoder impl
  - `foc-firmware/Tools/encoder.h` (all lines) — encoder header
  - `foc-firmware/Core/Src/main.c:120-161,202-218` — init + FOC loop
  - `fw/stm32g4_adc.h` (all lines) — moteus ADC header
  - `fw/stm32g4_adc.cc:137-151` — LPTIM1 trigger
  - `fw/drv8323.h:58-67` — CsaSettlingTime()
  - `fw/drv8323.cc:237-268` — CSA calibration
  - `fw/foc.h:34-76` — CORDIC
  - `fw/bldc_servo_control.h:464-627,750-768,817-1124` — FOC ISR
  - `fw/moteus_hw.cc:322-388` — family 2 (c1) pin config
  - `fw/motor_driver.h` (all lines) — motor driver interface
  - `DRV8323S Datasheet SLVSEM5B §7.5` — CSA settling time
  Acceptance criteria:
  - File exists at `docs/INTEGRATION_REVIEW.md`
  - Contains all 5 required sections
  - All 6 critical findings documented with file:line references
  - No mention of DMA, LPTIM1, SPI1, I2C1, TIM3, CAN
  QA scenarios:
  - Happy: `grep -c "ADC\|DRV8323\|encoder\|FOC\|CSA\|sampling\|trigger\|VBUS\|HiZ\|CORDIC\|LPTIM1\|DMA" docs/INTEGRATION_REVIEW.md` → at least 10+ matches
  - Coverage: `grep -c "CRÍTICO\|🔴" docs/INTEGRATION_REVIEW.md` → must be >= 6
  - Anti-slop: `grep -c "SPI1\|I2C1\|TIM3\|CAN\|DMA\|LPTIM1" docs/INTEGRATION_REVIEW.md` → must be 0 (only in "future" section as "NO")
  - Evidence: `.omo/evidence/task-1-foc-firmware-integration-review.txt`
  Commit: N (documentation)

### Wave 2 — ADC Fixes

- [x] 2. Increase ADC sampling time from 2.5 to 47.5+ cycles
  What to do / Must NOT do:
  En `Core/Src/adc.c`:
  - Localizar los 3 lugares donde se usa `ADC_SAMPLETIME_2CYCLES_5`:
    - `adc.c:80` — ADC1 CH4 (Ia = PA3)
    - `adc.c:92` — ADC1 CH12 (Ic = PB1)
    - `adc.c:139` — ADC2 CH3 (Ib = PA6)
  - Cambiar a `ADC_SAMPLETIME_47CYCLES_5` (47.5 ciclos ≈ 1.12 µs @ 42.5 MHz)
  - Debe estar en la sección `/* USER CODE BEGIN ADC1_Init 2 */` y equivalente para ADC2
  
  Verificación: Para gain=20 V/V: 1.12 µs < 1.5 µs recomendado por moteus pero es el segundo valor más alto. Si resulta insuficiente, escalar a `ADC_SAMPLETIME_640CYCLES_5`.
  
  Must NOT do:
  - NO cambiar el clock prescaler del ADC (mantener SYNC_PCLK_DIV4)
  - NO tocar la configuración de scan mode o número de conversiones
  - NO modificar la sección generated de CubeMX (solo USER CODE blocks)
  
  Parallelization: Wave 2 | Blocked by: (none) | Blocks: 5,6
  References: `Core/Src/adc.c:78-84, 89-94, 137-140`
  Acceptance criteria: `grep -c "ADC_SAMPLETIME_47CYCLES_5" Core/Src/adc.c` → 3 matches (CH4, CH12, CH3)
  QA scenarios:
  - Happy: Compilar sin errores -> `tools/bazel build ...` o `make`
  - Built: `grep "ADC_SAMPLETIME_47CYCLES_5" Core/Src/adc.c | wc -l` → 3
  - Evidence: `.omo/evidence/task-2-foc-firmware-integration-review.txt`
  Commit: Y | `fix(adc): increase sampling time to 47.5 cycles for DRV8323 CSA settling`

- [x] 3. Configure hardware trigger for ADC from TIM2_TRGO
  What to do / Must NOT do:
  En `Core/Src/adc.c`:
  - Cambiar `hadc1.Init.ExternalTrigConv` de `ADC_SOFTWARE_START` a `ADC_EXTERNALTRIG_T2_TRGO`
  - Cambiar `hadc1.Init.ExternalTrigConvEdge` de `ADC_EXTERNALTRIGCONVEDGE_NONE` a `ADC_EXTERNALTRIGCONVEDGE_RISING`
  - Mismos cambios para `hadc2`
  
  En `Core/Src/tim.c` (USER CODE de TIM2):
  - Configurar TIM2_TRGO = Update event (cada vez que TIM2 llega al centro del PWM)
  - `sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;`
  
  Esto sincroniza el muestreo ADC con el centro del PWM (mínimo ruido de conmutación).
  
  Must NOT do:
  - NO configurar dual mode (ADC_MODE_DUAL_REGULAR_PARALLEL) — eso requiere cambios más grandes
  - NO modificar la sección generated de CubeMX (solo USER CODE blocks)
  - NO cambiar LPTIM1 o DMA
  
  Parallelization: Wave 2 | Blocked by: (none) | Blocks: 5,6
  References: `Core/Src/adc.c:58-59, 124-125`, `Core/Src/tim.c:100`
  Acceptance criteria:
  - `grep "ADC_EXTERNALTRIG_T2_TRGO" Core/Src/adc.c` → 2 matches (ADC1 + ADC2)
  - `grep "TIM_TRGO_UPDATE" Core/Src/tim.c` → 1 match
  QA scenarios:
  - Happy: Compilar sin errores
  - Coverage: Ambos ADCs configurados con trigger HW
  - Evidence: `.omo/evidence/task-3-foc-firmware-integration-review.txt`
  Commit: Y | `fix(adc): configure TIM2_TRGO hardware trigger for simultaneous sampling`

- [x] 4. Add VBUS sensing on ADC2_IN5 (PA9)
  What to do / Must NOT do:
  En `Core/Src/adc.c`:
  - Agregar ADC2_CH5 (PA9) como rank 2 de ADC2 (después de CH3/Ib)
  - Cambiar `hadc2.Init.NbrOfConversion` de 1 a 2
  - Configurar sampling time para CH5: `ADC_SAMPLETIME_47CYCLES_5`
  - Esto debe ir en `/* USER CODE BEGIN ADC2_Init 2 */`
  
  En `Tools/foc.h`:
  - Agregar constante `FOC_VBUS_ADC_SCALE` para convertir ADC → Voltios
  - Asumiendo divisor 10:1: escala = 3.3 / 4096 * 11 (ajustar según PCB)
  
  En `Tools/foc.c`:
  - En `read_phase_currents()` o función nueva `read_vbus()`: leer ADC2_CH5 después de Ib
  - Reemplazar el hardcoded `24.0f` con la medición real
  - El factor de escala del divisor debe ser configurable
  
  Must NOT do:
  - NO romper la lectura de Ib (CH3) existente
  - NO usar DMA para la lectura (mantener trigger HW + polling del resultado)
  - NO asumir un valor específico de divisor resistivo sin verificar PCB
  - Documentar el valor como `TODO: verify divider ratio on PCB`
  
  Parallelization: Wave 2 | Blocked by: (none) | Blocks: 5,6
  References: `Core/Src/adc.c:109-150`, `Tools/foc.c:67-87, 215`, FIRMWARE_STATUS.md línea 82
  Acceptance criteria:
  - `grep "ADC_CHANNEL_5" Core/Src/adc.c` → 1 match
  - `grep "NbrOfConversion.*2" Core/Src/adc.c` → 1 match
  - `grep -c "24.0f" Tools/foc.c` → must be 0 (eliminar hardcode)
  QA scenarios:
  - Happy: Compilar, VBUS se lee de ADC2_CH5
  - Anti-regression: Ib sigue leyendo correctamente de ADC2_CH3
  - Evidence: `.omo/evidence/task-4-foc-firmware-integration-review.txt`
  Commit: Y | `feat(adc): add VBUS sensing on ADC2_IN5 (PA9), remove hardcoded 24V`

### Wave 3 — DRV8323 Fixes

- [x] 5. Implement DRV8323 CSA offset calibration
  What to do / Must NOT do:
  En `Tools/drv8323.c`:
  - En `DRV8323_Enable()`, después de `HAL_Delay(1)` y `DRV8323_WriteConfig(h)`:
    1. Leer reg6 actual: `uint16_t reg6 = DRV8323_ReadReg(h, DRV8323_REG_CSA_CTRL);`
    2. Escribir reg6 con CAL bits activados: `DRV8323_WriteReg(h, DRV8323_REG_CSA_CTRL, reg6 | 0x1C);`
    3. Esperar ~10 µs (usando DWT cycle counter como ya está implementado)
    4. Escribir reg6 sin CAL bits: `DRV8323_WriteReg(h, DRV8323_REG_CSA_CTRL, reg6 & ~0x1C);`
    5. Verificar que CAL bits volvieron a 0: leer reg6 y chequear `(reg6 & 0x1C) == 0`
  - Si la verificación falla, marcar fault_config
  
  En `Tools/drv8323.h`:
  - Agregar `#define DRV8323_CSA_CAL_MASK 0x1C`
  
  Must NOT do:
  - NO cambiar la estructura de `DRV8323_Config`
  - NO agregar nuevos campos al handle si no son necesarios
  - NO usar HAL_Delay para microsegundos (usar DWT existente)
  
  Parallelization: Wave 3 | Blocked by: 2,3 | Blocks: 7
  References: `fw/drv8323.cc:237-268` (StartCalibrate/StopCalibrate/FinishCalibrate), `Tools/drv8323.c:183-188`, DRV8323S datasheet §7.6.3
  Acceptance criteria:
  - `grep -c "0x1C\|CAL" Tools/drv8323.c` → al menos 2 referencias
  - Después de `DRV8323_Enable()`, el CSA está calibrado
  QA scenarios:
  - Happy: Enable → calibración ejecutada sin error
  - Failure: Si CAL bits no se limpian, fault_config se marca
  - Evidence: `.omo/evidence/task-5-foc-firmware-integration-review.txt`
  Commit: Y | `feat(drv8323): implement CSA offset calibration on enable`

- [x] 6. Add HiZ pin control for safe power-on/off
  What to do / Must NOT do:
  En `Tools/drv8323.h`:
  - Agregar campo `hiz_port` y `hiz_pin` al `DRV8323_HandleTypeDef`
  - Agregar parámetros a `DRV8323_Init()` o crear función `DRV8323_SetHiZPin()`
  - Agregar funciones `DRV8323_PowerOn()` / `DRV8323_PowerOff()`
  
  En `Tools/drv8323.c`:
  - HiZ pin: HIGH = driver habilitado, LOW = high-Z (disipación segura)
  - `DRV8323_PowerOff()`: escribir HiZ = LOW (pone gates en high-Z)
  - `DRV8323_PowerOn()`: escribir HiZ = HIGH (gates controlados por PWM)
  
  En `Core/Src/main.c`:
  - Configurar PC15 como HiZ output (o el pin que corresponda según PCB)
  - Llamar `DRV8323_PowerOff()` antes de habilitar, `DRV8323_PowerOn()` después
  
  Must NOT do:
  - NO cambiar la firma de `DRV8323_Enable`/`Disable` (son aparte)
  - NO asumir que PC15 es el pin correcto sin verificar PCB (seguir c1 como referencia: NC = evaluar)
  
  Parallelization: Wave 3 | Blocked by: 2,3 | Blocks: 7
  References: `fw/drv8323.cc:450-451` (PowerOn/PowerOff), `fw/moteus_hw.cc:324` (PC15 como HiZ), `Tools/drv8323.h:122-133`
  Acceptance criteria:
  - `grep "PowerOn\|PowerOff\|hiz" Tools/drv8323.h` → funciones declaradas
  - `grep "PowerOn\|PowerOff\|hiz" Tools/drv8323.c` → implementadas
  QA scenarios:
  - Happy: PowerOff pone HiZ LOW, PowerOn pone HiZ HIGH
  - Integration: Enable+PowerOn → PWM activo; Disable+PowerOff → HC high-Z
  - Evidence: `.omo/evidence/task-6-foc-firmware-integration-review.txt`
  Commit: Y | `feat(drv8323): add HiZ pin control for safe power sequencing`

### Wave 4 — FOC ISR + Lookup Table

- [x] 7. Move FOC loop to TIM6/7 ISR @ 20 kHz with LUT sin/cos
  What to do / Must NOT do:
  
  **Parte A — Timer ISR:**
  - Configurar TIM6 o TIM7 como timer básico para interrupción @ 20 kHz
  - En `Core/Src/tim.c` (USER CODE): MX_TIM6_Init con Prescaler=0, Period=8500 (170 MHz / 20 kHz = 8500)
  - En `Core/Src/stm32g4xx_it.c`: implementar `TIM6_DAC_IRQHandler` o `TIM7_IRQHandler`
  - En la ISR: llamar `Encoder_Update(&encoder)` + `FOC_Run(...)` + clear flag
  - Prioridad de interrupción: más alta que SPI2, más baja que SysTick
  
  **Parte B — Lookup Table para sin/cos:**
  - En `Tools/foc.h` o nuevo `Tools/trig_lut.h`:
    - Tabla de 256 o 1024 entradas para seno en Q15 o float
    - `static inline float lut_sin(uint16_t index)` con interpolación lineal
  - En `Tools/foc.c`:
    - Convertir θ_eléc (float [0, 2π)) → índice LUT
    - Reemplazar `sinf(theta_elec)` / `cosf(theta_elec)` con LUT
    - La tabla puede ser seno y derivar coseno por desplazamiento de índice
  
  **Parte C — Integración:**
  - Eliminar el FOC del main loop (el que usa HAL_GetTick)
  - Mantener SPI2 polling + telemetría en main loop
  - Mantener DRV8323 fault check periódico en main loop
  - Sincronizar: el main loop ya no necesita llamar FOC_Run ni Encoder_Update
  
  Must NOT do:
  - NO usar CORDIC todavía (fase v2)
  - NO cambiar la estructura de FOC_State ni la API de FOC_Run
  - NO mover SPI2 ni telemetría a la ISR
  - NO eliminar el DRV8323 fault check
  - NO cambiar el period de TIM2 (mantener 4250 para center-aligned 20 kHz)
  
  Parallelization: Wave 4 | Blocked by: 5,6 | Blocks: (none)
  References:
  - `Core/Src/tim.c:90-127` — referencia de TIM2 init
  - `Core/Src/stm32g4xx_it.c` — estructura de handlers
  - `Tools/foc.c:246-277` — FOC_Run a migrar
  - `Tools/encoder.c:62-94` — Encoder_Update a migrar
  - `Core/Src/main.c:202-218` — FOC loop a eliminar
  - `fw/bldc_servo_control.h:464-627` — referencia de ISR structure
  
  Acceptance criteria:
  - `grep "TIM6\|TIM7" Core/Src/stm32g4xx_it.c` → handler implementado
  - `grep "TIM6\|TIM7" Core/Src/tim.c` → init function presente
  - `grep "LUT\|lut_sin\|trig_lut" Tools/foc.c` → lookup table reemplaza sinf/cosf
  - `grep "FOC_Run\|Encoder_Update" Core/Src/main.c` → SOLO en USER CODE 3? debe estar 0 en el loop principal
  QA scenarios:
  - Happy: ISR se ejecuta a 20 kHz, FOC_Run se completa dentro de la ventana de 50 µs
  - Timing: Medir tiempo de ejecución de ISR (debe ser < 40 µs para margen)
  - Anti-regression: main loop ya no llama FOC_Run ni Encoder_Update
  - Evidence: `.omo/evidence/task-7-foc-firmware-integration-review.txt`
  Commit: Y | `feat(foc): move FOC to TIM6 ISR @ 20kHz with LUT sin/cos`

## Final verification wave
- [ ] F1. Plan compliance audit — verificar cada Must NOT have
- [ ] F2. Code quality review — sin regresiones en ADC, DRV8323, encoder
- [ ] F3. Build verification — compila sin errores ni warnings
- [ ] F4. Scope fidelity — solo se modificaron los archivos especificados

## Commit strategy
Commits atómicos por todo (documentación, ADC, DRV8323, FOC ISR). Prefijo convencional: `fix:` para bugs, `feat:` para funciones nuevas.

## Success criteria
- `docs/INTEGRATION_REVIEW.md` existe con análisis completo
- ADC sampling time = 47.5+ ciclos
- ADC trigger = TIM2_TRGO (hardware simultáneo)
- VBUS se lee de ADC2_IN5 (PA9) en lugar de hardcode
- DRV8323 CSA calibrado automáticamente al enable
- FOC se ejecuta en TIM6/7 ISR a 20 kHz con LUT sin/cos
- HiZ pin controla power-on/off seguro
- Sin regresiones en SPI2, I2C2, AS5600, encoder básico
