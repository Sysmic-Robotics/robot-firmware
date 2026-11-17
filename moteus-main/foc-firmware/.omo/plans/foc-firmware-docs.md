# foc-firmware-docs - Work Plan

## TL;DR (For humans)
<!-- Fill this LAST, after the detailed plan below is written, so it summarizes the REAL plan. -->
<!-- Plain English for a non-engineer: NO file paths, NO todo numbers, NO wave/agent/tool names. -->

**What you'll get:** Un solo documento `HARDWARE.md` que reemplaza los dos actuales (que estaban alucinados). Describe EXACTAMENTE cómo está configurado el STM32G431 por CubeMX: reloj a 170MHz, TIM1 como encoder, TIM2 como PWM, SPI2/SPI3, I2C2, ADC1/ADC2, GPIOs y NVIC. Nada inventado, nada de lo que no hay en el código.

**Why this approach:** Los documentos existentes están completamente inventados (hablan de hardware y periféricos que no existen). La única fuente confiable es el código generado por CubeMX. Un solo documento elimina la duplicación y la confusión.

**What it will NOT do:** No va a inventar hardware (DRV8328S, AS5600), no va a documentar periféricos no configurados (DMA, LPTIM, SPI1, I2C1, TIM3, UART), no va a incluir notas especulativas de "futuro". Solo los hechos del código.

**Effort:** Short
**Risk:** Low - solo documentación, sin cambios de código
**Decisions to sanity-check:** Que toda la información en HARDWARE.md coincida con los archivos .c/.h reales

Your next move: Ejecuta `$start-work` para que el worker genere el documento y elimine PERIPHERALS.md.

---

> TL;DR (machine): Short | Low | Unified HARDWARE.md replaces both hallucinated files with real CubeMX configuration data

## Scope
### Must have
### Must NOT have (guardrails, anti-slop, scope boundaries)

## Verification strategy
> Zero human intervention - all verification is agent-executed.
- Test decision: <TDD | tests-after | none> + framework
- Evidence: .omo/evidence/task-<N>-foc-firmware-docs.<ext>

## Execution strategy
### Parallel execution waves
> Target 5-8 todos per wave. Fewer than 3 (except the final) means you under-split.

### Dependency matrix
| Todo | Depends on | Blocks | Can parallelize with |
| --- | --- | --- | --- |

## Todos
> Implementation + Verification = ONE todo.
<!-- APPEND TASK BATCHES BELOW THIS LINE WITH edit/apply_patch - never rewrite the headers above. -->

### Wave 1 — Documentation Generation
- [x] 1. Write unified HARDWARE.md based on real CubeMX code
  What to do / Must NOT do: 
  Create `/home/sysmic/Desktop/SysmicProyect/robot/moteus-main/foc-firmware/HARDWARE.md` with the following sections based EXACTLY on the real code. The document must match the structure below and contain ONLY real configuration data from the listed source files.
  
  Sections to include:
  1. **MCU Overview**: STM32G431CBU6, UFQFPN48, Cortex-M4F @ 170MHz, 128KB Flash, 32KB SRAM. Include table of key specs.
  2. **System Clock Configuration**: HSI 16MHz → PLLM=/4 → PLLN=85 → PLLP=/2, PLLQ=/2, PLLR=/2 → SYSCLK=170MHz. Include actual code explanation (main.c:120-160). Show HCLK=170MHz, APB1=170MHz, APB2=170MHz.
  3. **Peripheral Configuration**: Per-peripheral subsection with:
     - **TIM1 — Encoder Mode**: PA8(TIM1_CH1), PA9(TIM1_CH2). EncoderMode=TI1, Prescaler=0, Period=65535, no interrupts. (tim.c:44-63)
     - **TIM2 — PWM Generation**: PA0(CH1), PA1(CH2), PA2(CH3). PWM1 mode, Prescaler=0, Period=4294967295 (32-bit max), OCFast enabled. (tim.c:90-121)
     - **SPI2 — Full-Duplex Slave**: PF1(SCK, AF5), PB14(MISO, AF5), PA11(MOSI, AF5). 16-bit, CPOL=LOW, CPHA=1EDGE, NSS Soft. (spi.c:41-57, 110-137)
     - **SPI3 — Full-Duplex Master**: PC10(SCK, AF6), PC11(MISO, AF6), PB5(MOSI, AF6). 16-bit, CPOL=LOW, CPHA=2EDGE, Prescaler=/16 (~10.6MB/s). (spi.c:74-91, 153-170)
     - **I2C2 — Fast Mode**: PC4(SCL, AF4), PF0(SDA, AF4). Timing=0x40621236, 7-bit addressing, analog+digital filter enabled. (i2c.c:40-70)
     - **ADC1**: PA3(IN4), PB1(IN12). 12-bit, right-align, single conversion, software trigger, 2.5 cycle sampling. (adc.c:47-91)
     - **ADC2**: PA6(IN3). Same config as ADC1. (adc.c:109-145)
     - **GPIO**: Outputs (push-pull): PC13, PC14, PC15, PC6, PB0, PB15. Input (floating): PB13. Initial states: PC6=HIGH, rest=LOW. (gpio.c:37-76, gpio.h)
     - **NVIC**: Priority group 4. System exceptions only (BusFault, MemManage, HardFault, SVCall, PendSV, SysTick@prio15). No peripheral interrupts enabled.
     - **HAL Modules**: ADC, I2C, SPI, TIM, GPIO, EXTI, DMA, RCC, FLASH, PWR, CORTEX. (stm32g4xx_hal_conf.h:37-75)
  4. **Complete Pinout Table**: All 25 MCU pins used, with: Pin#, Pin name, Function, Configuration, Alternate Function.
  
  Must NOT do:
  - NO references to DRV8328S, AS5600, or any external IC
  - NO DMA, LPTIM, pipeline, or trigger chain config
  - NO SPI1, I2C1, TIM3, UART, CAN — only what the code configures
  - NO speculative "future work", "planned", "TODO", "this could be used for"
  - NO register-level code samples (keep at HAL config level)
  - NO markdown diagrams or ASCII art of external hardware

  Parallelization: Wave 1 | Blocked by: (none) | Blocks: (none)
  References (executor has NO interview context - be exhaustive):
  - cubemx/FOCTest.ioc (all lines) — CubeMX master config
  - Core/Src/main.c:120-160 — SystemClock_Config
  - Core/Src/tim.c:44-63 — TIM1 Encoder init
  - Core/Src/tim.c:90-121 — TIM2 PWM init
  - Core/Src/tim.c:129-201 — TIM MSP init/postinit
  - Core/Src/spi.c:41-57 — SPI2 Slave init
  - Core/Src/spi.c:74-91 — SPI3 Master init
  - Core/Src/spi.c:110-170 — SPI MSP init GPIO
  - Core/Src/i2c.c:40-70 — I2C2 init
  - Core/Src/i2c.c:96-111 — I2C2 MSP init GPIO
  - Core/Src/adc.c:47-91 — ADC1 init
  - Core/Src/adc.c:109-145 — ADC2 init
  - Core/Src/adc.c:150-230 — ADC12 MSP init GPIO
  - Core/Src/gpio.c:37-76 — MX_GPIO_Init
  - Core/Src/stm32g4xx_it.c:183-192 — SysTick_Handler only
  - Core/Inc/stm32g4xx_hal_conf.h:37-75 — HAL modules enabled
  - Core/Inc/*.h — all peripheral headers
  Acceptance criteria (agent-executable):
  - File exists at `/home/sysmic/Desktop/SysmicProyect/robot/moteus-main/foc-firmware/HARDWARE.md`
  - Every section references ONLY peripherals actually present in the code
  - All pin names match those in gpio.c, tim.c, spi.c, i2c.c, adc.c
  - No mention of DRV8328S, AS5600, DMA, LPTIM, SPI1, I2C1, TIM3
  QA scenarios: 
  - Happy: `grep -c "DRV8328S\|AS5600\|DMA\|LPTIM\|SPI1\|I2C1\|TIM3" HARDWARE.md` -> must return 0
  - Coverage: `grep "TIM1\|TIM2\|SPI2\|SPI3\|I2C2\|ADC1\|ADC2\|GPIO\|NVIC" HARDWARE.md | head -10` -> must find all
  - Evidence: .omo/evidence/task-1-foc-firmware-docs.txt
  Commit: N (documentation)

- [x] 2. Remove hallucinated PERIPHERALS.md
  What to do / Must NOT do:
  Delete `/home/sysmic/Desktop/SysmicProyect/robot/moteus-main/foc-firmware/PERIPHERALS.md` since all content (which was hallucinated) is now superseded by the unified HARDWARE.md.
  
  Must NOT do:
  - Do NOT create any backup or replacement file
  
  Parallelization: Wave 1 | Blocked by: Todo 1 | Blocks: (none)
  References: PERIPHERALS.md (file to delete)
  Acceptance criteria: File `PERIPHERALS.md` no longer exists
  QA scenarios: `ls PERIPHERALS.md` -> must fail (file not found)
  Evidence: .omo/evidence/task-2-foc-firmware-docs.txt
  Commit: N (documentation)

## Final verification wave
> Runs in parallel after ALL todos. ALL must APPROVE. Surface results and wait for the user's explicit okay before declaring complete.
- [x] F1. Plan compliance audit — verify every "Must NOT have" item is absent
- [x] F2. Code fidelity check — spot-check 3 random facts in HARDWARE.md against source code
- [x] F3. Document structure review — HARDWARE.md sections are complete and well-formatted
- [x] F4. Hallucination sweep — grep for any speculatives

## Commit strategy
No code changes — this is a documentation-only task. No commits needed.

## Success criteria
- HARDWARE.md exists with complete, accurate documentation of all CubeMX-generated configuration
- Every fact in HARDWARE.md is traceable to a specific line in Core/Src/*.c or Core/Inc/*.h
- PERIPHERALS.md is deleted
- Zero references to non-existent hardware or unconfigured peripherals
- Zero speculative/future-work content
