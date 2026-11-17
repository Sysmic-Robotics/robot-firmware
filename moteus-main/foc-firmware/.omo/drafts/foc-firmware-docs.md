---
slug: foc-firmware-docs
status: awaiting-approval
intent: clear
pending-action: write .omo/plans/foc-firmware-docs.md
approach: Replace hallucinated HARDWARE.md (373 lines) and PERIPHERALS.md (368 lines) with one unified HARDWARE.md based solely on real CubeMX-generated code.
---

# Draft: foc-firmware-docs

## Components (topology ledger)
<!-- Lock the SHAPE before depth. One row per top-level component that can succeed or fail independently. -->
<!-- id | outcome (one line) | status: active|deferred | evidence path -->
| id | outcome | status | evidence path |
|---|---|---|---|
| unified-doc | New HARDWARE.md with real config | active | Core/Src/*.c, Core/Inc/*.h, cubemx/FOCTest.ioc |
| remove-hallucinated | Delete PERIPHERALS.md | active | HARDWARE.md (hallucinated), PERIPHERALS.md (hallucinated) |

## Open assumptions (announced defaults)
<!-- Record any default you adopt instead of asking, so the user can veto it at the gate. -->
<!-- assumption | adopted default | rationale | reversible? -->
| assumption | adopted default | rationale | reversible? |
|---|---|---|---|
| Document format | Single markdown file HARDWARE.md | User approved unified doc | Yes (can split later) |
| Level of detail | Only existing config, no speculation | User chose "only what exists" | Yes (can add later) |
| Section ordering | System → Peripherals (by instance) → Pinout → NVIC | Logical flow: top-down from clock to pins | Yes (cosmetic) |

## Findings (cited - path:lines)

### Hallucinations in HARDWARE.md
- TIM1 claimed as 3-phase PWM (line ~107-119) → Actually TIM1 is Encoder Mode TI1 (tim.c:44-63)
- ADC1 claimed as IN1/IN2/IN3 on PA0/PA1/PA2 (line ~124-128) → Actually ADC1_IN4(PA3), ADC1_IN12(PB1), ADC2_IN3(PA6) (adc.c:78-84, 132-138)
- SPI1 claimed as main processor link (lines ~192-211) → No SPI1 exists; SPI2=Slave, SPI3=Master (spi.c)
- I2C1 claimed for AS5600 on PB6/PB7 (lines ~155-161) → Actually I2C2 on PC4(SCL)/PF0(SDA) (i2c.c:96-111)
- TIM3 claimed for quad encoder (lines ~167-173) → No TIM3 exists
- DRV8328S/AS5600/LPTIM/DMA claimed → None configured in code
- GPIO claimed PB12(nFAULT), PB13(EN) (lines ~274-277) → Actual: outputs=PC13,PC14,PC15,PC6,PB0,PB15; input=PB13 (gpio.c:48-75)

### Real configuration from code
- MCU: STM32G431CBU6, UFQFPN48 (FOCTest.ioc:26-39)
- SYSCLK: HSI 16MHz → PLLM=/4 → PLLN=85 → PLLR=/2 → 170MHz (main.c:132-159)
- TIM1: Encoder Mode TI1, Prescaler=0, Period=65535 (tim.c:44-63)
- TIM2: PWM CH1/CH2/CH3, Prescaler=0, Period=4294967295 (32-bit) (tim.c:90-121)
- SPI2: Slave, 16-bit, CPOL=LOW, CPHA=1EDGE (spi.c:41-57)
- SPI3: Master, 16-bit, CPOL=LOW, CPHA=2EDGE, Prescaler=/16~10.6MB/s (spi.c:74-91)
- I2C2: Fast Mode, Timing=0x40621236, 7-bit addr (i2c.c:40-70)
- ADC1: CH4(PA3) + CH12(PB1), 12-bit, SW trigger (adc.c:47-91)
- ADC2: CH3(PA6), 12-bit, SW trigger (adc.c:109-145)
- GPIO: Outputs=PC13,PC14,PC15,PC6,PB0,PB15; Input=PB13 (gpio.c:37-76)
- NVIC: System exceptions only (SysTick at priority 15) (stm32g4xx_it.c + FOCTest.ioc:72-81)
- HAL enabled: ADC, I2C, SPI, TIM, GPIO, EXTI, DMA, RCC, FLASH, PWR, CORTEX (stm32g4xx_hal_conf.h:37-75)

## Decisions (with rationale)
| Decision | Choice | Rationale |
|---|---|---|
| Unified doc | One HARDWARE.md replaces both hallucinated files | User explicitly chose unified over split |
| Content scope | Only CubeMX-generated config, no hardware assumptions | No external components (DRV8328S, AS5600) are in this code |
| Remove PERIPHERALS.md | Delete it | Content merged into HARDWARE.md |

## Scope IN
- Pinout table with real MCU pin functions
- System clock configuration (HSI→PLL→170MHz)
- Per-peripheral configuration tables (TIM1, TIM2, SPI2, SPI3, I2C2, ADC1, ADC2, GPIO)
- HAL module list from stm32g4xx_hal_conf.h
- NVIC configuration (system exceptions only)
- Delete PERIPHERALS.md

## Scope OUT (Must NOT have)
- Any reference to DRV8328S, AS5600, or external hardware
- DMA, LPTIM, or any pipeline configuration
- SPI1, I2C1, TIM3, UART, CAN, or any unconfigured peripheral
- Interrupt handlers for peripherals (none configured)
- Notes about "future work", "planned", "TODO"
- Speculative pin sharing analysis or conflicts

## Open questions
None — all questions answered by exploration + user preferences.

## Approval gate
status: awaiting-approval
<!-- When exploration is exhausted and unknowns are answered, set status: awaiting-approval. -->
<!-- That durable record is the loop guard: on a later turn read it and resume at the gate instead of re-running exploration. -->
