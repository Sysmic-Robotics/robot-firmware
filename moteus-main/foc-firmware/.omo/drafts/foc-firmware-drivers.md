---
slug: foc-firmware-drivers
status: awaiting-approval
intent: clear
pending-action: approve the plan, then execute `$start-work`
approach: 3 drivers (AS5600 fix + TIM1 encoder helpers + DRV8323S new) in parallel Wave 1, then main.c integration in Wave 2
---

# Draft: foc-firmware-drivers

## Components (topology ledger)
1. AS5600 driver fix (.h/.c) — STM32G4 ports, blocking I2C, fix LPM2 bug
2. TIM1 encoder helpers (encoder.h) — inline read/reset functions
3. DRV8323S driver (.h/.c) — SPI register access via HAL_SPI, HW SPI3
4. main.c integration — initialization + demo code in USER CODE blocks

## Open assumptions (announced defaults)
- DRV8323 CPHA=2EDGE works: user reviewed datasheet and confirmed compatibility
- SPI3 bus exclusive to DRV8323: user confirmed no other SPI devices share it
- AS5600 DIR pin not connected: user confirmed no DIR pin used
- All new/modified code in Tools/ or USER CODE sections only

## Findings (cited - path:lines)
- AS5600 driver was for STM32F0 (stm32f0xx_hal.h, stm32f031x6.h) — need G4 headers
- AS5600 driver uses interrupt-based I2C (_IT) — need to convert to blocking
- Bug at LPM2: `a->confRegister[1] &= (1UL << 0)` should be `&= ~(1UL << 0)` — as5600.c:86
- TIM1 already in EncoderMode=TI1 (tim.c:44-63) — verified earlier
- SPI3 CPOL=LOW, CPHA=2EDGE, 16-bit, Master at ~10.6 MB/s — user confirmed compatible with DRV8323
- DRV8323 reference exists at fw/drv8323.{h,cc} — uses bitbang SPI, C++ mbed framework
- Pins chosen: CS=PC13, ENABLE=PC14, FAULT=PB13

## Decisions (with rationale)
| Decision | Rationale |
|----------|-----------|
| Blocking I2C (polling) | NVIC no tiene periféricos habilitados. Más simple. |
| Hardware SPI3 para DRV8323 | Ya configurado como Master 16-bit. Exclusivo para gate driver. |
| NO bitbang SPI | El proyecto moteus lo usa por contexto (mbed), pero tenemos HAL_SPI disponible. |
| AS5600 address 0x6C | HAL G4 espera dirección 7-bit left-aligned (0x36 << 1) |

## Scope IN
- Tools/as5600.h + .c: fix includes, fix I2C calls, fix LPM2 bug
- Tools/encoder.h: new helper for TIM1 encoder
- Tools/drv8323.h + .c: new DRV8323 driver
- Core/Src/main.c: USER CODE integration

## Scope OUT (Must NOT have)
- No CubeMX file changes (tim.c, spi.c, i2c.c, etc.)
- No .ioc changes
- No fw/ modifications
- No C++ code in Tools/
- No interrupt-based I2C
- No printf/uart debug code

## Open questions
None — all resolved with user.

## Approval gate
status: awaiting-approval
The plan (.omo/plans/foc-firmware-drivers.md) is written and ready for review.
