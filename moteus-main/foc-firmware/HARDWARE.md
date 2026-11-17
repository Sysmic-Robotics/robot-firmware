# Hardware Configuration

Board: custom (FOCTest project)
STM32CubeMX version: 6.17.0
Firmware Package: STM32Cube FW_G4 V1.6.3
Target Toolchain: STM32CubeIDE (GCC)

## MCU Overview

| Property | Value |
|---|---|
| Part number | STM32G431CBU6 |
| Package | UFQFPN48 (48 pins) |
| Core | ARM Cortex-M4F with single-precision FPU |
| Max frequency | 170 MHz |
| Flash | 128 KB |
| SRAM | 32 KB |

## System Clock Configuration

The system clock is generated from the internal HSI oscillator via the PLL.

**Clock source:** HSI 16 MHz

**PLL configuration:**
- PLL source: HSI
- PLLM (prescaler): /4 (16 MHz / 4 = 4 MHz)
- PLLN (multiplier): x85 (4 MHz x 85 = 340 MHz VCO)
- PLLR (system clock divider): /2 (340 MHz / 2 = 170 MHz)
- PLLP (other divider): /2
- PLLQ (other divider): /2

**Final clock tree:**

| Bus | Frequency | Source |
|---|---|---|
| SYSCLK | 170 MHz | PLLRCLK |
| HCLK (AHB) | 170 MHz | SYSCLK / 1 |
| APB1 (PCLK1) | 170 MHz | HCLK / 1 |
| APB2 (PCLK2) | 170 MHz | HCLK / 1 |
| ADC12 | 170 MHz | SYSCLK (before ADC prescaler) |

**Flash latency:** 4 wait states

**Voltage scaling:** PWR_REGULATOR_VOLTAGE_SCALE1_BOOST

## Peripheral Configuration

### TIM1 -- Encoder Interface

Configured as a quadrature encoder interface for position sensing. Used in TI12 mode (counts on BOTH edges of BOTH channels for 4x resolution).

| Parameter | Value |
|---|---|
| Prescaler | 0 |
| Counter mode | UP |
| Period | 65535 (16-bit max) |
| Auto-reload preload | DISABLE |
| Encoder mode | TI12 (counts on both TI1 and TI2 edges) |
| IC1 polarity | RISING |
| IC1 selection | DIRECTTI |
| IC1 prescaler | DIV1 |
| IC1 filter | 0 |
| IC2 polarity | RISING |
| IC2 selection | DIRECTTI |
| IC2 prescaler | DIV1 |
| IC2 filter | 0 |
| Master output trigger | TRGO_RESET |
| Master output trigger 2 | TRGO2_RESET |
| Master-slave mode | DISABLE |

**GPIO mapping:**

| Signal | Pin | Alternate function |
|---|---|---|
| TIM1_CH1 | PA8 | AF6 |
| TIM1_CH2 | PA9 | AF6 |

### TIM2 -- 3-Phase PWM Generation

Three-channel PWM generator in center-aligned mode. The auto-reload is set to 4250 (20 kHz PWM switching). TIM2's TRGO_UPDATE triggers the ADCs for synchronized current sampling.

| Parameter | Value |
|---|---|
| Prescaler | 0 |
| Counter mode | CENTERALIGNED2 (center-aligned, counter counts up then down; compare flags set on down-count only) |
| Period | 4250 (20 kHz PWM switching) |
| Auto-reload preload | ENABLE |
| Master output trigger | TRGO_UPDATE (triggers ADC1/ADC2 conversions) |
| Master-slave mode | DISABLE |

**Output channel configuration (CH1, CH2, CH3):**

| Parameter | Value |
|---|---|
| Mode | PWM1 |
| Pulse | 0 (updated in FOC ISR at 20 kHz) |
| Polarity | HIGH |
| Fast mode | ENABLE |

**GPIO mapping:**

| Signal | Pin | Alternate function |
|---|---|---|
| TIM2_CH1 | PA0 | AF1 |
| TIM2_CH2 | PA1 | AF1 |
| TIM2_CH3 | PA2 | AF1 |

### TIM6 -- FOC Loop Timer

TIM6 provides the 20 kHz time base for the FOC control loop. Its update event triggers the TIM6_DAC_IRQn interrupt, where `HAL_TIM_PeriodElapsedCallback` calls `FOC_ISR()`.

| Parameter | Value |
|---|---|
| Prescaler | 0 |
| Counter mode | UP |
| Period | 8499 (→ 20 kHz: 170 MHz / (8499 + 1)) |
| Auto-reload preload | ENABLE |
| Interrupt | TIM6_DAC_IRQn (preempt priority 0, sub 0) |

No GPIO pins assigned (TIM6 is a basic timer).

### CORDIC -- Hardware Math Accelerator

The CORDIC co-processor is initialized via `MX_CORDIC_Init()` for hardware-accelerated trigonometric computations (used by Park/Inverse-Park transforms in the FOC loop).

| Parameter | Value |
|---|---|
| Instance | CORDIC |
| Clock | __HAL_RCC_CORDIC_CLK_ENABLE() during `HAL_CORDIC_MspInit` |

No GPIO pins assigned (internal peripheral).

### SPI2 -- Full-Duplex Slave (Host Communication)

Configured as a 16-bit slave for communication with the external host controller. The protocol uses 5-word frames (80 bits) with software NSS management.

| Parameter | Value |
|---|---|
| Mode | SLAVE |
| Direction | 2 lines (full duplex) |
| Data size | 16 bits |
| Clock polarity (CPOL) | LOW |
| Clock phase (CPHA) | 1 EDGE (capture on first edge) |
| NSS | Software managed |
| First bit | MSB |
| TI mode | DISABLE |
| CRC calculation | DISABLE |

**GPIO mapping:**

| Signal | Pin | Alternate function |
|---|---|---|
| SPI2_SCK | PF1 (OSC_OUT) | AF5 |
| SPI2_MISO | PB14 | AF5 |
| SPI2_MOSI | PA11 | AF5 |

### SPI3 -- Full-Duplex Master (DRV8323 Gate Driver)

Configured as a 16-bit master for configuring the DRV8323 gate driver over SPI.

| Parameter | Value |
|---|---|
| Mode | MASTER |
| Direction | 2 lines (full duplex) |
| Data size | 16 bits |
| Clock polarity (CPOL) | LOW |
| Clock phase (CPHA) | 2 EDGE (capture on second edge) |
| Baud rate prescaler | /32 |
| Calculated baud rate | 5.3125 MBits/s |
| NSS | Software managed (CS on PC13 via GPIO) |
| First bit | MSB |
| TI mode | DISABLE |
| CRC calculation | DISABLE |

**GPIO mapping:**

| Signal | Pin | Alternate function |
|---|---|---|
| SPI3_SCK | PC10 | AF6 |
| SPI3_MISO | PC11 | AF6 |
| SPI3_MOSI | PB5 | AF6 |

### I2C2 -- Fast Mode (AS5600 Absolute Encoder)

Configured for communication with the AS5600 magnetic absolute encoder. The AS5600 supports Fast Mode I2C up to 400 kHz.

| Parameter | Value |
|---|---|
| Speed mode | FAST |
| Timing register | 0x40621236 |
| Addressing mode | 7-bit |
| Own address 1 | 0 |
| Dual address | DISABLE |
| General call | DISABLE |
| No stretch | DISABLE |
| Analog filter | ENABLE |
| Digital filter | 0 |
| Clock source | PCLK1 (170 MHz) |

**GPIO mapping:**

| Signal | Pin | Alternate function |
|---|---|---|
| I2C2_SCL | PC4 | AF4 (open-drain) |
| I2C2_SDA | PF0 (OSC_IN) | AF4 (open-drain) |

### ADC1 -- Analog-to-Digital (Master, Dual with ADC2)

Configured as the master in a dual ADC regular parallel mode, triggered by TIM2 TRGO for synchronized current sampling. Used in injected mode by the FOC loop.

| Parameter | Value |
|---|---|
| Clock prescaler | SYNC_PCLK_DIV4 (42.5 MHz from 170 MHz APB2) |
| Resolution | 12-bit |
| Data alignment | RIGHT |
| Scan mode | ENABLE |
| Continuous conversion | DISABLE |
| Number of conversions | 2 |
| External trigger | TIM2 TRGO (ADC_EXTERNALTRIG_T2_TRGO) |
| External trigger edge | RISING |
| Overrun | DATA PRESERVED |
| Oversampling | DISABLE |
| Multi-mode | DUAL REGULAR PARALLEL (ADC1=master) |

**Regular conversion sequence:**

| Rank | Channel | Pin | Sampling time | Signal type |
|---|---|---|---|---|
| 1 | ADC_CHANNEL_4 | PA3 | 47.5 cycles | SINGLE-ENDED |
| 2 | ADC_CHANNEL_12 | PB1 | 47.5 cycles | SINGLE-ENDED |

**Conversion mapping (via code injection):**

| Index in `hadc1.InjectionData[]` | Channel | Pin | Variable | Signal |
|---|---|---|---|---|
| 0 (va) | ADC_CHANNEL_4 | PA3 | Ia | Phase A current |
| 2 (vc) | ADC_CHANNEL_12 | PB1 | Ic | Phase C current |

### ADC2 -- Analog-to-Digital (Slave, Dual with ADC1)

Configured as the slave in dual ADC mode, triggered by TIM2 TRGO. Used in injected mode by the FOC loop.

| Parameter | Value |
|---|---|
| Clock prescaler | SYNC_PCLK_DIV4 (42.5 MHz from 170 MHz APB2) |
| Resolution | 12-bit |
| Data alignment | RIGHT |
| Scan mode | ENABLE |
| Continuous conversion | DISABLE |
| Number of conversions | 2 |
| External trigger | TIM2 TRGO (ADC_EXTERNALTRIG_T2_TRGO) |
| External trigger edge | RISING |
| Overrun | DATA PRESERVED |
| Oversampling | DISABLE |

**Regular conversion sequence:**

| Rank | Channel | Pin | Sampling time | Signal type |
|---|---|---|---|---|
| 1 | ADC_CHANNEL_3 | PA6 | 2.5 cycles | SINGLE-ENDED |
| 2 | ADC_CHANNEL_3 | PA6 | 47.5 cycles | SINGLE-ENDED |

**Conversion mapping (via code injection):**

| Index in `hadc2.InjectionData[]` | Channel | Pin | Variable | Signal |
|---|---|---|---|---|
| 1 (vb) | ADC_CHANNEL_3 | PA6 | Ib | Phase B current |

Note: ADC2 dual-rank sampling of the same channel (CH3) is a CubeMX artifact. Only `InjectionData[1]` (index `vb`) is read by the code for Ib.

### VBUS Sensing

The VBUS voltage is **hardcoded** in `main.c` as `24.0f` (watched variable `vbus`). The PA9 pin is configured as TIM1_CH2 (encoder input), NOT as an ADC input. VBUS ADC sensing (originally planned on ADC2_CH5 / PA9) is not implemented.

### GPIO

**Output pins (push-pull, LOW speed, no pull unless noted):**

| Pin | Initial state | Function |
|---|---|---|
| PC13 | LOW (GPIO_PIN_RESET) | DRV8323 CS (chip select, active low, SPI3) |
| PC14 (OSC32_IN) | LOW (GPIO_PIN_RESET) | Unused (available) |
| PC15 (OSC32_OUT) | HIGH (GPIO_PIN_SET) | DRV8323 HiZ (high-Z output control) |
| PC6 | HIGH (GPIO_PIN_SET) | nSLEEP (DRV8323 sleep control, high = awake) |
| PB0 | LOW (GPIO_PIN_RESET) | Unused (available for debug LED, etc.) |
| PB15 | LOW (GPIO_PIN_RESET) | Unused (available for debug LED, etc.) |

**Input pins with interrupt:**

| Pin | Configuration | Function |
|---|---|---|
| PB13 | Falling-edge interrupt (GPIO_MODE_IT_FALLING), internal pull-up ENABLED | DRV8323 nFAULT (fault indicator, active low) |

The PB13 nFAULT interrupt (EXTI15_10_IRQn) is enabled with preempt priority 0, sub priority 0. The ISR sets a global `fault_triggered` flag, which is polled in the main loop.

### NVIC

| Property | Value |
|---|---|
| Priority group | NVIC_PRIORITYGROUP_4 (4 bits preempt, 0 bits sub) |

**Enabled interrupt channels:**

| Interrupt | Preempt Priority | Sub Priority | Enabled by |
|---|---|---|---|
| NonMaskableInt | 0 | 0 | System |
| HardFault | 0 | 0 | System |
| MemManage | 0 | 0 | System |
| BusFault | 0 | 0 | System |
| UsageFault | 0 | 0 | System |
| SVCall | 0 | 0 | System |
| DebugMonitor | 0 | 0 | System |
| PendSV | 0 | 0 | System |
| SysTick | 15 | 0 | HAL |
| TIM6_DAC_IRQn | 0 | 0 | CubeMX (FOC loop @ 20 kHz) |
| EXTI15_10_IRQn | 0 | 0 | CubeMX (DRV8323 nFAULT) |
| SPI2_IRQn | (runtime) | (runtime) | HAL (`HAL_SPI_TransmitReceive_IT`) |

**Peripheral interrupt summary:**

| Interrupt | Source | Handler | Purpose |
|---|---|---|---|
| TIM6_DAC_IRQn | TIM6 update event | `TIM6_DAC_IRQHandler` → HAL → `FOC_ISR()` | FOC control loop at 20 kHz |
| EXTI15_10_IRQn | PB13 falling edge | `EXTI15_10_IRQHandler` → sets `fault_triggered` global | DRV8323 fault detection |
| SPI2_IRQn | SPI2 transfer events | `SPI2_IRQHandler` → HAL → `HAL_SPI_TxRxCpltCallback` → `COMM_SPI_ProcessFrame()` | Host communication |

### SysTick

SysTick is configured as the HAL time base source. The `SysTick_Handler` calls `HAL_IncTick()`. SysTick fires every 1 ms.

### HAL Modules

The following HAL modules are enabled in `stm32g4xx_hal_conf.h`:

- HAL_ADC_MODULE_ENABLED
- HAL_CORDIC_MODULE_ENABLED
- HAL_I2C_MODULE_ENABLED
- HAL_SPI_MODULE_ENABLED
- HAL_TIM_MODULE_ENABLED
- HAL_GPIO_MODULE_ENABLED
- HAL_EXTI_MODULE_ENABLED
- HAL_RCC_MODULE_ENABLED
- HAL_FLASH_MODULE_ENABLED
- HAL_PWR_MODULE_ENABLED
- HAL_CORTEX_MODULE_ENABLED

## Complete Pinout Table

All 23 physical pins assigned in the CubeMX configuration, plus 2 virtual system pins, for a total of 25 configured resources.

### Physical Pins

| Pin # | Pin Name | Function | Configuration | Alternate Function |
|---|---|---|---|---|
| 2 | PC13 | GPIO Output (DRV8323 CS) | Push-pull, LOW speed, no pull, initial LOW | -- |
| 3 | PC14 | GPIO Output | Push-pull, LOW speed, no pull, initial LOW | -- |
| 4 | PC15 | GPIO Output (DRV8323 HiZ) | Push-pull, LOW speed, no pull, initial HIGH | -- |
| 5 | PC4 | I2C2_SCL (AS5600) | Alternate function open-drain, no pull | AF4 |
| 7 | PC6 | GPIO Output (nSLEEP) | Push-pull, LOW speed, no pull, initial HIGH | -- |
| 9 | PF0 | I2C2_SDA (AS5600) | Alternate function open-drain, no pull | AF4 |
| 10 | PF1 | SPI2_SCK | Alternate function push-pull, no pull | AF5 |
| 11 | PA0 | TIM2_CH1 (PWM U) | Alternate function push-pull, no pull | AF1 |
| 12 | PA1 | TIM2_CH2 (PWM V) | Alternate function push-pull, no pull | AF1 |
| 13 | PA2 | TIM2_CH3 (PWM W) | Alternate function push-pull, no pull | AF1 |
| 14 | PA3 | ADC1_IN4 (Ia) | Analog, no pull | -- |
| 17 | PA6 | ADC2_IN3 (Ib) | Analog, no pull | -- |
| 19 | PA8 | TIM1_CH1 (Encoder A) | Alternate function push-pull, no pull | AF6 |
| 20 | PA9 | TIM1_CH2 (Encoder B) | Alternate function push-pull, no pull | AF6 |
| 22 | PA11 | SPI2_MOSI | Alternate function push-pull, no pull | AF5 |
| 23 | PB0 | GPIO Output | Push-pull, LOW speed, no pull, initial LOW | -- |
| 24 | PB1 | ADC1_IN12 (Ic) | Analog, no pull | -- |
| 28 | PB5 | SPI3_MOSI (DRV8323) | Alternate function push-pull, no pull | AF6 |
| 37 | PC10 | SPI3_SCK (DRV8323) | Alternate function push-pull, no pull | AF6 |
| 38 | PC11 | SPI3_MISO (DRV8323) | Alternate function push-pull, no pull | AF6 |
| 40 | PB13 | EXTI13 (nFAULT) | Floating input with pull-up, falling-edge interrupt | -- |
| 41 | PB14 | SPI2_MISO | Alternate function push-pull, no pull | AF5 |
| 42 | PB15 | GPIO Output | Push-pull, LOW speed, no pull, initial LOW | -- |

### Virtual / System Pins

| Reference | Function |
|---|---|
| VP_SYS_VS_Systick | SysTick timer as HAL time base |
| VP_SYS_VS_DBSignals | Disable dead battery signals (SYS configuration) |

## Clock Tree Diagram (Text)

```
HSI 16 MHz
  |
  v
PLLM (/4) --> 4 MHz
  |
  v
PLLN (x85) --> 340 MHz VCO
  |
  v
PLLR (/2) --> 170 MHz SYSCLK --> HCLK (AHB) 170 MHz
                                    |
                                    +--> APB1 (PCLK1) 170 MHz
                                    |       +--> I2C2
                                    |       +--> TIM6
                                    |
                                    +--> APB2 (PCLK2) 170 MHz
                                            +--> TIM1
                                            +--> TIM2
                                            +--> SPI2
                                            +--> SPI3
                                            +--> ADC1 (via /4 prescaler --> 42.5 MHz)
                                            +--> ADC2 (via /4 prescaler --> 42.5 MHz)
```

## Known Issues / Observations

1. **TIM2 ARR at 32-bit max (0xFFFFFFFF):** ⚠️ **CORREGIDO** — The auto-reload was set to its maximum 32-bit value (~0.02 Hz). It has been updated to 4250 for 20 kHz PWM switching. This was corrected via CubeMX (`.ioc` → `TIM2.PeriodNoDither=4250`).

2. **ADC2 dual-rank oversampling of CH3:** Both regular conversion ranks in ADC2 sample the same channel (CH3 / PA6 / Ib). The code only reads `hadc2.InjectionData[1]`. This is harmless but redundant.

3. **VBUS hardcoded:** The VBUS voltage is hardcoded to 24.0V in `main.c`. No ADC channel is configured for VBUS sensing. If real VBUS measurement is needed, PA9 (currently TIM1_CH2) would need to be reconfigured, or a different pin assigned.

4. **SPI2 interrupt enabled at runtime:** SPI2_IRQn is not configured in CubeMX's NVIC table but is enabled at runtime by `HAL_SPI_TransmitReceive_IT()`. This is valid HAL behavior but should be noted if debugging interrupt-related issues.
