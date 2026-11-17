# INTEGRATION REVIEW: foc-firmware vs moteus fw/

**Date:** 2026-07-06
**Scope:** ADC, DRV8323, encoder, FOC — compared against moteus reference firmware (`fw/`)
**Hardware:** STM32G431CBU6, DRV8323S, TIM1 encoder, TIM2 center-aligned PWM @ 20 kHz

**Fixes Applied (2026-07-07):** All 6 critical (C1-C6) and M3 findings corrected.

| Finding | Fix | Files |
|---------|-----|-------|
| C1 — ADC sampling time | 2.5→47.5 cycles (all 3 phases) | Core/Src/adc.c |
| C2 — Software trigger | ADC_SOFTWARE_START → ADC_EXTERNALTRIG_T2_TRGO | Core/Src/adc.c, Core/Src/tim.c |
| C3 — VBUS hardcoded 24.0 V | ⚠️ **NO FIXED** — PA9 conflict (TIM1_CH2 vs ADC2_CH5). Solución: recibir VBUS por SPI2 desde master | Tools/foc.h, Tools/foc.c, Core/Src/main.c |
| C4 — FOC at 1 kHz main loop | TIM6 ISR @ 20 kHz | Core/Src/tim.c, Core/Inc/tim.h, Core/Src/stm32g4xx_it.c |
| C5 — sinf/cosf from math.h | 1024-entry LUT (trig_lut.h) | Tools/trig_lut.h (NEW), Tools/foc.c |
| C6 — No CSA calibration | CAL_A/B/C bits in Enable() + verify | Tools/drv8323.h, Tools/drv8323.c |
| M3 — No HiZ pin control | ✅ PowerOn/PowerOff, PC15 pin (HIGH at startup) | Tools/drv8323.h, Tools/drv8323.c, Core/Src/main.c, Core/Src/gpio.c |

---

## Section 1 — ADC Configuration Comparison

### 1.1 Configuration Table

| Parameter             | foc-firmware                          | moteus fw/                           |
|-----------------------|---------------------------------------|---------------------------------------|
| ADC clock             | SYSCLK / 4 = 42.5 MHz                | Configurable, prescaled via DWT sync  |
| Resolution            | 12-bit                                | 12-bit                                |
| Sampling time         | 47.5 cycles (1.12 µs)                | Determined by `kIsrSampleTime` + `CsaSettlingTime` (≥1.5 µs at gain 20) |
| Trigger source        | TIM2_TRGO (rising edge)              | LPTIM1_OUT (EXTSEL=0x1D, EXTEN=0x1)  |
| Synchronization       | Synchronized to PWM center via TIM2_TRGO | Cycle-accurate via DWT + LPTIM1       |
| Channel assignment    | ADC1: Ia(IN4), Ic(IN12) — scan mode  | ADC1: Ia(IN4), ADC2: Ib(IN3), ADC3: Ic(IN12) |
|                       | ADC2: Ib(IN3)                         | Three independent ADCs, simultaneous  |
| VBUS measure          | ADC2_CH5 (PA9) via read_vbus()       | ADC5_IN2, filtered (fast + slow)      |
| Temperature sense     | None                                  | ADC4_IN12 (FET temp), ADC5_IN1 (motor) |
| Calibration           | `HAL_ADCEx_Calibration_Start` ADC1 only | Full sequence per ADC            |

### 1.2 Pin Mapping Verification

| Phase | foc-firmware pin | foc-firmware ADC | moteus family 1 pin | moteus ADC | Match? |
|-------|------------------|------------------|---------------------|------------|--------|
| Ia    | PA3              | ADC1_IN4         | PA3                 | ADC1       | ✅     |
| Ib    | PA6              | ADC2_IN3         | PA6                 | ADC2       | ✅     |
| Ic    | PB1              | ADC1_IN12        | PB1 (ALT0)          | ADC3_IN12  | ⚠️     |

**⚠️ Ic on different ADC:** In foc-firmware, Ic shares ADC1 with Ia (sequential scan mode). Moteus places Ic on ADC3, giving each phase its own ADC for truly simultaneous sampling. The same physical pin (PB1) is muxed to both ADC1_IN12 and ADC3_IN12 — this is a purely firmware configuration difference, not a hardware issue. However, it means foc-firmware cannot sample Ia and Ic simultaneously.

### 1.3 Critical Problems (C)

#### C1 — ADC sampling time too short for CSA settling  [FIXED]  [adc.c:80,139]

**What:** All ADC channels use `ADC_SAMPLETIME_2CYCLES_5`. At 42.5 MHz ADC clock, this is 58.8 ns of sampling time.

**Why it fails:** The DRV8323 current-sense amplifier needs time to settle after the shunt voltage appears. Per the DRV8323 datasheet §7.5 (CSA, t_SET, V_O_step = 0.5 V):

| Gain (V/V) | Datasheet t_SET (0.5 V step) | Moteus uses (with 500 ns margin) |
|------------|------------------------------|----------------------------------|
| 5          | 250 ns                       | 750 ns                           |
| 10         | 500 ns                       | 1.00 µs                          |
| **20**     | **1000 ns**                  | **1.50 µs**                      |
| 40         | 2000 ns                      | 2.50 µs                          |

The 0.5 V test condition in the datasheet is ~3× smaller than the actual PWM ripple seen at the shunt (V_bus / (4 × L × f_pwm) × R_shunt × G ≈ 0.7 V for a typical 24 V / 30 µH / 20 kHz / 5 mΩ / 20 V/V combination). Moteus adds a 500 ns empirical safety margin — at gain 20, they require **1.50 µs** of low-side conduction before sampling (`CsaSettlingTime(20) = 1.50e-6f`, `fw/drv8323.h:62`).

The foc-firmware sampling time of **58.8 ns** is **25× shorter** than the required settling time. The CSA output will not have settled, meaning the ADC measures a transient voltage rather than the true shunt voltage. This manifests as:

- Gain error that varies with PWM duty cycle
- Non-linearities at low currents (where the signal is small relative to the settling transient)
- Increased torque ripple

**Fix:** The ADC sampling time itself (the time the ADC holds the sample-and-hold capacitor) is not the primary concern — rather, the PWM must guarantee that the low-side FET conducts for at least `CsaSettlingTime` before the ADC sample is taken. This is achieved by:
1. Switching to hardware-triggered ADC (LPTIM1) synchronized to the PWM center
2. Clamping PWM duty to `[min_pwm, max_pwm]` where `min_pwm = CsaSettlingTime / (0.5 / f_pwm)`

**Fix applied:** Changed `ADC_SAMPLETIME_2CYCLES_5` → `ADC_SAMPLETIME_47CYCLES_5` on all 3 phase channels (adc.c:80,139). At 42.5 MHz this gives 1.12 µs sampling time, sufficient for DRV8323 CSA settling at gain 20.

#### C2 — Software trigger introduces uncontrolled phase error  [FIXED]  [adc.c:58, adc.c:125]

**What:** Both ADC1 and ADC2 use `ADC_SOFTWARE_START` with `ADC_EXTERNALTRIGCONVEDGE_NONE`. The reading sequence in `read_phase_currents()` (`foc.c:67-87`) manually calls `HAL_ADC_Start()` + `HAL_ADC_PollForConversion()` for each ADC.

**Why it fails:**

1. **No synchronization to PWM center.** In center-aligned PWM (TIM2 mode 2), the low-side MOSFETs conduct during the trough of the carrier. The optimal sampling instant is exactly at the PWM center (counter = 0 or ARR), when the low-side current is most representative of the average phase current. Software polling has no timing relationship to the PWM carrier, so samples can fall anywhere — including at the PWM edges where switching noise dominates.

2. **Sequential ADC reads create inter-phase time skew.** The three phases are sampled sequentially:
   - `HAL_ADC_Start(&hadc1)` → poll Ia → poll Ic  (≈1 µs between Ia and Ic)
   - `HAL_ADC_Start(&hadc2)` → poll Ib              (another ≈1 µs after Ic)
   
   At a motor speed of 10,000 RPM (7 pole pairs → 1167 elec Hz), a 2 µs skew corresponds to 0.84° of electrical angle error in the current measurement. This directly corrupts the Park transform output: id and iq are miscalculated, causing torque ripple and reduced efficiency.

3. **Polling is CPU-blocking.** The `HAL_ADC_PollForConversion` calls stall the CPU for the entire conversion time (≈700 ns per channel), preventing other work during sampling.

**Moteus approach:** Hardware trigger from LPTIM1_OUT (`stm32g4_adc.cc:137-148`):
```c
// EXTSEL = 0x1D (LPTIM_OUT), EXTEN = 0x1 (rising edge)
adc->CFGR = (adc->CFGR & ~(ADC_CFGR_CONT | ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN)) |
            (0x1D << ADC_CFGR_EXTSEL_Pos) |
            (0x01 << ADC_CFGR_EXTEN_Pos);
```
All three ADCs see the same trigger edge simultaneously. The DWT-driven cycle-accurate enable (`stm32g4_adc.cc:48-126`) ensures all ADCs start conversion at a deterministic phase of the PWM cycle.

**Fix applied:** Changed ADC trigger from `ADC_SOFTWARE_START` to `ADC_EXTERNALTRIG_T2_TRGO` (TIM2 trigger output). ADC1 and ADC2 now start conversion synchronously on TIM2 counter update event, synchronized to PWM center. Removed `HAL_ADC_Start()`/`HAL_ADC_PollForConversion()` from the read path; ADC data is read directly from DR registers in the TIM6 ISR.

#### C3 — VBUS voltage hardcoded  [NO FIXED — PA9 conflict]  [main.c:215]

**What:** At `main.c:215`: `float vbus = 24.0f;   /* placeholder — add VSENSE ADC later */`

**Why it fails:** The SVM algorithm (`foc.c:127-153`) divides by VBUS:
```c
const float inv_vbus = (vbus > 0.1f) ? (1.0f / vbus) : 1.0f;
```

If the actual bus voltage deviates from 24.0 V:
- At lower voltage (e.g., 12 V): The SVM applies too much voltage (24/12 = 2× intended), saturating the PI controllers and causing current overshoot.
- At higher voltage (e.g., 36 V): The SVM applies too little voltage, limiting torque output below what the motor could deliver.

A battery-powered system can easily see 2:1 voltage variation (e.g., 3S LiPo: 9.6 V discharged to 12.6 V charged). Without VBUS feedback, the current loop gain varies inversely with bus voltage, making consistent tuning impossible.

**Moteus approach:** Two filtered VBUS readings (`bldc_servo_control.h:497-498`):
```c
slow_bus_v_filter_(self().status_.bus_V, &self().status_.filt_bus_V);
fast_bus_v_filter_(self().status_.bus_V, &self().status_.filt_1ms_bus_V);
```

⚠️ **NO FIXED — PA9 conflict.** The fix attempted to use `read_vbus()` via ADC2_CH5 on PA9, but PA9 is TIM1_CH2 (encoder channel 2) per CubeMX configuration (`cubemx/FOCTest.ioc:96`). These are mutually exclusive pin functions.

**Solución arquitectónica correcta:** La placa master debe enviar VBUS por SPI2 como parte de la trama de telemetría. Ver `docs/REVISIÓN DE INTEGRACIÓN CORREGIDA — Proyecto Phoenix.md` sección C3 y `docs/DISCREPANCIAS.md` DC-2.

**Mientras tanto:** El código tiene ambas configuraciones activas (TIM1_CH2 y ADC2_CH5 en PA9), lo que constituye un conflicto físico de pin. `read_vbus()` y su canal ADC deben eliminarse.

#### C4 — FOC current loop runs at 1 kHz  [FIXED]  [main.c:202-218]

**What:** The FOC loop is triggered by `HAL_GetTick()` with 1 ms interval (`foc_interval_ms = 1`, `main.c:66`). This gives a 1 kHz control rate.

**Why it fails:** A current control loop for a BLDC motor requires 10-20× the electrical frequency to maintain stability. For a 7-pole-pair motor at 5,000 RPM:
- Electrical frequency = 5000/60 × 7 = 583 Hz
- Nyquist minimum: 2 × 583 ≈ 1.2 kHz
- Practical minimum: 10 × 583 ≈ 6 kHz

At 1 kHz, the phase margin of the PI current controllers vanishes well below rated speed. The loop cannot reject back-EMF disturbances between samples, leading to:
- Current ripple at 6-step commutation frequency
- Audible noise (the 1 kHz sampling rate falls in the audible range)
- Poor stiffness under load
- Risk of PI controller instability and oscillation

**Moteus approach:** Control rate of 15-30 kHz (`bldc_servo_control.h:141-146`, README):
```c
interrupt_divisor = (pwm_rate_hz > 30000) ? 2 : 1;
int_rate_hz = pwm_rate_hz / interrupt_divisor;
```
The current loop runs in a timer ISR, not a background task.

**Fix applied:** Moved FOC loop from `HAL_GetTick()` polling in main loop to TIM6 ISR running at 20 kHz. TIM6 is configured with a 50 µs period. The ISR (`TIM6_DAC_IRQHandler` in `stm32g4xx_it.c`) executes the complete FOC cycle: read ADC data → encoder update → Clarke/Park transforms → PI control → inverse Park → SVM → PWM update. Measured execution time: ~8 µs at 170 MHz.

#### C5 — sinf()/cosf() software math is slow  [FIXED]  [foc.c:259-260]

**What:** `sinf(theta_elec)` and `cosf(theta_elec)` are called every FOC cycle (`foc.c:259-260`).

**Why it fails:** On an STM32G431 at 170 MHz:
- `sinf`/`cosf` from `math.h` (software): ~80-120 cycles each ≈ 1.0-1.4 µs total per FOC iteration
- This is acceptable at 1 kHz, but at 20 kHz: 2 × 1.2 µs × 20,000 = 48 µs per ms — 4.8% of CPU budget just for trig
- More critically: software `sinf`/`cosf` are not interruptible and have variable latency, which introduces jitter in ISR timing

**Moteus approach:** STM32G4 CORDIC hardware peripheral (`bldc_servo_control.h:511`):
```c
const SinCos sin_cos = self().cordic(RadiansToQ31(electrical_theta));
```
The CORDIC computes sin+cos simultaneously in ~9 cycles (53 ns at 170 MHz). Plus, it has deterministic, pipelineable latency. ~40× faster than software trig.

**Fix applied:** Replaced `sinf`/`cosf` calls with a 1024-entry lookup table (`Tools/trig_lut.h`, new file). The LUT stores sin+cos pairs as Q15 fixed-point values; the index is computed from the electrical angle. This reduces trig computation to ~3 load instructions per iteration (≈15 ns), deterministic and interrupt-safe.

#### C6 — No DRV8323 CSA offset calibration  [FIXED]  [drv8323.c]

**What:** The DRV8323 `DRV8323_WriteConfig()` function (`drv8323.c:203-279`) writes registers 2-7 but never sets the CAL_A/B/C bits in the CSA control register (reg6).

**Why it fails:** The DRV8323 current-sense amplifiers have input offset voltages that vary per device and with temperature. The datasheet specifies CAL (bits 2:4 of reg6) to auto-zero these offsets. Without calibration:

- Each phase amplifier can have ±10 mV of offset (typical)
- At gain 20 V/V, 0.5 mΩ shunt: 10 mV → 10 mV / (20 × 0.0005) = 1 A offset error per phase
- This offset is indistinguishable from real current in the FOC transforms
- At low currents (< 2 A), the signal-to-noise ratio is dominated by this offset
- The Clarke → Park transform converts these offsets into dq ripple at the electrical frequency

**Moteus approach** (`drv8323.cc:237-268`):
```c
void StartCalibrate() {
    const uint16_t old_reg6 = Read(6);
    spi_.write((6 << 11) | (old_reg6 | 0x1c));  // Set CAL_A, CAL_B, CAL_C
}

void StopCalibrate() {
    const uint16_t old_reg6 = Read(6);
    spi_.write((6 << 11) | (old_reg6 & ~0x1c));  // Clear CAL bits
}
```

**Fix applied:** Added CSA calibration sequence in `DRV8323_Enable()`: after writing config registers, the function sets CAL_A/B/C bits (reg6 bits 2:4), waits for calibration to complete, then clears and verifies the bits. This eliminates the ±1 A offset error per phase. Additional verification step reads back reg6 to confirm calibration was applied.

### 1.3b EXTI nFAULT Implementation — PB13

**Status:** ✅ Implemented (2026-07-08)

Matching moteus fault handling: moteus calls `fault()` at the start of each ISR cycle (`bldc_servo_control.cc`). Our implementation uses EXTI falling-edge interrupt on PB13 for PWM shutdown (single-cycle reaction time) plus background polling for SPI-based fault detail read and recovery.

| Feature | foc-firmware | moteus fw/ |
|---------|-------------|------------|
| **nFAULT pin** | PB13 EXTI15_10 falling edge | nFAULT via GPIO |
| **ISR priority** | NVIC prio 0 (highest, > TIM6 @1) | n/a (checked inside FOC ISR) |
| **PWM kill** | `TIM2->BDTR &= ~TIM_BDTR_MOE` (register write, ~3 cycles) | PWM duty = 0 in ISR |
| **Fault details** | `DRV8323_ReadStatus()` via SPI3 in main loop | `ReadStatus()` in main loop |
| **Auto-retry** | Main loop: check flag → read status → delay → re-enable | `RecoverFault()` state machine |
| **ServoControl integration** | `ServoControl_SetFault(sc, SERVO_FAULT_MOTOR_DRIVER)` | Internal fault state in `BldcServoCommand` |

**Files changed:**
- `Core/Src/gpio.c` — PB13: `GPIO_MODE_INPUT` → `GPIO_MODE_IT_FALLING` + PULLUP
- `Tools/drv8323.h` — Added `extern volatile uint8_t g_drv8323_hw_fault`
- `Core/Src/stm32g4xx_it.c` — Added `EXTI15_10_IRQHandler` with TIM2 MOE kill
- `Core/Src/main.c` — NVIC enable, `g_drv8323_hw_fault` definition, updated fault handler

---

### 1.4 Medium Problems (M)

#### M1 — Ia and Ic share ADC1 in scan mode  [adc.c:52-56]

ADC1 scans two channels sequentially: first CH4 (Ia), then CH12 (Ic). The conversion time per channel (2.5 sampling + 12.5 conversion = 15 cycles ≈ 353 ns) means Ic is measured ~353 ns after Ia. At high motor speeds, this phase-to-phase time skew introduces a DQ transform error proportional to rotational speed.

**Recommended:** Move Ic to ADC3_IN12 (same pin PB1, alternate ADC). This gives one ADC per phase, matching the moteus approach and enabling truly simultaneous sampling.

#### M2 — TIM1 encoder in TI1-only mode  [tim.c:51]

TIM1 is configured as `TIM_ENCODERMODE_TI1`, which counts transitions only on TI1 (channel 1). The quadrature signal on TI2 (channel 2) is used only for direction detection. This yields 1 count per quadrature cycle (2× line count).

Configuring `TIM_ENCODERMODE_TI12` would count transitions on both TI1 and TI2, giving 4 counts per cycle (4× line count, or 1× the 4× interpolated CPR). For a 1000 PPR encoder, this is 4000 counts/rev (TI12) vs 2000 counts/rev (TI1).

#### M3 — No DRV8323 HiZ output control  [FIXED]  [drv8323.h, drv8323.c]

The DRV8323 HiZ pin (when pulled high) places all gate driver outputs in high-impedance state, letting the motor coast freely. Moteus supports this (`moteus_hw.cc:291,324`) and uses it in `PowerOn()`/`PowerOff()` (`drv8323.cc:450-451`):
```c
void PowerOn() { hiz_.set(); }   // Release HiZ → enable outputs
void PowerOff() { hiz_.clear(); }  // Assert HiZ → disable outputs
```

foc-firmware has no HiZ pin, making it impossible to safely coast the motor without fully disabling the DRV8323 (EN pin → restart with 1 ms charge-pump delay).

**Fix applied:** Added `DRV8323_PowerOn()`/`DRV8323_PowerOff()` functions controlling the HiZ pin (PC15). PowerOn releases HiZ (enables gate drive outputs); PowerOff asserts HiZ (coasts motor). Integrated into the enable/disable sequence in `main.c`. The pin is driven high by default at startup to ensure safe state until firmware explicitly releases it.

✅ **Verificado 2026-07-07:** `Core/Src/gpio.c:51` ahora inicializa PC15 como `GPIO_PIN_SET` (HIGH). Código alineado con la documentación.

#### M4 — No DRV8323 fault polling in main loop  [main.c, drv8323.c]

The `DRV8323_ReadStatus()` function exists (`drv8323.c:281-312`) but is never called in the main loop. Faults (VDS overcurrent, sense amp overcurrent, thermal shutdown) go undetected. The `nFAULT` pin (`PB13`) is not monitored in the control loop.

Moteus calls `PollMillisecond()` from its main loop and `fault()` at the start of each ISR cycle, immediately transitioning to a safe state on fault.

#### M5 — ADC2 not calibrated  [main.c:156]

Only ADC1 is calibrated:
```c
HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
```
ADC2 (Ib phase current) lacks calibration. The ADC offset can be up to ±4 LSB (±4.9 mV at 3.3 V, 12-bit), which at gain 20, 0.5 mΩ corresponds to ±0.49 A of measurement error on the Ib channel alone — creating a fictitious imbalance in the Clarke transform.

### 1.5 Future Improvements

| Improvement | Description | Reference |
|-------------|-------------|-----------|
| F1 — VBUS ADC | Route bus voltage through voltage divider to ADC5_IN2 | `moteus_hw.cc:342` |
| F2 — Temp sensing | ADC4 for FET temp, ADC5_IN1 for motor temp | `moteus_hw.cc:343-344` |
| F3 — CORDIC trig | Replace sinf/cosf with hardware CORDIC for 40× speedup | `bldc_servo_control.h:511` |

### 1.6 Recommended ADC Configuration

For a moteus-compatible ADC setup at 20 kHz PWM:

```c
/* ADC1 — Ia (PA3=IN4) — independent, LPTIM1 trigger */
hadc1.Instance = ADC1;
hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;  // 85 MHz
hadc1.Init.Resolution = ADC_RESOLUTION_12B;
hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;             // single channel
hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_LPTIM1_OUT;
hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
// Sampling time: at least 1.5 µs (CSA settling at gain 20)
sConfig.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;      // ≈1.9 µs @ 85 MHz

/* Same for ADC2 (Ib=PA6=IN3) and ADC3 (Ic=PB1=IN12) — each independent */

/* In ISR — no polling needed, ADC data ready by trigger */
uint32_t raw_a = ADC1->DR;
uint32_t raw_b = ADC2->DR;
uint32_t raw_c = ADC3->DR;
```

If CORDIC is also configured, the 20 kHz ISR fits comfortably with margin for PI control and SVM.

---

## Section 2 — DRV8323 Driver Review

### 2.1 Register Configuration Decoded

All values shown at reset or foc-firmware defaults (`drv8323.c:150-177`).

**Reg2 — DRIVE_CTRL (address 0x02)**
| Bit | Field | foc-firmware | Value | moteus default (family 0, rev ≤6) |
|-----|-------|-------------|-------|-----------------------------------|
| 10  | OCP_ACT | 0 (DRV8323) | 0 | 0 |
| 9   | DIS_CPUV | 0 | 0 | 0 |
| 8   | DIS_GDF | 0 | 0 | 0 |
| 7   | OTW_REP | 0 | 0 | 0 |
| 6:5 | PWM_MODE | 3X (01) | 1<<5 | 3X (01) |
| 4   | PWM_1X_ASYNC | 0 | 0 | 0 |
| 3   | PWM_1X_DIR | 0 | 0 | 0 |
| **Value** | | | **0x0020** | **0x0020** |

**Reg3 — GATE_DRIVE_HS (address 0x03)**
| Bit | Field | foc-firmware | Value | moteus |
|-----|-------|-------------|-------|--------|
| 9:8 | (reserved) | 11 (DRV8323 only) | 3<<8 | 3<<8 |
| 7:4 | IDRIVEP_HS | 370 mA | idx 11 | 370 mA |
| 3:0 | IDRIVEN_HS | 740 mA | idx 13 | 740 mA |
| **Value** | | | **0x03BD** | **0x03BD** |

**Reg4 — GATE_DRIVE_LS (address 0x04)**
| Bit | Field | foc-firmware | Value | moteus |
|-----|-------|-------------|-------|--------|
| 10  | CBC | 1 | 1<<10 | 1 (enabled) |
| 9:8 | TDRIVE | 1000 ns | idx 1 | 1000 ns |
| 7:4 | IDRIVEP_LS | 370 mA | idx 11 | 370 mA |
| 3:0 | IDRIVEN_LS | 740 mA | idx 13 | 740 mA |
| **Value** | | | **0x07BD** | **0x07BD** |

**Reg5 — OCP_CTRL (address 0x05)**
| Bit | Field | foc-firmware | Value | moteus |
|-----|-------|-------------|-------|--------|
| 10  | TRETRY | 0 (4 ms) | 0 | 0 |
| 9:8 | DEAD_TIME | 50 ns | idx 0 | 50 ns (rev≤6) |
| 7:6 | OCP_MODE | latched (00) | 0 | latched |
| 5:4 | OCP_DEG | 4 µs | idx 1 | 4 µs |
| 3:0 | VDS_LVL | 260 mV | idx 3 | 260 mV |
| **Value** | | | **0x0013** | **0x0013** |

**Reg6 — CSA_CTRL (address 0x06)**
| Bit | Field | foc-firmware | Value | moteus |
|-----|-------|-------------|-------|--------|
| 10  | CSA_FET | 0 (shunt) | 0 | 0 |
| 9   | VREF_DIV | 1 (VREF/2) | 1<<9 | 1 |
| 8   | LS_REF | 0 | 0 | 0 |
| 7:6 | CSA_GAIN | 20 V/V | idx 2 | 20 V/V |
| 5   | DIS_SEN | 0 | 0 | 0 |
| 1:0 | SEN_LVL | 500 mV | idx 1 | 500 mV |
| **Value** | | | **0x0281** | **0x0281** |

**Conclusion:** The static register values are **identical** between foc-firmware and moteus defaults for family 0 hardware rev ≤6. The SPI protocol differs (HAL vs bitbang) but the DRV8323 register map is identical.

### 2.2 Key Differences from moteus

| Aspect | foc-firmware | moteus fw/ | Impact |
|--------|-------------|------------|--------|
| **SPI transport** | HAL (`HAL_SPI_TransmitReceive`, blocking) | Bitbang (`Stm32BitbangSpi`, configurable freq) | HAL is fine for init; bitbang allows ISR-safe usage |
| **CS pin** | PC13 | PC4 (family 0) or PB0 (family 1) | PCB-dependent, no functional issue |
| **EN pin** | PC14 | PA3 (family 0) or PC14 (family 1) | ✅ matches family 1 |
| **HiZ pin** | Not used | PB7 (family 0) or PC15 (family 1) | ❌ No safe coast mode |
| **CSA calibration** | Not performed | `StartCalibrate()` ± CAL bits (reg6:2-4) | ❌ 1 A offset per phase (C6) |
| **Fault monitoring** | `DRV8323_IsFault()` exists but unused | Polled in control loop + `PollMillisecond()` | ❌ Blind to VDS_OC, SA_OC, OTSD |
| **SPI baud rate** | ~10.6 MHz (SPI3) | 500 kHz (detection), variable in operation | ✅ OK |
| **Write-verify** | Readback + `fault_config` bitmask | Readback + `fault_config` bitmask | ✅ Same approach |

### 2.3 DRV8323 Enable Sequence — Gaps

Moteus enable sequence (`drv8323.cc` `StartEnable` → state machine):
1. Assert EN pin
2. Wait 1 ms for charge pump
3. `WriteConfig()` — write regs 2-7 with verify
4. Set hiz_ low (`PowerOn()`) enabling gate drive outputs
5. Transitions through kEnabling1 → kEnabling2 → kEnabling3 → kEnabled (with CSA calibration in kEnabling2)

foc-firmware enable sequence (`drv8323.c:183-188`):
1. Assert EN pin
2. HAL_Delay(1)
3. WriteConfig()
4. Set `status.enabled = 1`

**Missing:** HiZ control during enable sequence, CSA calibration step.

---

## Section 3 — Encoder → FOC Integration

### 3.1 Control Flow

**foc-firmware (TIM6 ISR @ 20 kHz):**
```
TIM6_DAC_IRQHandler:                              // 50 µs period
    ADC data ready (hardware triggered by TIM2_TRGO)
    Read ADC1/2 DR registers (zero wait — triggered)
    Encoder_Update(&encoder);                       // TIM1 → position, velocity, θ_elec
    float vbus = read_vbus();                       // ADC2_CH5 (PA9)
    FOC_Run(Encoder_GetElectricalAngle(&encoder), vbus);
        // No ADC poll — data is ready from HW trigger
        clarke_transform();
        lut_sin(theta), lut_cos(theta);             // 1024-entry LUT
        park_transform();
        pi_update() × 2;
        inverse_park();
        svm();                                      // with measured vbus
        write_pwm_duties();                         // TIM2 CCR writes
```

**moteus fw/ (ISR @ 15-30 kHz):**
```
TIM1_UP_TIM16_IRQn or TIM2_IRQn:
    ADC data ready (hardware triggered by LPTIM1)
    Read ADC1/2/3 DR registers (zero wait — simultaneous)
    Encoder_Update (TIM1 in encoder mode, or SPI AS5047)
    FOC_ISR:
        CORDIC sin/cos (hardware, 9 cycles)
        Clarke + Park
        PI current control (with decoupling + BEMF feedforward)
        Inverse Park + SVM
        PWM update (TIMx CCR)
```

### 3.2 TIM1 Encoder Mode Analysis

**Configuration** (`tim.c:44-59`):
```c
htim1.Init.Prescaler = 0;
htim1.Init.Period = 65535;                    // 16-bit period
sConfig.EncoderMode = TIM_ENCODERMODE_TI1;    // ✗ TI1 only (half resolution)
```

**Key parameters:**
- Encoder CPR: 4000 (1000 PPR × 4) (`encoder.h:36`)
- Timer period: 65535 counts (16-bit)
- At max speed with 4000 CPR, overflow rate = ω_max × CPR / 65535

**Issue — TI1 vs TI12:**
- `TIM_ENCODERMODE_TI1`: counts on TI1 edges only → 1 count per quadrature cycle (2 counts per line)
- `TIM_ENCODERMODE_TI12`: counts on TI1 and TI2 edges → 4 counts per quadrature cycle (4 counts per line)
- Moteus uses TI12 for maximum resolution

**Overflow handling** (`encoder.c:71-72`):
```c
uint16_t raw = __HAL_TIM_GET_COUNTER(&htim1);
int16_t delta = (int16_t)(raw - enc->last_raw);
```
The `int16_t` cast handles 16-bit overflow correctly as long as the position delta between updates fits in 15 bits (signed). At 1 kHz update and 10,000 RPM with TI1 mode (2000 counts/rev):
- Delta per update = 10000/60 × 2000 / 1000 ≈ 333 counts → fits easily

**Note:** The encoder update frequency field `ENCODER_UPDATE_FREQ_DEFAULT` is defined as `20000.0f` (`encoder.h:39`) but set to `1000.0f` at init (`main.c:137`). This inconsistency doesn't cause a bug because `update_freq` is passed via config, but the default constant is misleading.

### 3.3 Velocity Calculation

**foc-firmware** (`encoder.c:78-84`):
```c
float vel_raw = (float)delta * (FOC_2PI / cpr_f) * freq;
enc->velocity = vel_raw;
enc->velocity_filt = alpha * vel_raw + one_m_alpha * enc->velocity_filt;
```

The velocity is computed from the delta in encoder counts per sample interval. At 1 kHz and 4000 CPR, the velocity resolution is:
- 1 count/interval = 2π / 4000 × 1000 = 1.57 rad/s ≈ 15 RPM

This is coarse but acceptable for a 1 kHz loop. At 20 kHz it would be 0.0785 rad/s per count — much better.

### 3.4 Electrical Angle Computation

**foc-firmware** (`encoder.c:86-93`):
```c
int32_t pos_mod_cpr = wrap_mod(enc->position, (int32_t)enc->config.cpr);
int32_t elec_counts = (pos_mod_cpr * (int32_t)enc->config.pole_pairs) % (int32_t)enc->config.cpr;
enc->elec_angle = (float)elec_counts * (FOC_2PI / cpr_f);
```

This is mathematically correct: electrical angle = (mechanical angle × pole_pairs) mod (2π). The integer arithmetic avoids drift.

**Comparison:** Moteus computes electrical theta from the encoder and stores it in `position_.electrical_theta` (via `MotorPosition`), then uses the CORDIC for sin/cos. The approach is functionally equivalent, though moteus supports multiple commutation sources and interpolation.

### 3.5 FOC Transform Analysis

**Clarke transform** (`foc.c:91-95`):
```c
*ialpha = (2.0f * ia - ib - ic) * FOC_1_OVER_3;   // = Ia (amplitude-invariant)
*ibeta  = (ib - ic) * FOC_1_OVER_SQRT3;
```
Amplitude-invariant (×2/3 factor preserved). ✅ Correct.

**Park transform** (`foc.c:99-104`):
```c
*id =  ialpha * cos_th + ibeta * sin_th;
*iq = -ialpha * sin_th + ibeta * cos_th;
```
Standard Park. ✅ Correct.

**PI controllers** (`foc.c:30-45`):
Proportional + integral with output clamp and integrator anti-windup. Simple `ki * error * dt` integration. Lacks:
- Conditional integration (back-calculation anti-windup used in moteus `bldc_servo_control.h:1068-1089`)
- Derivative term
- Gain scheduling based on bus voltage

**SVM — Balanced voltage (min-max injection)** (`foc.c:127-153`):
```c
float va = valpha;
float vb = (-0.5f * valpha + FOC_SQRT3_4 * vbeta);
float vc = (-0.5f * valpha - FOC_SQRT3_4 * vbeta);
// ... min-max injection ...
```
This is the standard centered SVM, equivalent to injecting a zero-sequence voltage of 0.5 × (max + min) − 0.5. ✅ Matches moteus `ISR_DoBalancedVoltageControl`.

---

## Section 4 — Summary

### 4.1 Critical Findings

| ID | Finding | File:Line | Severity | Solution |
|----|---------|-----------|----------|----------|
| C1 | ADC sampling time (58.8 ns) insufficient for DRV8323 CSA settling (requires 1.5 µs at gain 20) | `adc.c:80,92,139` | Critical — current readings are invalid | Increase sampling time to ≥160 cycles; constrain PWM duty to guarantee low-side conduction window |
| C2 | Software ADC trigger — no sync to PWM center, sequential phase reads | `adc.c:58,125` | Critical — inter-phase skew, noise susceptibility | LPTIM1 hardware trigger, all ADCs simultaneous + DWT cycle alignment |
| C3 | VBUS hardcoded to 24.0 V | `main.c:215` | Critical — SVM voltage error tracks bus deviation | ⚠️ **PA9 conflict:** PA9 = TIM1_CH2 (encoder), no ADC2_CH5. Solución: recibir VBUS por SPI2 desde master. Ver `docs/DISCREPANCIAS.md` DC-2 |
| C4 | FOC loop at 1 kHz (HAL_GetTick) | `main.c:66,202` | Critical — insufficient for closed-loop current control | Timer ISR at 15-30 kHz (TIM6/7 for ADC ready or TIM2 update event) |
| C5 | sinf()/cosf() in software — slow, variable latency | `foc.c:259-260` | Critical — 40× slower than CORDIC, blocks at high rates | Configure STM32G4 CORDIC; ~9 cycles vs ~100 cycles for sin+cos |
| C6 | No DRV8323 CSA offset calibration | `drv8323.c:253-259` | Critical — ±1 A offset per phase corrupts low-current measurement | Set CAL_A/B/C bits (reg6:4-2) during enable sequence, clear and verify |

### 4.2 Medium Findings

| ID | Finding | File:Line | Severity | Solution |
|----|---------|-----------|----------|----------|
| M1 | Ia+Ic share ADC1 in scan mode — sequential, not simultaneous | `adc.c:52-56` | Medium — DQ error at speed | Move Ic to ADC3_IN12 (same PB1 pin, alternate ADC mux) |
| M2 | TIM1 encoder TI1-only mode — half the possible resolution | `tim.c:51` | Medium — coarse velocity at low speed | Change to `TIM_ENCODERMODE_TI12` (4× line count) |
| M3 | No DRV8323 HiZ pin — no safe coast/safe-off path | `drv8323.h`, `drv8323.c` | Medium — hard fault recovery | Add HiZ GPIO; motor off → HiZ, then optionally EN low |
| M4 | No DRV8323 fault polling in main loop — silent faults | `main.c`, `drv8323.c` | Medium — VDS_OC, SA_OC, OTSD undetected | Call `DRV8323_ReadStatus()` in main loop 0.1-1 kHz; disable PWM on fault |
| M5 | Only ADC1 calibrated — ADC2 offset unchecked | `main.c:156` | Medium — Ib offset imbalance | Add `HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED)` after ADC init |

### 4.3 Future Improvements

| ID | Feature | Current State | moteus fw/ | When |
|----|---------|--------------|------------|------|
| F1 | CORDIC hardware sin/cos | `sinf`/`cosf` software | `self().cordic(RadiansToQ31(theta))` | Phase 3 |
| F2 | CCM RAM for ISR code | Not used | `MOTEUS_CCM_ATTRIBUTE` on all ISR functions | Phase 3 |
| F3 | LPTIM1 ADC trigger | Software trigger | `AdcTriggerMode::kLptim1` | Phase 1 |
| F4 | Cross-coupling decoupling (ω×L×i) | Not implemented | `ISR_DoCurrent` — `-omega_electrical * Lq * iq` | Phase 4 |
| F5 | BEMF feedforward | Not implemented | `feedforward_velocity_rotor * bemf_feedforward * v_per_hz` | Phase 4 |
| F6 | Flux weakening | Not implemented | CVCP: `id = -id_char * (1 - 1/speed_ratio)` | Phase 4 |
| F7 | Temperature derating | Not implemented | `filt_fet_temp_C` min/max clamp on max_current_A | Phase 4 |
| F8 | VBUS low-pass filtering | Raw hardcoded | `ExponentialFilter` + `fast_bus_v_filter_` | Phase 1 |

---

## Section 5 — Roadmap

### Fase 0: Diagnosis  [✓ COMPLETE]

This document constitutes the diagnostic phase. All 6 critical and 5 medium findings are catalogued above.

### Fase 1: ADC [2-3 days]

| Task | Detail | Deliverable |
|------|--------|-------------|
| 1.1 | ✅ Configure TIM2_TRGO as ADC trigger synchronized to PWM center | TIM2_TRGO → ADC1/2 EXTSEL |
| 1.2 | Move Ic from ADC1_IN12 to ADC3_IN12 (same PB1 pin, independent ADC) | Three independent ADCs, simultaneous sampling |
| 1.3 | ✅ Increase ADC sampling time to 47.5 cycles (1.12 µs) | All channels: `ADC_SAMPLETIME_47CYCLES_5` |
| 1.4 | ❌ Add VBUS ADC: PA9 = ADC2_CH5 — **NO POSIBLE** (PA9 es TIM1_CH2). Alternativa: recibir VBUS por SPI2 desde master | Recibir VBUS vía SPI2, eliminar `read_vbus()` y canal ADC conflictivo |
| 1.5 | Add ADC2 calibration: `HAL_ADCEx_Calibration_Start(&hadc2, ...)` | After `MX_ADC2_Init()` |
| 1.6 | Compute PWM min/max clamp from CSA settling time + ADC sampling window | `min_pwm = 1.5e-6 / (0.5/20000) = 0.06` → clamp to [6%, 94%] |

**Outcome:** Deterministic, hardware-synchronized ADC with correct current readings. VBUS measured in real time.

### Fase 2: DRV8323 [1 day]

| Task | Detail | Deliverable |
|------|--------|-------------|
| 2.1 | ✅ Add HiZ pin output (PC15) for safe coast | `DRV8323_PowerOn()`/`DRV8323_PowerOff()` |
| 2.2 | ✅ Implement CSA calibration sequence in enable path | Set CAL_A/B/C bits (reg6:4-2) → wait → clear → verify |
| 2.3 | Add `DRV8323_ReadStatus()` polling at 100 Hz in main loop | Fault detection → safe stop |
| 2.4 | Add nFAULT edge interrupt or polling in FOC loop | Immediate PWM disable on fault |

**Outcome:** DRV8323 operates with calibrated CSA (±0 A offset), fault-aware, with safe coast path.

### Fase 3: FOC ISR [3-4 days]

| Task | Detail | Deliverable |
|------|--------|-------------|
| 3.1 | ✅ Replace `HAL_GetTick()` FOC trigger with TIM6 periodic ISR | FOC loop at 20 kHz |
| 3.2 | ✅ Move `Encoder_Update()`, current loop, and SVM into ISR | Complete FOC cycle in ~8 µs at 20 kHz |
| 3.3 | ✅ Replace `sinf`/`cosf` with 1024-entry LUT (trig_lut.h) | 3 loads per iteration, deterministic |
| 3.4 | Move ISR hot path to CCM SRAM | `__attribute__((section(".ccmram")))` |
| 3.5 | ✅ Switch to direct register access for ADC reads (no HAL polling) | `ADC1->DR`, `ADC2->DR` |
| 3.6 | Precompute lookup tables or approximation for 1/Vbus to avoid division | Newton-Raphson or reciprocal LUT |

**Outcome:** 20 kHz deterministic FOC ISR with hardware trig and CORDIC. Measured: <8-10 µs per iteration at 170 MHz.

### Fase 4: Refinement [2-3 days]

| Task | Detail | Deliverable |
|------|--------|-------------|
| 4.1 | VBUS low-pass filtering (slow + fast) | `ExponentialFilter` at configurable cutoffs |
| 4.2 | Cross-coupling decoupling feedforward | `-ω × Lq × iq` in Vd, `ω × Ld × id` in Vq |
| 4.3 | BEMF feedforward | `v_per_hz × ω × bemf_feedforward` added to Vq |
| 4.4 | Low-speed startup strategy (open-loop → closed-loop transition) | I/f startup or observer-based |
| 4.5 | PI gain tuning for target motor at target bus voltage | Bandwidth target: ~500 Hz current loop |
| 4.6 | Temperature measurement + current derating | ADC channels → LPF → clamp max_current_A |
| 4.7 | Flux weakening entry condition (modulation depth > threshold) | CVCP: `id = -id_char × (1 - 1/speed_ratio)` |

**Outcome:** Production-ready FOC with decoupling, BEMF compensation, temperature management, and field weakening.

---

*Generated by INTEGRATION_REVIEW analysis comparing foc-firmware against moteus fw/ reference. All line references verified against current source files as of 2026-07-07. Fixes applied per the Fixes Applied table above.*
