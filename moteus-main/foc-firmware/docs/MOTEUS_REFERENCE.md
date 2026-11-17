# MOTEUS Reference — Peripheral Configuration Comparison

> **Propósito:** Documentar cómo los periféricos clave se configuran en el firmware **moteus original** (mjbots/moteus) vs nuestra implementación **Phoenix**.
>
> **Fuente moteus:** `fw/bldc_servo.cc`, `fw/drv8323.cc`, `fw/bldc_servo_structs.h` (commit `main`, Jul 2026)
>
> **Fuente Phoenix:** `Core/Src/tim.c`, `Tools/drv8323.c`, `Tools/drv8323.h`, `cubemx/FOCTest.ioc`
>
> **Board moteus de referencia:** moteus-c1 (family 2, hw rev ≥ 3)
>
> **Board Phoenix:** Basada en moteus-c1 (family 2), STM32G431CBU6

---

## 1. PWM Timer (TIM2)

### Configuración General

| Parámetro | Moteus-c1 | Phoenix (actual) | Phoenix (objetivo) |
|---|---|---|---|
| **Timer** | TIM2 | TIM2 | TIM2 |
| **CounterMode** | CENTERALIGNED2 | CENTERALIGNED2 ✅ | CENTERALIGNED2 |
| **AutoReloadPreload** | ENABLED (ARPE=1) | ENABLED ✅ | ENABLED |
| **Prescaler (PSC)** | 0 | 0 ✅ | 0 |
| **ARR** | ~2833 | 0xFFFFFFFF ⚠️ | **4250** (corregido) |
| **PWM rate** | 30 kHz | ~0.02 Hz | **30 kHz** |
| **OC Fast Mode** | Default mbed | ENABLED | ENABLED |

### Cálculo del ARR

**Moteus** (`bldc_servo.cc:510`):
```cpp
pwm_counts_ = HAL_RCC_GetPCLK1Freq() * 2 / (2 * rate_config_.pwm_rate_hz);
timer_->ARR = pwm_counts_;
```

**Default pwm_rate_hz** (`bldc_servo_structs.h:497-502`):
```cpp
uint16_t pwm_rate_hz =
    (g_measured_hw_family == 0 && g_measured_hw_rev <= 2) ? 60000 :
    g_measured_hw_family == 3 ? 15000 :
    30000;  // ← c1 (family 2) → 30 kHz
```

**Para moteus-c1** (family 2, APB1=85 MHz):
```
TIM2CLK = PCLK1 × 2 = 170 MHz   (APB1 ÷ 2)
ARR = PCLK1 × 2 / (2 × pwm_rate)
   = 170 MHz / (2 × 30,000)
   = 2833  (truncado)

f_pwm = TIM2CLK / (2 × ARR) = 170 MHz / 5,666 ≈ 30 kHz ✓
```

**Para Phoenix** (APB1=170 MHz, TIM2CLK=170 MHz):
```
TIM2CLK = PCLK1 = 170 MHz   (APB1 ÷ 1, sin prescaler)
ARR = PCLK1 / (2 × pwm_rate)     ← misma fórmula, TIM2CLK directo
   = 170 MHz / (2 × 30,000)
   = 2833  (mismo valor numérico)

f_pwm = TIM2CLK / (2 × ARR) = 170 MHz / 5,666 ≈ 30 kHz ✓
```

> **Nota:** El valor numérico de ARR coincide porque aunque la topología de reloj difiere (APB1 sin divider en Phoenix vs ÷2 en c1), la frecuencia efectiva del timer es la misma: 170 MHz.

### Control Register (CR1)

**Moteus** (`bldc_servo.cc:490-498`):
```cpp
timer_->CR1 =
    (2 << TIM_CR1_CMS_Pos) |   // BIT[6:5] = 10 → Center-aligned mode 2
    TIM_CR1_ARPE;               // BIT[7]   = 1  → Auto-reload preload enabled
```

### Trigger de ADC

| Aspecto | Moteus-c1 | Phoenix |
|---|---|---|
| **Mecanismo** | DMA (CC4 → LPTIM1 → ADCs) | TIM2 TRGO → ADC1/2 |
| **Trigger source** | TIM2_CC4 | TIM2_Update |
| **Frecuencia trigger** | 1× por ciclo PWM (al bajar) | 2× por ciclo PWM (sube y baja) |
| **DMA** | Habilitado | No usado |

**Moteus** configura:
- `timer_->DIER |= TIM_DIER_CC4DE;` — DMA request en CC4
- `timer_->CCR4 = pwm_counts_;` — CC4 = ARR, trigger al peak del PWM (down-count)
- En center-aligned mode 2, CC4 flag se setea **solo en down-count** → exactamente 1 trigger por ciclo PWM
- El DMA canal mueve datos a LPTIM1 que a su vez triggera los ADCs

**Phoenix** usa TIM2_TRGO = Update. En center-aligned mode, el update event ocurre **dos veces por ciclo PWM** (al inicio del up-count y al inicio del down-count). Esto significa que los ADCs se triggeran al doble de la frecuencia PWM. Es funcional pero no óptimo.

### Interrupción

| Aspecto | Moteus-c1 | Phoenix |
|---|---|---|
| **Update IRQ** | TIM2_UIE habilitado | No usado |
| **FOC timing** | TIM2 update IRQ | TIM6 (timer separado a 20 kHz) |
| **IRQ priority** | Default | Preempt 0 (TIM6) |

Moteus usa el propio TIM2 como fuente de interrupción para el lazo FOC. Phoenix usa un timer básico separado (TIM6) para el timing, lo que desacopla la frecuencia del FOC de la frecuencia PWM.

### Cambio en Línea de la Frecuencia PWM

Moteus puede reconfigurar la frecuencia PWM en caliente (`ConfigurePwmTimer()` se llama cuando cambia `config_.pwm_rate_hz`). Frecuencias típicas:

| Board | pwm_rate_hz default | ARR (TIM2CLK=170 MHz) |
|---|---|---|
| moteus r4.x (early) | 60000 | 1416 |
| moteus-c1 | 30000 | 2833 |
| moteus-x1 | 15000 | 5667 |

---

## 2. Gate Driver DRV8323

### Interfaz SPI

| Aspecto | Moteus-c1 | Phoenix |
|---|---|---|
| **SPI implementation** | Bit-banged (`Stm32BitbangSpi`) | Hardware SPI3 |
| **Clock rate** | ~1 MHz | 5.3125 MBit/s |
| **Frame size** | 16-bit | 16-bit |
| **CPOL/CPHA** | mode 0 (CPOL=LOW, CPHA=1EDGE) | CPHA=2EDGE (mode 1) ⚠️ |
| **CS control** | GPIO (bit-banged) | GPIO (PC13, software) |
| **Pins** | SCK, MOSI, MISO + CS | SCK=PC10, MISO=PC11, MOSI=PB5, CS=PC13 |

**⚠️ Diferencia de CPHA:** Moteus usa CPHA=1EDGE (modo SPI 0, captura en primer flanco), mientras Phoenix usa CPHA=2EDGE (modo SPI 1, captura en segundo flanco). El DRV8323S soporta ambos modos (ver datasheet §7.5.4: "Data is clocked in on the rising edge of SCLK and data is clocked out on the falling edge of SCLK" en SPI mode 1.1), ambas configuraciones son válidas.

### Registros de Configuración

Los registros del DRV8323S se escriben con frames de 16 bits: `[bit15:11=addr] [bit10=R/W] [bit9:0=data]`.

#### Register Map

| Addr | Register | Moteus-c1 | Phoenix | Diferencia |
|------|----------|-----------|---------|------------|
| 0x00 | **FAULT1** | (solo lectura) | (solo lectura) | — |
| 0x01 | **FAULT2** | (solo lectura) | (solo lectura) | — |
| 0x02 | **DRV_CTRL1** | Ver desglose abajo | Ver desglose abajo | ⚠️ |
| 0x03 | **DRV_CTRL2** | Ver desglose abajo | Ver desglose abajo | ⚠️ |
| 0x04 | **CSA_CTRL** | Ver desglose abajo | Ver desglose abajo | ⚠️ |

#### DRV_CTRL1 (0x02) — Gate Drive

| Bitfield | Moteus-c1 | Phoenix | Notas |
|----------|-----------|---------|-------|
| GATE_RESET (15) | 0 | 0 | = |
| DIS_CPUV (14) | 0 (falso) | 0 | = |
| DIS_GDF (13) | 0 (falso) | 0 | = |
| OTW_REP (12) | 0 (falso) | 0 | = |
| PWM_MODE (11:10) | 01 (3X) | 01 (3X) | = |
| Reserved (9) | 0 | 0 | = |
| PWM_1X_ASYNC (8) | 0 | 0 | = |
| PWM_1X_DIR (7) | 0 | 0 | = |
| Reserved (6) | 0 | 0 | = |
| CBC (5) | 1 (true) | 1 | = |
| TDRIVE (4) | 0 (1000ns) | 0 (1000ns) | = |
| IDRIVEP_HS (3:1) | `000` (60 mA) | `010` (370 mA) ⚠️ | Diferente |
| IDRIVEN_HS (0) | `0` (20 mA) | `1` (740 mA) ⚠️ | Diferente |

Valores de **IDRIVEP_HS** (codificación 3-bit):

| Code | mA |
|------|-----|
| 000 | 60 |
| 001 | 130 |
| 010 | 260 |
| 011 | 370 (Phoenix high-side) |
| 100 | 510 |
| 101 | 600 |
| 110 | 680 |
| 111 | 740 |

Valores de **IDRIVEN_HS** (codificación 1-bit):

| Code | mA |
|------|-----|
| 0 | 20 (moteus) |
| 1 | 740 (Phoenix) |

#### DRV_CTRL2 (0x03) — OCP, Dead Time, VDS

| Bitfield | Moteus-c1 | Phoenix | Notas |
|----------|-----------|---------|-------|
| Reserved (15:13) | 000 | 000 | = |
| TDRIVE_LS (12) | 0 (1000ns) | 0 (1000ns) | = |
| IDRIVEP_LS (11:9) | `000` (60 mA) | `010` (370 mA) ⚠️ | Diferente |
| IDRIVEN_LS (8) | `0` (20 mA) | `1` (740 mA) ⚠️ | Diferente |
| DEAD_TIME (7:6) | `01` (50 ns) | `01` (50 ns) | = |
| OCP_MODE (5:4) | `00` (latch fault) | `00` (latch fault) | = |
| OCP_DEG (3:2) | `10` (4 µs) | `10` (4 µs) | = |
| VDS_LVL (1:0) | `11` (940 mV) | `00` (260 mV) ⚠️ | Diferente |

Valores de **VDS_LVL**:

| Code | mV |
|------|-----|
| 00 | 260 (Phoenix → más sensible) |
| 01 | 520 |
| 10 | 720 |
| 11 | 940 (moteus → menos sensible) |

Valores de **DEAD_TIME** codificación:

| Code | ns |
|------|-----|
| 00 | 25 |
| 01 | 50 |
| 10 | 100 |
| 11 | 200 |

Valores de **OCP_DEG**:

| Code | µs |
|------|-----|
| 00 | 1 |
| 01 | 2 |
| 10 | 4 |
| 11 | 8 |

#### CSA_CTRL (0x04) — Current Sense Amplifier

| Bitfield | Moteus-c1 | Phoenix | Notas |
|----------|-----------|---------|-------|
| Reserved (15:13) | 000 | 000 | = |
| CSA_FET (12) | 0 (shunt) | 0 (shunt) | = |
| VREF_DIV (11) | 1 (div by 2) | 1 (div by 2) | = |
| LS_REF (10) | 0 | 0 | = |
| Reserved (9:7) | 000 | 000 | = |
| CSA_GAIN (6:5) | `01` (20 V/V) | `01` (20 V/V) | = |
| Reserved (4:2) | 000 | 000 | = |
| DIS_SEN (1) | 0 (enabled) | 0 (enabled) | = |
| SEN_LVL (0) | `0` (500 mV) | `0` (500 mV) | = |

#### OCP_CTRL (0x05) — Overcurrent Control

| Bitfield | Moteus-c1 | Phoenix | Notas |
|----------|-----------|---------|-------|
| TRETRY (14) | 0 (4ms) | 0 (4ms) | = |
| DEAD_TIME (7:6) | `01` (50 ns) | `01` (50 ns) | = (duplicado, solo en DRV8323S) |

### Resumen de Diferencias

| Parámetro | Moteus-c1 | Phoenix | Implicancia |
|-----------|-----------|---------|-------------|
| **SPI clock** | ~1 MHz (bitbang) | 5.3 MHz (HW) | Phoenix más rápido, menos overhead CPU |
| **SPI CPHA** | 1EDGE | 2EDGE | Ambos válidos para DRV8323S |
| **IDRIVEP_HS** | 60 mA | 370 mA | Phoenix maneja FETs más grandes |
| **IDRIVEN_HS** | 20 mA | 740 mA | Phoenix → switching más agresivo |
| **IDRIVEP_LS** | 60 mA | 370 mA | (simétrico con HS) |
| **IDRIVEN_LS** | 20 mA | 740 mA | (simétrico con HS) |
| **VDS_LVL** | 940 mV | 260 mV | Phoenix protege más temprano (menos ruido? o más sensibilidad?) |
| **CSA gain** | 20 V/V | 20 V/V | = |
| **Dead time** | 50 ns | 50 ns | = |
| **OCP mode** | Latched fault | Latched fault | = |
| **OCP deglitch** | 4 µs | 4 µs | = |
| **CBC** | true | true | = |
| **PWM mode** | 3X | 3X | = |

### CSA Calibration

Ambos implementan calibración de offset del Current Sense Amplifier.

**Moteus** (`fw/drv8323.cc`):
- Secuencia CAL_A → CAL_B → CAL_C con delays basados en `config_.csa_calib_wait` (~1 ms cada uno)
- Maneja estados de calibración con timeout
- Integrado en la máquina de estados del DRV8323

**Phoenix** (`Tools/drv8323.c`):
```c
void DRV8323_CalibrateCSA(DRV8323_HandleTypeDef *h) {
    DRV8323_Enable(h);
    DWT_Delay_us(10);
    // CAL_A: set bit 13 (CAL1) en OCP_CTRL
    reg = DRV8323_ReadReg(h, DRV8323_REG_OCP_CTRL);
    reg |= (1 << 13);
    DRV8323_WriteReg(h, DRV8323_REG_OCP_CTRL, reg);
    DWT_Delay_us(10);
    // CAL_B: bit 12 (CAL2)
    reg |= (1 << 12);
    DRV8323_WriteReg(h, DRV8323_REG_OCP_CTRL, reg);
    DWT_Delay_us(10);
    // CAL_C: clear bits 13 y 12 (complete calibration)
    reg &= ~((1 << 13) | (1 << 12));
    DRV8323_WriteReg(h, DRV8323_REG_OCP_CTRL, reg);
}
```
- Usa DWT cycle counter para delays precisos (~10 µs por paso)
- Secuencia similar: Enable → CAL_A → CAL_B → CAL_C (completar)
- Sin timeout ni manejo de estados

---

## 3. Observaciones y Recomendaciones

### PWM Timer
1. **Corregir ARR de TIM2** de 0xFFFFFFFF a **4250** para obtener PWM real a 20 kHz ✅
2. Evaluar si se desea el trigger de ADC por CC4+DMA (como moteus) o mantener TRGO_UPDATE
   - TRGO_UPDATE triggera 2× por ciclo PWM (center-aligned) — es funcional pero subóptimo
   - CC4+DMA da 1 trigger preciso por ciclo — más exacto pero requiere configuración adicional
3. La interrupción de update de TIM2 **no es necesaria** en Phoenix porque el FOC loop usa TIM6

### DRV8323
1. Las corrientes de gate drive **(370/740 mA vs 60/20 mA)** sugieren que Phoenix usa FETs de mayor capacidad de gate charge. Verificar que los FETs soporten esta corriente de gate.
2. **VDS_LVL = 260 mV** es mucho más sensible que moteus (940 mV). Verificar si causa falsos positivos de OCP durante transitorios normales de arranque. Si hay disparos espurios, incrementar a 520 mV o 940 mV.
3. El CPHA=2EDGE en Phoenix vs 1EDGE en moteus no debería causar problemas — el DRV8323S samplea en rising edge independientemente de la polaridad del clock.

---

## Fuentes

- Moteus: `fw/bldc_servo.cc` — `ConfigurePwmTimer()` (línea 458)
- Moteus: `fw/bldc_servo_structs.h` — `BldcServoConfig::pwm_rate_hz` (línea 498)
- Moteus: `fw/drv8323.cc` — `Config::MakeDefaultConfig()` + `Spi::Spi()`
- Phoenix: `Core/Src/tim.c` — `MX_TIM2_Init()` (línea 84)
- Phoenix: `Tools/drv8323.c` — `DRV8323_Init()` + `DRV8323_WriteConfig()`
- Phoenix: `cubemx/FOCTest.ioc` — SPI3 y TIM2 configuration
- Datasheet: DRV8323S datasheet (SLVSF44) — register map §7.5
