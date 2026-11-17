# Learnings — foc-firmware-docs

## 2026-07-08 — Síntesis Comparativa Phoenix vs moteus-c1

### Hallazgos Clave

**Documento creado:** `foc-firmware/docs/SINTESIS_COMPARATIVA.md`

### Problema Crítico: TIM2 ARR=0xFFFFFFFF
- El PWM no conmuta realmente (frecuencia ~0.02 Hz en center-aligned mode)
- Los duty cycles se actualizan a 20 kHz desde TIM6 ISR, pero el timer tarda ~50 s/ciclo
- El valor correcto debería ser ARR=4250 para 20 kHz o 2833 para 30 kHz
- Esto es un error de configuración en CubeMX/IOC, no intencional
- Referencia: moteus-c1 calcula ARR = PCLK1×2 / (2×pwm_rate) = 170 MHz / 60,000 = 2833

### Corrientes de Gate Drive (DRV8323)
- Phoenix configura IDRIVEP_HS=370 mA e IDRIVEN_HS=740 mA
- moteus-c1 (family 2) usa 60 mA / 20 mA respectivamente
- Las proporciones son ~6× (HS source) y 37× (HS sink)
- Esto puede indicar FETs con mayor Q_g, o configuración por defecto no ajustada
- El registro real del DRV8323 para 370 mA codifica como `010` en bits 3:1 de CTRL1 (mapping de Phoenix)
  - La tabla de mapeo de Phoenix (drv8323.c:18) tiene valores DRV8323S
  - Index 2 en tabla Phoenix = 370 mA (datasheet DRV8323S index 2 = 260 mA)

### Conflicto PA9
- PA9 no puede ser TIM1_CH2 (encoder) y ADC2_CH5 (VBUS) simultáneamente
- Phoenix eligió encoder; VBUS hardcodeado a 24.0V
- Solución arquitectónica: VBUS por SPI2 desde master
- Mientras tanto, el lazo de corriente no compensa variaciones de batería

### Ic en ADC1 en vez de ADC3
- Mismo pin PB1, pero muxed a ADC1_IN12 (Phoenix) vs ADC3_IN12 (moteus)
- ADC1 comparte Ia e Ic en modo scan → skew de ~353 ns
- Reconfigurar a ADC3_IN12 para muestreo simultáneo como opción de mejora

### VDS_LVL = 260 mV vs 940 mV
- Phoenix tiene umbral de OCP 3.6× más sensible que moteus-c1
- Riesgo de falsos positivos, pero también más protección

### Encoder: AS5600 + Incremental vs AS5047 SPI
- Phoenix usa dos ICs (AS5600 I2C + TIM1 incremental) vs un AS5047 SPI
- Resolución 4× menor (4096 CPR vs 16384 CPR)
- Mayor complejidad de cableado y código

### Comunicación: SPI2 slave vs CAN-FD
- Cambio arquitectónico: punto a punto vs multi-drop
- Sin detección de errores ni arbitraje
- Sin transceptor CAN (menos BOM, menos robustez EMI)

### Archivos de Correspondencia Clave
- `Tools/foc.c` ↔ `fw/foc.cc` — Algoritmo FOC
- `Tools/servo_control.c` ↔ `fw/bldc_servo.cc` — State machine
- `Tools/drv8323.c` ↔ `fw/drv8323.cc` — Gate driver
- `Tools/encoder.c` ↔ `fw/aux_mbed.h` — Encoder incremental
- `Tools/comm_spi.c` ↔ `fw/fdcan.h` — Comunicación host

### Observaciones sobre la Toolchain
- Phoenix: CubeMX genera código, USER CODE sections son frágiles
- Sin suite de tests automatizados (moteus sí tiene con Bazel)
- HAL tiene overhead en ISR (HAL_TIM_PeriodElapsedCallback wrapper)
