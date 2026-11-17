---
slug: foc-firmware-integration-review
status: awaiting-approval
intent: clear
pending-action: write .omo/plans/foc-firmware-integration-review.md
approach: >
  Revisar configuración ADC (compatibilidad con DRV8323 CSA y FOC),
  revisar driver DRV8323, revisar integración encoder→FOC,
  documentar hallazgos en docs/INTEGRATION_REVIEW.md,
  y generar plan de correcciones priorizadas.
---

# Draft: foc-firmware-integration-review

## Components (topology ledger)

| id | outcome (one line) | status | evidence path |
|---|---|---|---|
| analysis-doc | Documento INTEGRATION_REVIEW.md con hallazgos ADC, DRV8323, encoder→FOC | active | fw/stm32g4_adc.cc, fw/drv8323.cc, fw/bldc_servo_control.h vs foc-firmware/* |
| adc-fix | Sampling time ADC aumentado, trigger HW configurado, VBUS sensing agregado | active | Core/Src/adc.c, foc.c |
| drv8323-fix | Calibración CSA implementada, HiZ agregado | active | Tools/drv8323.c, main.c |
| foc-isr | FOC movido a ISR TIM6/7 @ 20 kHz con CORDIC/LUT | active | Core/Src/*.c, Tools/foc.c, Core/Inc/*.h |

## Open assumptions (announced defaults)

| assumption | adopted default | rationale | reversible? |
|---|---|---|---|
| Sampling time ADC | 47.5 ciclos (~1.12 µs) | Compromiso entre precisión y ventana de tiempo | Yes (ajustable) |
| Trigger ADC | TIM2_TRGO (centro de PWM) | Más simple que LPTIM1, sincronización suficiente | Yes (se puede migrar a LPTIM1) |
| Frecuencia FOC | 20 kHz | Balance entre rendimiento y presupuesto de CPU | Yes (10-40 kHz) |
| sin/cos en ISR | Lookup table + interpolación (primera iteración) | Rápida de implementar, 0.5 µs | Yes (migrar a CORDIC después) |
| VBUS sensing | PA9/ADC2_IN5 con divisor resistivo | Coincide con pinout Phoenix | Yes |
| Documento de hallazgos | docs/INTEGRATION_REVIEW.md | Claramente diferenciado de PLAN_AVANCE.md | Yes |

## Findings (cited - path:lines)

### ADC — Critical (6 hallazgos)

**C1: Sampling time 2.5 cycles = ~59 ns vs DRV8323 CSA necesita ~1500 ns**
- `foc-firmware/Core/Src/adc.c:80,92,139` — `ADC_SAMPLETIME_2CYCLES_5`
- `fw/drv8323.h:58-67` — `CsaSettlingTime(20) = 1.5e-6` (1500 ns)
- Impacto: Lecturas de corriente inválidas, ruido en lazo FOC

**C2: ADC trigger por software, sin simultaneidad**
- `foc-firmware/Core/Src/adc.c:58` — `ADC_SOFTWARE_START`
- `fw/stm32g4_adc.cc:137-148` — LPTIM1 trigger, sincronizado
- Impacto: Error de fase entre Ia, Ib, Ic en transforms

**C3: VBUS hardcodeado a 24.0V**
- `foc-firmware/Tools/foc.c:215` — `float vbus = 24.0f`
- Phoenix PCB: PA9 = ADC2_IN5 (señal VBUS)
- Impacto: SVM incorrecto, sin protección UVLO/OVLO

**C4: FOC en main loop a 1 kHz**
- `foc-firmware/Core/Src/main.c:202-218` — bucle principal con HAL_GetTick
- `fw/bldc_servo_control.h` — control a 15-30 kHz
- Impacto: Ancho de banda insuficiente, respuesta lenta

**C5: sinf/cosf sin aceleración (50-100 µs)**
- `foc-firmware/Tools/foc.c:259-260` — `sinf(theta_elec)` / `cosf(theta_elec)`
- `fw/foc.h:34-76` — CORDIC HW (5 ciclos)
- Impacto: A 20 kHz (50 µs ventana), consume todo el CPU

**C6: Sin calibración CSA del DRV8323**
- `foc-firmware/Tools/drv8323.c` — no hay StartCalibrate/StopCalibrate
- `fw/drv8323.cc:237-268` — CAL bits en reg6
- Impacto: Offset DC en medición de corriente

### DRV8323 — Verificación

- Config de registros correcta (CSA gain 20, VREF/2, PWM 3X, dead time 50ns)
- Lookup tables correctas (copiadas de moteus)
- SPI protocol match (16-bit, addr<<11, bit 15=R/W)
- ⚠️ Pin HiZ no usado (moteus sí lo usa para PowerOn/Off)
- ⚠️ Enable sequence directa vs state machine (moteus)

### Encoder → FOC

- TIM1 encoder mode TI1: correcto
- Manejo overflow 16-bit: correcto (int16_t delta)
- Ángulo eléctrico: correcto
- AS5600 para seed absoluto: correcto
- ⚠️ TIM1 solo TI1 (1x) vs TI12 (4x) = menor resolución

## Decisions (with rationale)

| Decision | Choice | Rationale |
|---|---|---|
| Sampling time | 47.5 cycles first, evaluar 640.5 | Balance entre precisión y ventana ADC |
| Trigger ADC | TIM2_TRGO (no LPTIM1) | Menos cambios, TIM2 ya está configurado |
| VBUS en ADC2 | Canal 5 (PA9) como segundo rank en ADC2 | Pinout Phoenix lo define así |
| FOC freq | 20 kHz center-aligned | TIM2 ya configurado así |
| sin/cos | LUT + interpolación (v1), CORDIC (v2) | Rápido de implementar, bajo riesgo |
| CAL CSA | En WriteConfig después de enable | Sigue el patrón de moteus |
| HiZ | PC15 como output, LOW=High-Z | Pin libre, mejora seguridad |

## Scope IN

- Documento completo en `docs/INTEGRATION_REVIEW.md` con hallazgos, comparativas, roadmap
- Corrección de sampling time ADC (C1)
- Configuración de trigger hardware para ADC desde TIM2_TRGO (C2)
- Agregar VBUS sensing PA9/ADC2_IN5 (C3)
- Calibración CSA del DRV8323 (C6)
- Calibración de offset ADC en startup
- Agregar pin HiZ para power-on seguro
- Mover FOC a ISR de timer básico @ 20 kHz (C4)
- Implementar lookup table para sin/cos (C5)

## Scope OUT (Must NOT have)

- NO CORDIC en v1 (se hará en v2)
- NO LPTIM1 trigger (se evaluará después)
- NO DMA para ADC (innecesario para 20 kHz con trigger HW)
- NO field weakening ni MTPA (optimizaciones futuras)
- NO refactor completo del plan de avance existente
- NO cambios en SPI2, I2C2, o comunicación con master
- NO cambios en AS5600 driver

## Open questions

Ninguna — todos los hallazgos se resolvieron explorando el código.

## Approval gate
status: awaiting-approval
