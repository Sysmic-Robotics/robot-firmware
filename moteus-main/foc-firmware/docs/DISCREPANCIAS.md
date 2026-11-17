# Corrección de Discrepancias — Documentación Phoenix

> **Propósito:** Este documento identifica discrepancias entre la documentación y el código fuente del proyecto Phoenix.
> **Fuente de verdad:** Código fuente (`Core/Src/`, `Core/Inc/`, `Tools/`, `cubemx/FOCTest.ioc`)
> **Fecha de última revisión:** 2026-07-07

---

## Resumen de Estado

| Archivo | Discrepancias críticas | Discrepancias menores |
|---------|----------------------|----------------------|
| `HARDWARE.md` | 0 | 0 — ✅ Actualizado al código |
| `FIRMWARE_STATUS.md` | 0 | 0 — ✅ Actualizado al código |
| `INTEGRATION_REVIEW.md` | **1** (C3 VBUS fix status) | 1 (PB13 EXTI) |
| `PLAN_AVANCE.md` | 0 | 0 — ✅ Actualizado |
| `REVISIÓN DE INTEGRACIÓN CORREGIDA — Proyecto Phoenix.md` | 0 | 0 — ✅ Consistente |

---

## Discrepancia DC-1 (Resuelta) — Pines I2C2

**Estado: ✅ RESUELTA**

`FIRMWARE_STATUS.md` anteriormente listaba I2C2 como PB10/PB11. **Corregido**: ahora muestra PC4 (SCL) y PF0 (SDA) — coincidiendo con el código y el IOC.

---

## Discrepancia DC-2 (Resuelta) — Conflicto PA9 / VBUS

**Estado: ✅ RESUELTA**

El código anterior tenía `read_vbus()` en PA9 como ADC2_CH5, lo que causaba conflicto con TIM1_CH2 (encoder). **Solución implementada**: VBUS está hardcodeado como 24.0V en `main.c`. No hay más intento de leer VBUS por ADC. La documentación refleja este cambio.

---

## Discrepancia DC-3 (Pendiente) — C3 en INTEGRATION_REVIEW.md

**Estado: ⚠️ PENDIENTE**

`INTEGRATION_REVIEW.md` marca C3 como "FIXED" refiriéndose a la implementación de `read_vbus()`. Como VBUS ahora es hardcodeado, C3 debe actualizarse. Pendiente de revisión de INTEGRATION_REVIEW.md completo.

---

## Discrepancia DC-4 (Resuelta) — PC15 HiZ

**Estado: ✅ RESUELTA**

`HARDWARE.md` ahora documenta correctamente PC15 = initial HIGH (HiZ activo al arranque), coincidiendo con el código en `gpio.c`. El DRV8323 inicia en HiZ seguro hasta que el firmware completa la inicialización y desactiva HiZ.

---

## Discrepancia DC-5 (Resuelta) — PB13 EXTI

**Estado: ✅ RESUELTA (sin corrección necesaria, documentado para claridad)**

`HARDWARE.md` ahora documenta PB13 como EXTI falling-edge con pull-up interno, tal como está configurado en `gpio.c`.

---

## Discrepancia DC-6 (Resuelta) — Pines GPIO no documentados

**Estado: ✅ RESUELTA**

`FIRMWARE_STATUS.md` ahora incluye PC6 (nSLEEP), PB0, PB15 en la tabla de periféricos. `HARDWARE.md` documenta todas las funciones GPIO.

---

## Discrepancia DC-7 (Resuelta) — read_vbus() bloqueante en ISR

**Estado: ✅ RESUELTA**

La función `read_vbus()` fue eliminada. VBUS es ahora una constante (24.0V). No hay polling bloqueante en la ISR de TIM6.

---

## Discrepancia DC-8 (Reinterpretada) — Período de TIM2

**Estado: ✅ DOCUMENTADO COMO ISSUE**

La discrepancia original decía que `HARDWARE.md` debía cambiar TIM2.Period de 0xFFFFFFFF a 4250. Sin embargo, **el código real (`tim.c:96`) sí tiene Period = 4294967295 (0xFFFFFFFF)**. La documentación anterior reflejaba correctamente el código.

Tras revisar el código completo:
- TIM2 ARR = 0xFFFFFFFF (configuración actual en `tim.c` y `.ioc`)
- Center-aligned mode (TIM_COUNTERMODE_CENTERALIGNED2)
- PWM duty cycles actualizados a 20 kHz desde TIM6 ISR

**Esta configuración produce una frecuencia PWM física de ~0.02 Hz**, lo cual es extremadamente baja para control motor. Se documenta en HARDWARE.md y CONCLUSIONES.md como **problema conocido** que requiere corrección (ARR debe ser ~4250 para 20 kHz PWM).

---

## Estado de Documentación Cruzada

| Documento vs Realidad | Estado | Notas |
|---|---|---|
| `HARDWARE.md` ↔ código | ✅ Coinciden | Actualizado Jul 2026 |
| `FIRMWARE_STATUS.md` ↔ código | ✅ Coinciden | Actualizado Jul 2026 |
| `CONCLUSIONES.md` ↔ código | ✅ Coinciden | Creado Jul 2026 |
| `INTEGRATION_REVIEW.md` ↔ código | ⚠️ Parcial | Requiere revisión de C3 (VBUS) y M3 (PC15) |
| `PLAN_AVANCE.md` ↔ código | ✅ Coinciden | Actualizado |
| `REVISIÓN DE INTEGRACIÓN CORREGIDA — Proyecto Phoenix.md` ↔ código | ✅ Coinciden | Consistente |

---

## Referencias

- `HARDWARE.md` — Documentación de hardware actualizada
- `FIRMWARE_STATUS.md` — Estado del firmware actualizado
- `CONCLUSIONES.md` — Resumen del proyecto
- `cubemx/FOCTest.ioc` — Archivo de configuración STM32CubeMX
- `Core/Src/main.c` — Aplicación principal (VBUS hardcodeado)
- `Core/Src/tim.c` — TIM2 ARR = 0xFFFFFFFF (problema conocido)
- `Core/Src/gpio.c` — PC15 inicial HIGH, PB13 EXTI falling-edge
