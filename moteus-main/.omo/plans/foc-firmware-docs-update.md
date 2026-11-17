# foc-firmware-docs-update — Work Plan

## TL;DR (For humans)

**What:** Actualizar 3 documentos de documentación que describen el estado PRE-fix del firmware, después de que todos los hallazgos críticos (C1-C6) y M3 fueran corregidos.

**Files to update:**
1. `foc-firmware/docs/INTEGRATION_REVIEW.md` — Marcar C1-C6 y M3 como FIXED, actualizar tablas, roadmap
2. `foc-firmware/docs/FIRMWARE_STATUS.md` — FOC loop: ❌→✅, agregar CSA cal, HiZ, cambios ADC
3. `foc-firmware/docs/PLAN_AVANCE.md` — Estado "falta FOC" actualizado, Fase 2 items completos marcados

## Todos

- [x] 1. [DOCS] Update INTEGRATION_REVIEW.md
  What to do:
  1. Insert "Fixes Applied (2026-07-07)" section after the header with a summary table of all 7 fixes (C1-C6, M3)
  2. Mark each C1-C6 finding header with `**[FIXED]**` and add a "Fix applied" subsection showing what changed
  3. Mark M3 with `**[FIXED]**` similarly
  4. Update Section 1.1 configuration table — sampling time now 47.5 cycles, trigger TIM2_TRGO, VBUS on ADC2_CH5
  5. Update Section 3 control flow diagram — show 20 kHz ISR path instead of 1 kHz main loop
  6. Update Section 5 roadmap — mark Fase 1 tasks 1.3, 1.4 as done, Fase 2 tasks 2.1-2.3 as done, Fase 3 tasks 3.1-3.2 as done
  References: Tools/foc.h, Tools/foc.c, Core/Src/adc.c, Core/Src/tim.c, Core/Src/main.c, Tools/trig_lut.h, Tools/drv8323.h, Tools/drv8323.c (current state)
  QA: grep for "2.5 cycles", "SOFTWARE_START", "24.0f", "sinf", "not performed" — none should remain in "Current state" descriptions (only in "Before/After" comparison)

- [x] 2. [DOCS] Update FIRMWARE_STATUS.md
  What to do:
  1. Section 2 — Drivers: Update DRV8323 row: add "CSA calibration, HiZ control (PC15)"
  2. Section 3 — Component table: FOC loop: ❌ Pendiente → ✅ ISR @20kHz with LUT sin/cos (Tools/trig_lut.h)
  3. Section 3 — ADC row: add detail "Muestreo @47.5 ciclos, trigger HW TIM2_TRGO, VBUS en PA9"
  4. Section 4 — Add HiZ pin (PC15) row
  5. Section 5 (Roadmap) — Fase 2 "FOC Control Loop": mark as ✅ COMPLETED, add note
  6. Update date to 2026-07-07
  QA: Verify no "❌ Pendiente" for FOC loop after edit

- [x] 3. [DOCS] Update PLAN_AVANCE.md
  What to do:
  1. Update header line 4: "Estado firmware: FOC loop @20kHz funcional, falta comunicación + control alto nivel"
  2. Add "Log de Cambios — 2026-07-07" section after the header: list completed FOC items
  3. Fase 2 "FOC Control Loop": add "✅ COMPLETED" badge after the title
  4. Mark tasks 2.1 (TIM6 ISR), 2.2 (ADC sync), 2.3-2.7 (FOC math — already existed), 2.9 (BEMF — not done) with status
  5. Update table at bottom: P2 "4-6 días" → "✅ COMPLETED"
  QA: grep for "falta FOC" — should be removed

## Verification
- All 3 files read correctly after edits
- No stale mentions of "24.0f", "1 kHz", "software trigger", "no CSA", "sinf" as current state
- Todos actualizados en todowrite
