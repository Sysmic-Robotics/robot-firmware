# foc-firmware-drivers - Work Plan

## TL;DR (For humans)

**What you'll get:** Driver funcional para el encoder magnético AS5600 (vía I2C2), helpers para leer el encoder incremental (TIM1), y driver completo para el gate driver DRV8323S (vía SPI3). Todo integrado en `Tools/` con un ejemplo de inicialización en main.c.

**Why this approach:** El AS5600 se comunica por I2C2 (ya configurado), el DRV8323S por SPI3 (ya configurado como Master). TIM1 ya está en modo encoder desde CubeMX. Solo necesitamos escribir los drivers y conectarlos en el código de usuario.

**What it will NOT do:** No modifica la configuración de periféricos existente (TIM1, SPI3, I2C2 se quedan como están). No toca cubemx/FOCTest.ioc. No reemplaza los drivers originales del proyecto moteus (son referencia C++ para otro entorno).

**Effort:** Short
**Risk:** Low - solo código de usuario en Tools/ y secciones USER CODE de main.c
**Decisions to sanity-check:** Los defines de configuración del DRV8323 (CSA gain, PWM mode, dead time)

Your next move: **Aprueba este plan** para que los workers ejecuten la implementación.

---

> TL;DR (machine): Short | Low | 3 drivers (AS5600, TIM1 encoder, DRV8323S) + integración en main.c

## Scope
### Must have
- Driver AS5600 funcional para STM32G4 con I2C bloqueante
- Helper(s) para leer posición del encoder incremental (TIM1)
- Driver DRV8323S con acceso SPI a registros de configuración + lectura de fault status
- Ejemplo de inicialización y uso en USER CODE de main.c

### Must NOT have
- NO modificar configuración de periféricos CubeMX (tim.c, spi.c, i2c.c, gpio.c, adc.c)
- NO modificar .ioc
- NO tocar código del proyecto moteus (fw/)
- NO modificar HARDWARE.md (documentación ya está)
- NO usar interrupciones I2C (solo polling)
- NO asumir conexión física de DIR en AS5600

## Verification strategy
- Test decision: none (embedded, requiere HW real)
- Evidence: .omo/evidence/task-<N>-foc-firmware-drivers.md

## Execution strategy
### Parallel execution waves
Wave 1: AS5600 fix + TIM1 encoder helpers + DRV8323 driver (independientes, paralelos)
Wave 2: Integración en main.c (depende de Wave 1)

### Dependency matrix
| Todo | Depends on | Blocks | Can parallelize with |
| --- | --- | --- | --- |
| 1. AS5600 driver | (none) | 4 | 2, 3 |
| 2. TIM1 encoder helpers | (none) | (none) | 1, 3 |
| 3. DRV8323 driver | (none) | 4 | 1, 2 |
| 4. main.c integration | 1, 3 | (none) | (none) |

## Todos
> Implementation + Verification = ONE todo.
<!-- APPEND TASK BATCHES BELOW THIS LINE WITH edit/apply_patch - never rewrite the headers above. -->

### Wave 1 — AS5600 Driver Fix

- [x] 1. Fix AS5600 driver for STM32G4 (polling I2C)
  What to do / Must NOT do:
  Edit `Tools/as5600.h` and `Tools/as5600.c` to:
  1. In `as5600.h`:
     - Change `#include "stm32f0xx_hal.h"` → `#include "stm32g4xx_hal.h"`
     - Remove `#include "stm32f031x6.h"`
     - Add `#include <stdlib.h>` (needed for calloc)
     - Keep `AS5600_SLAVE_ADDRESS 0x36` as-is, but document in a comment that HAL expects `0x6C` (0x36 << 1)
  2. In `as5600.c`:
     - Replace ALL `HAL_I2C_Mem_Write_IT` → `HAL_I2C_Mem_Write` (8 occurrences)
     - Replace ALL `HAL_I2C_Mem_Read_IT` → `HAL_I2C_Mem_Read` (5 occurrences)
     - Fix BUG line 86: `a->confRegister[1] &= (1UL << 0);` → `a->confRegister[1] &= ~(1UL << 0);`
     - Add `#include <stdlib.h>` at top
  3. No other changes — DO NOT modify the API, struct, or register definitions

  Must NOT do:
  - NO cambiar la estructura AS5600_TypeDef ni las function prototypes
  - NO agregar nuevas funciones
  - NO modificar la lógica de configuración de registros
  - NO eliminar el header de licencia MIT

  Parallelization: Wave 1 | Blocked by: (none) | Blocks: 4
  References:
  - Tools/as5600.h (all lines 1-207)
  - Tools/as5600.c (all lines 1-604)
  - Core/Inc/i2c.h (for hi2c2 handle)
  Acceptance criteria:
  - `grep -c "stm32f0xx_hal.h\|stm32f031x6.h" Tools/as5600.h` -> must return 0
  - `grep -c "stm32g4xx_hal.h" Tools/as5600.h` -> must return 1
  - `grep -c "HAL_I2C_Mem_Write_IT\|HAL_I2C_Mem_Read_IT" Tools/as5600.c` -> must return 0
  - `grep -c "HAL_I2C_Mem_Write\|HAL_I2C_Mem_Read" Tools/as5600.c` -> must return > 0
  - `grep -c "&=" Tools/as5600.c | head -1` -> verify LPM2 line has `&= ~`
  QA scenarios:
  - Happy: `gcc -fsyntax-only -I Core/Inc -I Drivers/STM32G4xx_HAL_Driver/Inc Tools/as5600.c 2>&1` (if toolchain available, otherwise grep-based verification)
  - Failure: grep confirms no F0 includes remain, no _IT calls remain
  - Evidence: .omo/evidence/task-1-foc-firmware-drivers.md
  Commit: N (documentation/driver, no build system yet)

### Wave 1 — TIM1 Encoder Helpers

- [x] 2. Add TIM1 encoder usage helpers
  What to do / Must NOT do:
  Add minimal inline helper functions in `Core/Inc/tim.h` USER CODE section (or a separate `Tools/encoder.h`) that provide:
  1. `static inline uint16_t encoder_GetCount(void)` — returns `__HAL_TIM_GET_COUNTER(&htim1)`
  2. `static inline void encoder_Reset(void)` — sets `__HAL_TIM_SET_COUNTER(&htim1, 0)`
  3. Document encoder resolution: 0-65535 counts, 4x resolution vs CPR (counts per revolution = CPR × 4)

  Must NOT do:
  - NO modificar `MX_TIM1_Init()` ni ninguna función de CubeMX
  - NO cambiar configuración del timer (prescaler, period, encoder mode)
  - NO tocar tim.c
  - NO agregar interrupciones ni callbacks

  Parallelization: Wave 1 | Blocked by: (none) | Blocks: (none)
  References:
  - Core/Src/tim.c:44-63 (TIM1 Encoder init)
  - Core/Inc/tim.h:35-56 (existing extern and prototypes)
  - Core/Inc/main.h (for HAL includes)
  Acceptance criteria:
  - File `Tools/encoder.h` exists (or inline code in tim.h USER CODE)
  - Contains `encoder_GetCount` and `encoder_Reset`
  - Compiles cleanly when included
  QA scenarios:
  - `grep -c "encoder_GetCount\|encoder_Reset" Tools/encoder.h` -> must find both
  - No CubeMX AUTO sections modified
  - Evidence: .omo/evidence/task-2-foc-firmware-drivers.md
  Commit: N

### Wave 1 — DRV8323 Driver

- [x] 3. Create DRV8323S gate driver (SPI3)
  What to do / Must NOT do:
  Create `Tools/drv8323.h` and `Tools/drv8323.c` based on the reference in `fw/drv8323.{h,cc}` but ported to C with HAL_SPI.

  The DRV8323 SPI protocol:
  - 16-bit frames via SPI3
  - Register address is 3 bits (reg 0-7)
  - Read command: `0x8000 | (reg << 11)` — returns 11-bit value
  - Write command: `(reg << 11) | (value & 0x7ff)`
  - CS (PC13) controlled manually via HAL_GPIO_WritePin

  Include:
  1. **Types/config structs**:
     - `DRV8323_PwmMode` enum: 6x, 3x, 1x, Independent
     - `DRV8323_OcpMode` enum: LatchedFault, AutomaticRetry, ReportNoAction, Ignored
     - `DRV8323_Config` struct with all DRV8323 config registers
     - `DRV8323_Status` struct (fault register decoding)
  2. **Functions**:
     - `DRV8323_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *en_port, uint16_t en_pin, GPIO_TypeDef *flt_port, uint16_t flt_pin)` — init with pin mappings
     - `DRV8323_Enable(bool enable)` — toggle enable pin + SPI config write
     - `DRV8323_WriteReg(uint8_t reg, uint16_t value)` — single register write
     - `DRV8323_ReadReg(uint8_t reg)` — single register read
     - `DRV8323_WriteConfig(DRV8323_Config *config)` — write all 6 config registers + verify
     - `DRV8323_ReadStatus(DRV8323_Status *status)` — read fault regs 0+1
     - `DRV8323_IsFault(void)` — returns true if nFAULT is LOW
  3. **DRV8323 register map**:
     - Reg 0: Fault Status 1 (read-only)
     - Reg 1: Fault Status 2 (read-only)  
     - Reg 2: Drive Control
     - Reg 3: Gate Drive HS
     - Reg 4: Gate Drive LS
     - Reg 5: OCP Control
     - Reg 6: CSA Control
     - Reg 7: Reserved

  Default configuration values (safe for initial bringup):
  - PWM Mode: 3x (6 PWM input → 3 half-bridge)
  - CSA Gain: 20 V/V
  - VDS level: 700 mV
  - Dead time: 50 ns
  - OCP mode: Latched fault
  - IDRIVEP: 100 mA, IDRIVEN: 200 mA
  - VREF divider: enabled (bidirectional sensing)
  - SEN level: 500 mV

  Must NOT do:
  - NO modificar SPI3 configuration in spi.c
  - NO tocar otros periféricos
  - NO incluir dependencias del proyecto moteus (mjlib, mbed, PinNames)
  - NO usar bitbang SPI — usar HAL_SPI_TransmitReceive

  Parallelization: Wave 1 | Blocked by: (none) | Blocks: 4
  References:
  - fw/drv8323.h (all lines) — reference C++ API structure
  - fw/drv8323.cc:127-136 (SPI read/write protocol), 270-416 (WriteConfig with register tables)
  - Core/Src/spi.c:74-91 (SPI3 Master init)
  - Core/Inc/spi.h (SPI handle)
  - Core/Inc/gpio.h (for GPIO defines: PC13_CS, PC14_EN, PB13_FAULT)
  Acceptance criteria:
  - Files `Tools/drv8323.h` and `Tools/drv8323.c` exist
  - `grep -c "DRV8323_Init\|DRV8323_Enable\|DRV8323_WriteReg\|DRV8323_ReadReg\|DRV8323_WriteConfig\|DRV8323_ReadStatus\|DRV8323_IsFault" Tools/drv8323.h` -> must find all 7
  - No references to mbed, mjlib, PinNames, or C++ features
  QA scenarios:
  - Headers compile cleanly when included from C
  - Check no C++ syntax (class, namespace, template, etc.) remains in .h or .c
  - Evidence: .omo/evidence/task-3-foc-firmware-drivers.md
  Commit: N

### Wave 2 — Integration in main.c

- [x] 4. Add driver integration example in main.c USER CODE
  What to do / Must NOT do:
  Edit `Core/Src/main.c` in USER CODE sections only:
  1. **USER CODE BEGIN Includes**: Add `#include "Tools/as5600.h"` and `#include "Tools/drv8323.h"` and `#include "Tools/encoder.h"`
  2. **USER CODE BEGIN 2** (after MX_*_Init() calls): Add initialization code:
     ```c
     /* Initialize AS5600 */
     AS5600_TypeDef *as5600 = AS5600_New();
     if (as5600 != NULL) {
         as5600->i2cHandle = &hi2c2;
         as5600->i2cAddr = 0x6C;  // AS5600 address (0x36 << 1)
         as5600->PositiveRotationDirection = AS5600_DIR_CW;
         if (AS5600_Init(as5600) != HAL_OK) {
             Error_Handler();
         }
     }
     
     /* Initialize DRV8323 */
     DRV8323_Init(&hspi3, GPIOC, GPIO_PIN_13, GPIOC, GPIO_PIN_14, GPIOB, GPIO_PIN_13);
     DRV8323_Enable(true);
     
     /* Reset encoder position */
     encoder_Reset();
     ```
  3. **USER CODE BEGIN 3** (in main loop): Add example reading loop:
     ```c
     uint16_t as5600_angle = 0;
     uint16_t encoder_count = encoder_GetCount();
     if (AS5600_GetAngle(as5600, &as5600_angle) == HAL_OK) {
         // as5600_angle = 0-4095 (12-bit)
     }
     // encoder_count = 0-65535
     ```
  4. **USER CODE BEGIN 1** (exported variables): If needed

  Must NOT do:
  - NO modificar ninguna función MX_*_Init()
  - NO modificar código fuera de bloques USER CODE BEGIN/END
  - NO agregar interrupciones
  - NO agregar código de depuración serial (printf)
  - NO modificar .ioc

  Parallelization: Wave 2 | Blocked by: 1, 3 | Blocks: (none)
  References:
  - Core/Src/main.c (full file — already read)
  - Core/Inc/gpio.h (pin defines)
  Acceptance criteria:
  - `grep -c "Tools/as5600.h\|Tools/drv8323.h\|Tools/encoder.h" Core/Src/main.c` -> must find all 3
  - `grep -c "AS5600_Init\|DRV8323_Init\|encoder_Reset" Core/Src/main.c` -> must find all
  - All changes are inside USER CODE BEGIN/END blocks
  QA scenarios:
  - `grep "^[^/]" Core/Src/main.c | grep -c "USER CODE"` — changes ONLY in USER CODE blocks
  - No MX_ functions modified
  - Evidence: .omo/evidence/task-4-foc-firmware-drivers.md
  Commit: N

## Final verification wave
> Runs in parallel after ALL todos. ALL must APPROVE. Surface results and wait for the user's explicit okay before declaring complete.
- [x] F1. Plan compliance audit — verify Must Nots: no CubeMX file changes, no fw/ modifications
- [x] F2. Code quality review — check for C++ in C files, no F0 references remain, correct I2C/SPI API usage
- [x] F3. Pin consistency check — DRV8323 pins (PC13/PC14/PB13) match user spec
- [x] F4. DRV8323 register map verification — SPI protocol matches datasheet

## Commit strategy
No commits — documentation/driver files without build system integration.

## Success criteria
- Tools/as5600.h + .c — STM32G4 compatible, polling I2C, bug-free
- Tools/encoder.h — helper macros/functions for TIM1 encoder
- Tools/drv8323.h + .c — complete DRV8323 driver with SPI register access
- Core/Src/main.c — initialization example for all 3 drivers in USER CODE blocks
- Zero modifications to CubeMX-generated code (tim.c, spi.c, i2c.c, gpio.c, adc.c)
