# Base Station (STM32F103)

## Propósito

Puente transparente entre la PC (USB Serial) y los robots (nRF24L01+ 2.4 GHz).

```
PC ──USB Serial──> STM32F103 ──nRF24 TX (0x6B)──> Robot
                    (115200 baud)
PC <──USB Serial── STM32F103 <──nRF24 RX (0x6C)── Robot
```

## Hardware

- **MCU**: STM32F103C8T6 (BluePill) @ 72 MHz (HSE 8 MHz → PLL ×9)
- **UART**: USART1 @ 115200 baud, 8N1, DMA RX (Channel5) + IT TX (HAL_UART_Transmit_IT)
- **Radio**: 2 módulos nRF24L01+ (SPI1 compartido)

### Pin Mapping

| Señal | Función | Pin |
|-------|---------|-----|
| SPI1_SCK | Reloj SPI | PA5 |
| SPI1_MISO | Master In | PA6 |
| SPI1_MOSI | Master Out | PA7 |
| TX nRF24 CSN | Chip Select TX | PB0 |
| TX nRF24 CE | Chip Enable TX | PA4 |
| RX nRF24 CSN | Chip Select RX | PB1 |
| RX nRF24 CE | Chip Enable RX | PA3 |
| USART1_TX | Serial TX (PC) | PB6 (AFIO remap) |
| USART1_RX | Serial RX (PC) | PB7 (AFIO remap) |
| LED | Indicador | PC13 |
| (libre) | — | PC15 |

### Direcciones de Radio

| Dirección | Valor | Uso |
|-----------|-------|-----|
| TX (commands) → tx_device | `sysrx` | BS transmite → robot recibe (canal 0x6B) |
| RX (telemetry) → rx_device | `systx` | Robot transmite → BS recibe (canal 0x6C, pipe1) |
| TX (corrección) → tx_device | `sysrc` | BS transmite corrección de cámara → robot Pipe 1 (18 bytes) |

El tx_device usa `rx_node_addr` ("sysrx") como dirección TX, porque el robot escucha en "sysrx". El rx_device usa `tx_node_addr` ("systx") en pipe1, porque el robot transmite en "systx". Canales separados (0x6B / 0x6C) permiten operación simultánea sin interferencia.

Para la corrección de cámara, el tx_device cambia temporalmente su dirección TX a `correction_node_addr` ("sysrc") — la dirección de la Pipe 1 del robot — envía los 18 bytes y restaura `rx_node_addr` ("sysrx").

## Firmware

### Archivos

| Archivo | Descripción |
|---------|-------------|
| `Src/main.c` | Bucle principal, init periféricos, TX/RX radio |
| `Src/nrf24.c` | Driver SPI para nRF24L01+ |
| `Inc/nrf24.h` | API del driver nRF24L01+ |
| `Inc/main.h` | Header STM32F1 HAL |

### Flujo Principal

```
Init:
  1. SPI1, UART1 DMA RX (Channel5) + IT TX, GPIO
  2. TX nRF24 (tx_device): canal 0x6B, TX addr = rx_node_addr ("sysrx"), AA off, 32 bytes, modo TX
  3. RX nRF24 (rx_device): canal 0x6C, pipe1 addr = tx_node_addr ("systx"), AA off, 32 bytes, modo RX, CE=HIGH
  4. UART DMA RX: 30 bytes, interrupt on idle

Loop (event-driven por UART idle interrupt):
  1. Esperar send_flag (señaliza por HAL_UART_RxCpltCallback en IDLE)
  2. Leer bytes recibidos por DMA: serial_len = 30 - CNDTR
  3. Reconfigurar DMA para siguiente recepción
  4. Si ready_data[0] == 0xFE (corrección de cámara, 18 bytes):
     nRF24_SetAddr(tx_device, PIPETX, correction_node_addr)  // "sysrc"
     nRF24_TxPacket(tx_device, buf, 18)
     nRF24_SetAddr(tx_device, PIPETX, rx_node_addr)          // restaurar "sysrx"
     Si no: nRF24_TxPacket(tx_device, buf, serial_len > 0 ? serial_len : 30) → comandos
  5. Poll rx_device RX FIFO (canal 0x6C)
  6. Si hay telemetry y !uart_tx_busy:
     memcpy(uart_tx_buf, rx_buf, 32)
     HAL_UART_Transmit_IT(&huart1, uart_tx_buf, 32)
     uart_tx_busy = 1
```

### Protocolo Serial (PC ↔ Base Station)

| Dirección | Tamaño | Formato |
|-----------|--------|---------|
| PC → BS | 30 bytes | 6 robots × 5 bytes (comandos) |
| PC → BS | 18 bytes | Corrección de cámara (header 0xFE) |
| BS → PC | 32 bytes | Telemetría EKF del robot |

Los comandos se pasan directamente de UART a nRF24 sin modificación. La corrección de cámara (18 bytes, primer byte `0xFE`) se multiplexa a la Pipe 1 del robot cambiando temporalmente la dirección TX del `tx_device` a `"sysrc"`.

### HAL_UART_TxCpltCallback

Se invoca cuando `HAL_UART_Transmit_IT` termina de enviar. Su única función es limpiar el flag `uart_tx_busy = 0`, permitiendo que el loop envíe el siguiente paquete de telemetría. No hace nada más.

### USART1_IRQHandler (override)

El handler IRQ de USART1 solo llama a `HAL_UART_RxCpltCallback` cuando el flag IDLE está seteado (`USART_SR_IDLE`). Esto evita que se dispare durante TX complete u otros eventos IRQ, lo que rompía la recepción DMA.

```c
// Solo reaccionar a IDLE (fin de trama), no a TX complete u otros
if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    HAL_UART_RxCpltCallback(&huart1);
}
```

### Telemetría (32 bytes, robot → BS → PC)

| Offset | Size | Campo | Tipo |
|--------|------|-------|------|
| 0 | 1 | ID(3) + ball(1) + kicker(1) + sat(4) | uint8 |
| 1-2 | 2 | ball_range | uint16 LE |
| 3-6 | 4 | x_est | float32 |
| 7-10 | 4 | y_est | float32 |
| 11-14 | 4 | vx_est | float32 |
| 15-18 | 4 | vy_est | float32 |
| 19-22 | 4 | theta_est | float32 |
| 23-26 | 4 | omega_est | float32 |
| 27-30 | 4 | gyro_z | float32 |
| 31 | 1 | ang_acc_limit × 10 | uint8 |

## Estado Actual — Dual-Channel (0x6B Commands / 0x6C Telemetry)

Actualmente cada robot tiene **1 módulo nRF24L01+**. El robot alterna entre:
1. **RX** (recibe comando del BS)
2. **TX** (envía telemetría al BS)

El BS usa **2 módulos nRF24** con canales separados:
- **tx_device** (canal 0x6B): transmite comandos al robot
- **rx_device** (canal 0x6C): recibe telemetría del robot, CE=HIGH continuo

El uso de canales distintos permite que la BS pueda estar escuchando telemetría mientras transmite comandos, sin interferencia entre los módulos.

### Timing

```
BS TX (comando 30B) ──────────────>│
                                   │ ~1ms TX
                                   │
Robot: RX → procesa → TX (telemetry 32B) ──>│
                                             │ ~1ms TX
                                             │
BS RX: escucha ──────────────────────────────│ recibe telemetría
```

**Nota**: Con 6 robots compartiendo canal, las respuestas pueden colisionar. El firmware del robot usa `osDelay(robot_id)` para staggering, pero colisiones son posibles. Futuro: 2 nRF por robot (TX dedicado + RX dedicado) para full-duplex.

## Canal de Corrección de Cámara (Pipe 1, 18 bytes)

Además de los comandos de 30 bytes, la BS multiplexa las **correcciones de cámara** del engine hacia los robots. Estas viajan por la **Pipe 1** del robot (dirección `"sysrc"`, payload 18 bytes), distinta de la Pipe 0 de comandos (`"sysrx"`, 30 bytes).

### Multiplexación en el loop

El engine envía la corrección (18 bytes, primer byte `0xFE`) por USB. La BS detecta el header y cambia temporalmente la dirección TX de su `tx_device`:

```c
if (ready_data[0] == 0xFE) {
    /* Camera-correction packet (18 bytes) → dedicated correction pipe. */
    nRF24_SetAddr(&tx_device, nRF24_PIPETX, correction_node_addr);  // "sysrc"
    nRF24_TxPacket(&tx_device, ready_data, 18);
    nRF24_SetAddr(&tx_device, nRF24_PIPETX, rx_node_addr);          // restaurar "sysrx"
} else {
    nRF24_TxPacket(&tx_device, ready_data, serial_len > 0 ? serial_len : 30);
}
```

- `correction_node_addr` = `{'s','y','s','r','c'}` (5 bytes).
- El cambio de dirección es **temporal**: se restaura `rx_node_addr` ("sysrx") inmediatamente después del envío para que los comandos de 30 bytes sigan llegando a la Pipe 0 del robot.
- El robot recibe el paquete por Pipe 1 y lo procesa con `processCameraCorrection()` (ver `robot-firmware.md`).

## Bugs Corregidos

### 1. CE Compartido (CRITICAL)

**Problema**: Ambos módulos nRF24 usaban CE=PA4. Cuando el TX terminaba de enviar y CE bajaba, el RX también perdía CE y dejaba de escuchar. Las respuestas del robot llegaban después de que CE bajaba.

**Solución**: RX CE movido a PA3 (pin libre). RX CE se mantiene HIGH continuamente.

### 2. nRF24_ClearIRQFlagsRx Inexistente

**Problema**: `main.c` llamaba `nRF24_ClearIRQFlagsRx()` que no existía en `nrf24.c`.

**Solución**: Cambiado a `nRF24_ClearIRQFlags()`.

### 3. RX Payload Overflow

**Problema**: RX configurado con 30 bytes pero robot envía 32 bytes de telemetría.

**Solución**: Ambos TX y RX configurados a 32 bytes.

### 4. Address Mismatch (CRITICAL)

**Problema**: La base station RX escuchaba en `tx_node_addr` ("systx") pero el robot transmite en `rx_node_addr` ("sysrx"). El robot usa "sysrx" para TX y RX, por lo que la base station nunca recibía telemetría.

**Solución (original)**: Cambiado pipe1 de rx_device de `tx_node_addr` a `rx_node_addr`.

**Estado actual**: rx_device pipe1 ahora usa `tx_node_addr` ("systx"), que es correcto. El robot transmite en `tx_node_addr` ("systx") y el BS escucha en esa dirección. El cambio original fue un workaround temporal; la solución final es la dirección correcta para la arquitectura dual-channel.

### 5. SPI Clock Too Fast (MODERATE)

**Problema**: Con 2 módulos nRF24 en el bus SPI compartido, el reloj SPI a 9MHz (PRESCALER_8) causaba corrupción de datos.

**Solución**: SPI PRESCALER_8 → PRESCALER_32 (~2.25 MHz).

### 6. nRF24_TxPacket Timeout (MODERATE)

**Problema**: `nRF24_TxPacket()` podía quedar en loop infinito si el módulo nRF24 no respondía.

**Solución**: Agregado timeout de 100ms en `while (!(tx_status & nRF24_FLAG_TX_DS))`.

### 7. PA9/PA10 UART Quemados (HARDWARE)

**Problema**: FTDI conectado a 5V quemó los pines PA9/PA10 (UART1) en una board.

**Solución temporal**: Remap USART1 a PB6(TX)/PB7(RX) via `__HAL_AFIO_REMAP_USART1_ENABLE()` en `HAL_MspInit`. Cambiar `HAL_UART_MspInit` GPIO de PA9/PA10 (GPIOA) a PB6/PB7 (GPIOB).

**Nota**: Para revertir (board nueva): eliminar `__HAL_AFIO_REMAP_USART1_ENABLE()`, restaurar GPIO PA9/PA10.

### 8. UART Blocking TX (MODERATE)

**Problema**: `HAL_UART_Transmit()` era blocking, lo que bloqueaba el loop principal durante cada envío de telemetría a la PC. Con telemetría de 32 bytes a 115200 baud, cada transmisión tomaba ~3ms, causando pérdida de paquetes del robot.

**Solución**: Cambiado a `HAL_UART_Transmit_IT()` (interrupt-driven, non-blocking). Se agregó el flag `uart_tx_busy` para evitar envíos concurrentes. El callback `HAL_UART_TxCpltCallback` limpia el flag cuando la transmisión termina.

### 9. USART1 IRQ Handler ROMPE RX DMA (CRITICAL)

**Problema**: El handler `USART1_IRQHandler` original llamaba a `HAL_UART_RxCpltCallback()` en cada interrupción (TX complete, RXNE, IDLE, etc.). Esto interfería con la recepción DMA: cada TX complete disparaba un callback que corrompía el estado DMA.

**Solución**: El handler ahora solo llama a `HAL_UART_RxCpltCallback` cuando el flag IDLE está seteado (`USART_SR_IDLE`). Esto garantiza que el callback solo se ejecute al final de una trama completa, no por eventos de TX.

### 10. UART TX FIFO Drain (LOW)

**Problema**: El código original usaba un `while` loop para drenar el FIFO de UART TX, lo que provocaba que se enviaran datos incompletos o se perdieran paquetes cuando llegaban varios paquetes de telemetría en rápida sucesión.

**Solución**: Cambiado a un `if` guard: solo se envía un paquete por ciclo del loop. Si `uart_tx_busy` está activo, el paquete se descarta (o se encola). Esto evita el drain agresivo que perdía datos.

## Notas de Debugging

### Dual-nRF24 Board — Verificación de Inicialización

Al usar 2 módulos nRF24 en SPI compartido:

1. **MX_GPIO_Init**: Asegurar que CSN de ambos módulos esté en HIGH después de init (PB0=HIGH, PB1=HIGH)
2. **SPI PRESCALER**: Usar PRESCALER_32 mínimo (2.25 MHz) para evitar corrupción
3. **Status registers**: Leer `nRF24_GetStatus()` después de init — valor 14 (0x0E) indica configuración correcta
4. **CE separation**: TX CE=PA4, RX CE=PA3 — NUNCA compartir CE

### Telemetría RX — Checklist

1. rx_device escucha en canal **0x6C**, pipe1 con dirección `tx_node_addr` ("systx")
2. rx_device en modo RX, 32 bytes payload
3. RX CE=HIGH continuo (no pulsar)
4. tx_device envía comando en canal 0x6B → poll rx_device RX FIFO en 0x6C
5. Si hay datos y `!uart_tx_busy`: `memcpy` → `HAL_UART_Transmit_IT` forward a PC (32 bytes)

## Futuro — Full-Duplex (2 nRF por robot)

Con 2 módulos nRF24 por robot y 2 en la basestation:

```
BS TX_nrf ──> Robot RX_nrf    (comandos dedicados)
BS RX_nrf <── Robot TX_nrf    (telemetría dedicada)
```

Ventajas:
- Sin colisiones (cada dirección tiene canal dedicado)
- Telemetría continua sin esperar turno
- Mayor frecuencia de update (60+ Hz por robot)
