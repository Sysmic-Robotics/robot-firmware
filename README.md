
# Sysmic Robotics Firmware

Firmware académico y de investigación para robots autónomos basados en microcontroladores **STM32F7**. El sistema implementa control en tiempo real, comunicación inalámbrica y gestión de actuadores/sensores, empleando una arquitectura modular y orientada a tareas bajo **FreeRTOS**.

---

## Estructura del Proyecto

```text
firmware/
├── Core/
│   ├── Inc/         # Encabezados de tareas, configuración global y periféricos
│   └── Src/         # Implementación de tareas, inicialización y lógica principal
├── Drivers/         # Controladores HAL y CMSIS para STM32F7
├── Middlewares/     # Librerías de terceros (ej. FreeRTOS)
├── Tools/           # Módulos de hardware: motores, sensores, radio, PID, etc.
├── Debug/           # Archivos de compilación y depuración
├── Release/         # Archivos de compilación para versión final
├── robot-firmware.ioc # Configuración STM32CubeMX (pines, clocks, RTOS)
├── STM32F767BITX_*.ld # Scripts de linker para memoria
└── README.md
```

---

## Descripción General

El firmware está diseñado para controlar robots móviles autónomos, integrando:

- **Control de movimiento** (Drive): gestión de motores y cinemática.
- **Comunicación inalámbrica** (Radio): intercambio de datos con estaciones base u otros robots mediante NRF24L01.
- **Mecanismo de pateo** (Kick): control de actuadores de disparo.
- **Sensado**: integración de sensores de distancia y detección de balón.
- **Gestión de recursos**: uso de FreeRTOS para tareas concurrentes y sincronización.

### Principales módulos y tareas

- `driveTask` (Core/Inc/drive_task.h, Core/Src/drive_task.c):
   - Control de velocidad y dirección de motores usando lazo cerrado (PID).
   - Procesamiento de comandos de movimiento.
- `radioTask` (Core/Inc/radio_task.h, Core/Src/radio_task.c):
   - Comunicación inalámbrica vía NRF24L01.
   - Recepción y envío de comandos y telemetría.
- `kickTask` (Core/Inc/kick_task.h, Core/Src/kick_task.c):
   - Control del mecanismo de pateo y dribbler.
- `ballDetectorTask` (Core/Inc/ball_detector_task.h, Core/Src/ball_detector_task.c):
   - Detección de balón mediante sensor de distancia.
- `system_globals` (Core/Inc/system_globals.h, Core/Src/system_globals.c):
   - Variables globales y configuración compartida entre tareas.
- **Drivers y herramientas** (`Tools/`):
   - `motor`, `encoder`, `PID`, `board`, `nrf24`, `vl6180x`, `MAX581x`, etc.: módulos reutilizables para hardware específico.

---

## Sistema Operativo: FreeRTOS

El firmware utiliza **FreeRTOS** para la gestión de tareas concurrentes, temporización y sincronización. Cada funcionalidad principal se implementa como una tarea independiente, permitiendo modularidad y escalabilidad.

---

## Dependencias

- STM32CubeIDE (o entorno compatible con STM32F7)
- FreeRTOS (integrado vía STM32CubeMX)
- HAL STM32F7xx Drivers
- Módulo NRF24L01 (u otro compatible)

---

## Compilación y Uso

1. Clonar el repositorio:
    ```bash
    git clone https://github.com/Sysmic-Robotics/robot-firmware.git
    ```
2. Abrir el proyecto en STM32CubeIDE.
3. Compilar y cargar el firmware al microcontrolador (ST-Link u otro programador compatible).
4. Conectar sensores, motores y módulos según el diseño de hardware.

---


## Principales Contribuciones

- Arquitectura modular basada en FreeRTOS para tareas concurrentes.
- Implementación de control de motores con lazo cerrado (PID).
- Comunicación inalámbrica robusta mediante NRF24L01.
- Integración de sensores de distancia (VL6180X) y detección de balón.
- Módulo de control de pateo y dribbler para robots de fútbol.
- Abstracción de hardware y drivers reutilizables para STM32F7.

---

## Detalles de Hardware

- **Microcontrolador:** STM32F767BIT6 (Cortex-M7, 216 MHz, FPU, 512 KB RAM, 2 MB Flash)
- **Módulo RF:** NRF24L01+ (SPI)
- **Sensores:**
   - VL6180X (sensor de distancia por I2C)
   - Encoders para medición de velocidad de motores
- **Actuadores:**
   - Motores DC con controladores PWM
   - Módulo de pateo (solenoide controlado por MOSFET)
   - Dribbler (motor adicional)
- **Otros:**
   - Leds de estado
   - Pines de identificación de placa
   - Conversores DAC (MAX581x/MAX5814)

El diseño de hardware está orientado a robots móviles de competencia (ej. fútbol robótico), pero es adaptable a otras plataformas STM32F7.

---

## Licencia

Proyecto académico de **Sysmic Robotics**. Uso restringido a fines educativos e investigación. Para uso comercial, contactar a los desarrolladores.
