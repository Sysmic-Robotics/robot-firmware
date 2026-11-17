# SysmicProyect — RoboCup Small Size League (SSL)

Sistema completo de robótica para la RoboCup Small Size League. Consiste en tres subsistemas principales que trabajan en conjunto para controlar robots en una cancha de fútbol con visión por computadora, planificación de trayectorias y estrategias de IA.

## Subsistemas

| Subsistema | Plataforma | Lenguaje | Propósito |
|---|---|---|---|
| **Robot Firmware** | STM32F767ZI | C (FreeRTOS) | Control de motores, sensores, radio |
| **Base Station** | STM32F1 | C | Puente UART → nRF24L01+ |
| **Engine** | PC | Rust (Iced) | Visión, planificación, GUI, Lua AI |

## Repositorio

- **MCU**: Cortex-M7, 216 MHz, 2 MB Flash, 512 KB SRAM
- **Radio**: nRF24L01+ a 2.4 GHz, canal 0x6B (107)
- **Visión**: SSL Vision Protocol (multicast UDP: 224.5.23.2:10020)
- **Referee**: SSL Game Controller (multicast UDP: 224.5.23.1:10003)
- **Simulación**: grSim (UDP: 20011)
- **Estrategia**: Lua 5.4 con API de control y estado

## Estructura del proyecto

```
SysmicProyect/
├── robot/           # Firmware del robot (STM32F7 + FreeRTOS)
├── basestation/     # Firmware de la base station (STM32F1)
├── engine/          # Motor de control en PC (Rust)
├── lua/             # Scripts de estrategia (Lua 5.4)
└── docs/            # Documentación del proyecto
```

## Documentación

- [Arquitectura del sistema](architecture.md)
- [Firmware del robot](robot-firmware.md)
- [Base Station](base-station.md)
- [Engine (Rust)](engine.md)
- [Protocolo de comunicación](communication-protocol.md)
- [Estrategia Lua](lua-strategy.md)
- [Hardware](hardware.md)
- [Resumen ejecutivo](SUMMARY.md)
- [Bugs y Mejoras](bugs-and-improvements.md)

## Diagramas

- [Arquitectura del sistema](diagrams/system-architecture.md)
- [Flujo de datos](diagrams/data-flow.md)
- [Ciclo de control del robot](diagrams/robot-control-loop.md)
- [Stack de software](diagrams/software-stack.md)
