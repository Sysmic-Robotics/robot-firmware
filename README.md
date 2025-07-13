# Sysmic Robotics Firmware

Este repositorio contiene el firmware desarrollado por **Sysmic Robotics** para robots autónomos basados en microcontroladores **STM32F7**. Aquí se encuentra el código fuente relacionado con el control del robot, incluyendo tareas de movimiento, comunicación inalámbrica y mecanismos de acción, así como las bibliotecas utilizadas.

---

## Sistema Operativo: FreeRTOS

El comportamiento del robot está gobernado por el sistema operativo en tiempo real **FreeRTOS**. Este paradigma se basa en definir tareas (funciones concurrentes) con diferentes prioridades. El sistema operativo se encarga de programar su ejecución en función del tiempo, los recursos disponibles y los eventos del sistema.

---

## Tareas Principales

### driveTask

Encargada del control de movimiento del robot. Utiliza datos de sensores y comandos recibidos para ajustar la velocidad y dirección de los motores mediante control en lazo cerrado.

### radioTask

Maneja la comunicación inalámbrica del robot. Se encarga del envío y recepción de datos a través del módulo de radiofrecuencia (NRF24L01 u otro), permitiendo la interacción con estaciones base u otros robots.

### kickTask

Controla el mecanismo de pateo del robot. Esta tarea administra la activación del sistema de disparo, ejecutándolo cuando se recibe la señal adecuada vía radio.

---

## Dependencias

- STM32CubeIDE o cualquier entorno compatible con STM32F7.
- FreeRTOS (versión incluida o integrada en STM32CubeMX).
- Biblioteca de controladores HAL para STM32F7.
- Driver de módulo RF (por ejemplo, NRF24L01).

---

## Uso

1. Clonar el repositorio:
   ```bash
   git clone https://github.com/Sysmic-Robotics/robot-firmware.git
   ```
2. Abrir el proyecto en STM32CubeIDE.
3. Compilar y cargar el firmware al microcontrolador mediante ST-Link u otro programador.
4. Conectar sensores, motores y módulos según el diseño de hardware del robot.

---

## Contribuciones

Las contribuciones son bienvenidas. Favor de realizar *pull requests* bien documentados y con código probado.

---

## Licencia

Este proyecto es propiedad de **Sysmic Robotics** y está disponible únicamente para fines académicos y de investigación. Para uso comercial, contactar con los desarrolladores.
