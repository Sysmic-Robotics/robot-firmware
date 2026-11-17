# Especificación Técnica: Migración de Arquitectura de Control y Localización para Robot de RoboCup SSL

Este documento detalla la reestructuración del firmware (STM32F7, Cortex-M7 a 216 MHz) y del software central de la PC (Engine) para un robot de la Small Size League (SSL). El objetivo es migrar el Filtro de Kalman Extendido (EKF) de la PC al robot para cerrar el lazo de posición localmente a 1 kHz, eliminando la latencia de visión y optimizando el tiempo de CPU.

---

## 1. Diagnóstico y Cuellos de Botella Actuales

1. **I2C Bloqueante para Actuación:** La tarea de FreeRTOS corre a 1 kHz (ventana de 1000 µs). Enviar la referencia analógica (4 bytes por motor) mediante I2C bloqueante al DAC consume ~450 µs (~45% de la tarea).
2. **EKF en el Engine (PC):** El EKF actual se ejecuta en la PC e intenta fusionar datos en tiempo real (encoders) con datos del pasado (cámara a 60 FPS con un delay de visión de 15 a 30 ms). Esto genera errores de hasta 10 cm cuando el robot se desplaza a 4 m/s.
3. **Control por Velocidad Remota:** La PC envía comandos $(V_x, V_y, V_\theta)$ continuamente por radio. Si se pierde un paquete, el robot mantiene la velocidad anterior de forma ciega.

---

## 2. Nueva Arquitectura Propuesta

### A. En la PC (Engine)
* **Rol:** Actuar exclusivamente como un "GPS de verdad absoluta" asíncrono.
* **Acción:** Deja de ejecutar el EKF. Procesa el cuadro de la cámara, calcula la posición absoluta cruda del robot $(X_{cam}, Y_{cam}, \theta_{cam})$ y determina la marca de tiempo exacta del frame ($T_{cam}$).
* **Frecuencia de Radio (Bajada):** Baja de 1 kHz a ~60 Hz (alineado con los FPS de la cámara). Envía únicamente los **Waypoints de destino** $(X_{target}, Y_{target}, \theta_{target})$ junto con la corrección de la cámara del pasado: $[X_{cam}, Y_{cam}, \theta_{cam}, T_{cam}]$.

### B. En el Robot (STM32F7 con FreeRTOS a 1 kHz)
* **Rol:** Autonomía local en tiempo real y cierre del lazo de posición.
* **Predicción (1 kHz):** El EKF local se propaga continuamente a alta frecuencia usando los sensores internos de latencia cero: la **IMU** (acelerómetro/giroscopio) y los **encoders** de las 4 ruedas omnidireccionales.
* **Generación de Trayectorias:** El robot recibe el destino de la PC y calcula localmente perfiles de aceleración trapezoidales o S-Curves para evitar el patinaje (*wheel slip*).

---

## 3. Hoja de Ruta para la Implementación (Paso a Paso)

### Paso 1: Liberación de la CPU con DMA (Prioridad Alta)
* **Objetivo:** Recuperar los 450 µs desperdiciados por el I2C bloqueante.
* **Acción:** Configurar los periféricos para realizar la transmisión de las referencias al DAC mediante **DMA no bloqueante**. La CPU solo debe depositar los datos en un búfer de memoria RAM y disparar el DMA (< 1 µs de carga de CPU).

### Paso 2: Portar el EKF de 7 Estados al STM32F7
* **Vector de Estado ($X$):** $[x, y, v_x, v_y, \sin\theta, \cos\theta, v_\theta]^T$.
* **Requisitos de Código:**
  * Utilizar variables estrictamente de tipo `float` (32 bits) para aprovechar la **FPU por hardware** del Cortex-M7. No usar `double` (64 bits).
  * Implementar las operaciones matriciales utilizando la biblioteca optimizada **CMSIS-DSP** de ARM (ej. `arm_mat_mult_f32`).
  * *Tiempo de cómputo estimado con CMSIS-DSP:* ~28 µs (frente a los ~60 µs de un código C genérico).

### Paso 3: Implementación del Búfer de Historia (Máquina del Tiempo)
* **Objetivo:** Resolver el retraso (delay) de la cámara sin generar saltos bruscos en el control actual.
* **Acción:** Reservar un búfer circular en la memoria RAM del STM32F7 que almacene los últimos 40 o 50 estados calculados por el EKF (cada posición incluye el vector de estado $X$, la matriz de covarianza $P$ y un índice de tiempo/timestamp).

### Paso 4: Fusión Asíncrona con Retraso (Delayed Measurement Update)
Cuando el módulo de radio del robot reciba el paquete de la cámara con la posición corregida del pasado $[X_{cam}, Y_{cam}, \theta_{cam}]$ y su marca de tiempo $T_{cam}$:
1. El firmware detiene momentáneamente la actualización normal del lazo.
2. Recupera del búfer circular el estado guardado en el instante exacto $T_{cam}$.
3. Ejecuta el **Paso de Corrección de Kalman (Measurement Update)** tradicional, contrastando la medición cruda de la cámara contra lo que el robot estimaba en ese instante del pasado.
4. **Re-propagación rápida:** Corre un bucle `for` ultra veloz para recalcular (re-propagar) los pasos de predicción desde el instante corregido $T_{cam}$ hasta el milisegundo actual (presente).
5. Retoma el lazo de control con el presente corregido y limpio de deriva (*drift*).

---

## 4. Prompts sugeridos para OpenCode / Asistente de IA

Para empezar a resolver el código, puedes usar los siguientes prompts específicos:

* **Para el Paso 1:** *"Necesito configurar la transmisión de un búfer de 4 bytes hacia un DAC utilizando DMA no bloqueante en un STM32F7 usando HAL o LL. Muéstrame cómo inicializar el DMA y cómo disparar la transferencia dentro de una tarea periódica de FreeRTOS sin bloquear la CPU."*
* **Para el Paso 2:** *"Ayúdame a portar un Filtro de Kalman Extendido (EKF) de 7 estados escrito en Python/C++ genérico a código C optimizado para STM32F7 utilizando la biblioteca CMSIS-DSP (`arm_math.h`). El vector de estado es $[x, y, v_x, v_y, \sin\theta, \cos\theta, v_\theta]$. Asegúrate de usar solo floats de 32 bits."*
* **Para el Paso 3 y 4:** *"Diseña una estructura de datos en C para un búfer circular que actúe como historial de estados de un Filtro de Kalman en un microcontrolador. Necesito que guarde el vector `float X[7]` y la matriz `float P[7][7]` indexados por un timestamp. Además, escribe la lógica de una función que busque un estado pasado, aplique una corrección y corra un bucle para re-propagar los estados intermedios hasta el índice actual."*
