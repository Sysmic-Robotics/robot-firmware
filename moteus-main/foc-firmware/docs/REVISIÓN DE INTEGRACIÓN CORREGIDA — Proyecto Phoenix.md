### REVISIÓN DE INTEGRACIÓN CORREGIDA — Proyecto Phoenix

#### 1\. Identificación del Sistema y Estado del Firmware

Este documento técnico consolida la revisión arquitectónica y el estado de integración del firmware para la plataforma Phoenix, tras la implementación de las correcciones críticas en la cadena de control FOC.

* **MCU:**  STM32G431CBU6 (ARM Cortex-M4F)  
* **Frecuencia de Reloj:**  170 MHz (Boost Mode vía PLL, HSI de 16 MHz)  
* **Estado del Lazo FOC:**  Verificado y operativo a 20 kHz (período de 50 µs).  
* **Plataforma de Hardware:**  Phoenix Rev. A (derivada de arquitectura moteus-c1).

##### Resumen de Parámetros Críticos

Parámetro,Valor  
Frecuencia PWM,20 kHz (Center-aligned / Dual-edge)  
Frecuencia de Lazo FOC,20 kHz (TIM6 ISR)  
Tiempo de Ejecución ISR,\~8 µs @ 170 MHz  
Toolchain,CubeMX \+ STM32CubeIDE (GCC)

#### 2\. Matriz de Correcciones de Integración (Directiva 2026\)

Se ha validado la implementación de los siguientes módulos bajo la Directiva 2026, priorizando la estabilidad del lazo de corriente sobre implementaciones previas por software.| Hallazgo | Estado | Descripción de la Corrección || \------ | \------ | \------ || **C3 (Vbus)** | **Limitación de HW** | PA9 está asignado al Encoder (TIM1\_CH2). Se ha migrado la telemetría de Vbus a  **SPI2**  desde el master para alimentar el algoritmo SVM. || **Lazo FOC** | **Verificado** | Migración exitosa de la lógica de control desde el lazo principal a la interrupción periódica  **TIM6 ISR**  a 20 kHz. || **ADC Sampling** | **Verificado** | Configuración a 47.5 ciclos para garantizar el asentamiento del amplificador CSA del DRV8323. || **Trig LUT** | **Verificado** | Implementación de tabla de búsqueda de 1024 entradas para garantizar un determinismo temporal estricto. |

#### 3\. Configuración Detallada de Periféricos Analógicos y Digitales

##### 3.1 Mapeo de Pines ADC (Corriente de Fase)

Para la reconstrucción de la corriente de fase, se ha configurado la siguiente matriz de muestreo:| Fase | Pin | Canal ADC | Función || \------ | \------ | \------ | \------ || **Ia** | PA3 | ADC1\_IN4 | Detección de corriente (Fase A) || **Ib** | PA6 | ADC2\_IN3 | Detección de corriente (Fase B) || **Ic** | PB1 | ADC1\_IN12 | Detección de corriente (Fase C) |

##### 3.2 Interfaz de Encoder (TIM1)

El temporizador TIM1 opera en  **Modo Encoder TI1**  (conteo en flancos de TI1, nivel en TI2). Esta configuración utiliza los pines  **PA8 (TIM1\_CH1)**  y  **PA9 (TIM1\_CH2)** . Se confirma la eliminación de la lectura analógica en PA9, ya que este pin es físicamente necesario para el canal B del encoder incremental, lo que permite una resolución de 2000 CPR para el seguimiento de la posición.

##### 3.3 Comunicación SPI2 (Slave)

Dada la imposibilidad de utilizar el canal ADC2\_CH5 en PA9, la interfaz  **SPI2**  PF1 (SCK), PB14 (MISO), PA11 (MOSI) se ha configurado como esclavo en modo Full-Duplex de 16 bits. Esta vía recibe la telemetría de Vbus desde la placa master, alimentando el bloque de normalización de  **Space Vector Modulation (SVM)**  mediante el factor 1/Vbus, manteniendo así una ganancia de lazo de corriente consistente a pesar de las fluctuaciones de la batería.

#### 4\. Análisis de Problemas Críticos (C1-C6) y Resoluciones

##### C1: Tiempo de Muestreo ADC Insuficiente

La configuración inicial de 2.5 ciclos resultaba en mediciones inválidas por el ruido transitorio del CSA. Se ha ajustado a  **47.5 ciclos** , lo que representa un tiempo de muestreo de  **1.12 µs (@ 42.5 MHz ADC clock)** . Este ajuste cumple con el tiempo de asentamiento ( $t\_{SET}$ ) requerido por el DRV8323 para una ganancia de 20 V/V.

##### C2: Sincronización de Disparo (HW Trigger)

Se eliminó el disparo por software (Polling), que introducía un error de fase inaceptable. Ahora, los ADC se disparan mediante  **TIM2\_TRGO** , sincronizando la captura exactamente con el centro del ciclo PWM, donde el ruido de conmutación es mínimo y la corriente representa el valor promedio real.

##### C3: Conflictos de Pin PA9 y Compensación de Bus

Se ha determinado que el pin PA9 es crítico para el canal B del encoder. Intentos anteriores de usarlo como ADC2\_CH5 para censar voltaje eran erróneos. La resolución arquitectónica dicta que la compensación de voltaje para el algoritmo FOC debe provenir de la telemetría SPI2, asegurando que el torque sea independiente del voltaje de bus.

##### C4: Frecuencia del Lazo de Control

El control se ha movido de una frecuencia de 1 kHz (main loop) a  **20 kHz (TIM6 ISR)** . Esto proporciona el ancho de banda necesario para evitar la inestabilidad del controlador PI ante las frecuencias eléctricas de motores de alta velocidad.

##### C5: Latencia de Funciones Trigonométricas

El uso de sinf/cosf de la biblioteca estándar introducía un jitter variable y consumía ciclos de CPU excesivos. Se ha implementado una  **LUT (Lookup Table) de 1024 entradas** , permitiendo el cálculo de ángulos eléctricos con un costo computacional despreciable y determinismo temporal.

##### C6: Calibración de Offsets del CSA

Se integró una secuencia de calibración automática en la fase de habilitación del driver (DRV8323\_Enable). Mediante el control de los bits CAL\_A/B/C, se eliminan errores de offset de hasta ±1 A, garantizando que el punto cero de corriente sea preciso antes de cerrar el lazo de torque.

#### 5\. Análisis de Problemas Medios (M1-M5)

1. **M1 \- Skew Temporal en ADC1:**  Debido a que  **Ia (ADC1\_IN4)**  e  **Ic (ADC1\_IN12)**  comparten el ADC1, se muestrean secuencialmente en modo scan. Esto introduce un  **skew de \~353 ns**  entre fases, un factor de ruido conocido para la transformada de Park a altas RPM.  
2. **M2 \- Resolución del Encoder:**  Se mantiene el modo TI1; no obstante, se evalúa la migración a TI12 en futuras revisiones si se requiere duplicar la resolución de 2000 a 4000 CPR.  
3. **M3 \- Control de Estado HiZ:**  El pin  **PC15**  controla la alta impedancia del driver. Se ha configurado para estar en  **HIGH por defecto al arranque** , asegurando un estado de costa (coasting) seguro hasta que el firmware tome el control explícito.  
4. **M4 \- Monitoreo de Fallas de Hardware:**  Se utiliza el pin  **PB13 (nFAULT)**  vinculado a una interrupción EXTI de prioridad 0 para desactivar el PWM de forma inmediata ante fallas de sobrecorriente o térmicas del DRV8323.  
5. **M5 \- Simetría en ADC2:**  Se ha implementado la calibración independiente para el ADC2 (fase Ib), evitando desbalances en la transformada de Clarke que anteriormente degradaban la linealidad del torque.

#### 6\. Verificación de Lazo de Control y Rendimiento

La optimización hacia una arquitectura orientada a interrupciones y el uso de tablas de búsqueda han resultado en un sistema altamente eficiente.| Métrica de Rendimiento | Implementación Anterior | Phoenix (Firmware Corregido) || \------ | \------ | \------ || Frecuencia de Muestreo | 1 kHz (Software Polling) | 20 kHz (Hardware Triggered) || Método Trigonométrico | math.h (Variable) | LUT 1024 (Determinístico) || Sincronización PWM | Ninguna | Center-aligned (TIM2\_TRGO) || Tiempo de Ejecución ISR | N/A | \~8 µs (Determinismo Temporal) |  
"Con un tiempo de ejecución de la ISR de aproximadamente 8 µs operando a una frecuencia de 20 kHz, el algoritmo FOC consume únicamente el  **16% de la ventana de control de 50 µs** , dejando un margen sustancial para las tareas de comunicación SPI2 y la lógica de supervisión del sistema."

#### 7\. Conclusión y Roadmap de Integración

El firmware Phoenix se encuentra en estado  **"Listo para integración de alto nivel"** . La infraestructura analógica y el lazo de corriente de 20 kHz proporcionan una base robusta para las capas de control de velocidad y posición.**Prioridades Técnicas Inmediatas:**

1. Validación del protocolo  **SPI Slave 16-bit Full-Duplex**  para la recepción de telemetría Vbus y comandos de modo.  
2. Caracterización y calibración fina de los offsets de corriente remanentes bajo carga térmica de operación.  
3. Ajuste de las ganancias PID de la cascada de control (posición y velocidad) según el perfil de carga específico del actuador.

