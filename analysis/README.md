# Analysis Tools — Sysmic Robotics

Suite de herramientas Python para análisis de logs de robot, calibración de
controladores PID y validación multi-sensor.

## Requisitos

```bash
pip install -r analysis/requirements.txt
# matplotlib, numpy
```

---

## 1. CSV Analyzer — `csv_analyzer.py`

Analiza logs de robot capturados con STM32Monitor. Detecta escalones en la
señal de comando y calcula métricas de respuesta temporal (gain, rise time,
overshoot, settling time, etc.).

### Formato del CSV

El log debe tener una columna de tiempo y una columna de comando (referencia).
Ejemplo típico para calibración VTH:

```
time,v_vel[2],vth_enc,vth_est,
0.000,0,0,0.00018,
0.001,4,0.016,0.012,
0.002,4,0.306,0.228,
```

Auto-detecta la columna de comando buscando nombres que contengan
`v_vel`, `cmd`, `ref`, `setpoint`, `target` o `command`.

### Comandos

```bash
# Análisis básico
python3 analysis/csv_analyzer.py Log_myVariables_*.csv

# Especificar columna de comando
python3 analysis/csv_analyzer.py archivo.csv --cmd v_vel[2]

# Todos los escalones
python3 analysis/csv_analyzer.py archivo.csv --all-steps

# Sin gráficos (solo texto)
python3 analysis/csv_analyzer.py archivo.csv --no-plot

# Comparar dos logs (antes/después)
python3 analysis/csv_analyzer.py log_v1.csv log_v2.csv --compare

# Exportar métricas a JSON
python3 analysis/csv_analyzer.py archivo.csv --export resultados.json

# Combinado
python3 analysis/csv_analyzer.py archivo.csv -c v_vel[2] -a -e res.json -n

# Ejemplo VTH
python3 analysis/csv_analyzer.py Vth-40+4.csv --cmd v_vel[2] --all-steps
```

### Interpretación de métricas

| Métrica             | Descripción                                   | Buen valor     |
|---------------------|-----------------------------------------------|----------------|
| `gain`              | `(ss_meas - cmd_before) / delta`              | 0.90 – 1.10    |
| `ss_error`          | `cmd_after - ss_meas`                         | < 0.1          |
| `rise_time`         | Tiempo a 63% del steady-state                 | < 100 ms       |
| `overshoot_pct`     | Sobreimpulso porcentual                       | < 10%          |
| `settle_time_2pct`  | Tiempo de establecimiento banda 2%            | < 200 ms       |
| `settle_time_5pct`  | Tiempo de establecimiento banda 5%            | < 150 ms       |
| `tau_decay`         | Constante de tiempo al retornar a baseline    | —              |
| `latency_ms`        | Latencia comando → respuesta inicial          | < 10 ms        |
| `clipped`           | Señal saturada en 0 o ANALOG_MAX             | evitar         |
| `noisy`             | Ruido > 5% en estado estacionario            | evitar         |

---

## 2. PID Tuner — `pid_tuner.py`

Sugiere ganancias PID a partir de datos de step response. Tiene **dos modos**
según el tipo de datos disponibles:

### Modo closed-loop (default) — `pid_tuner.py archivo.csv --current Kp,Ki,Kd`

Analiza la respuesta actual del sistema en lazo cerrado y sugiere ajustes
heurísticos. **No requiere un modelo del plant** — solo las ganancias actuales.

```bash
# Análisis + ajuste heurístico con ganancias actuales
python3 analysis/pid_tuner.py Vth-40+4.csv -c v_vel[2] -m vth_est --current 2.0,6.0,0.0
```

Métricas calculadas: rise time, overshoot, ss_error, settling time, latency,
noise level. Basado en estas, sugiere deltas direccionales para Kp/Ki/Kd.

### Modo open-loop — `pid_tuner.py archivo.csv --open-loop`

Extrae un modelo FOPDT (First Order Plus Dead Time) del plant a partir de
un step response en lazo abierto, y aplica reglas de sintonía clásicas:

| Método | Característica |
|--------|---------------|
| **Ziegler-Nichols** | Agresivo, punto de partida |
| **Tyreus-Luyben** | Más conservador |
| **IMC (λ-tuning)** | Robusto, ajustable vía λ |

```bash
# Extraer FOPDT + tuning rules desde step en lazo abierto
python3 analysis/pid_tuner.py Vth_ol.csv -c v_vel[2] -m vth --open-loop
```

### Parámetros directos

Si ya conoces los parámetros FOPDT del plant (K, L, T):

```bash
python3 analysis/pid_tuner.py --plant K=1.5,L=0.02,T=0.15
```

### Output típico (closed-loop)

```
PID Tuner — Closed-Loop Analysis
Vth-40+4.csv  |  Step 1: 0.0 -> -4.0
── Current Performance ──
  Rise time (63%):     113ms
  Overshoot:           6.4%
  Settling time (2%):  652ms
  Steady-state err:    -0.236
── Heuristic Adjustment ──
  Current:   Kp=2.0000  Ki=6.0000  Kd=0.0000
  Suggested: Kp=2.0000  Ki=6.3540  Kd=0.0000
  Delta:       +0.0000    +0.3540    +0.0000
  Overshoot aceptable (6.4%)
  Error estacionario (-0.236): aumentar Ki
── Copy to drive_task.c (lines 146-151) ──
  robotPidParams.Kp = 2.0000f;
  robotPidParams.Ki = 6.3540f;
  robotPidParams.Kd = 0.0000f;
```

### Convención de ganancias

Compatibles con `PID_Init()` en el firmware:
- `Kp`: ganancia proporcional directa
- `Ki`: ganancia integral (acumula `Ki * error * dt`)
- `Kd`: ganancia derivativa sobre medición (`-Kd * (measure - lastMeasure)`)
- `sampleTime`: 5 ms (200 Hz) para el PID outer loop

---

## 3. Sensor Compare — `sensor_compare.py`

Valida la calidad de las estimaciones del robot comparando dos fuentes de datos
(ej: telemetría interna @1kHz vs cámara externa @60Hz). Interpola la señal de
menor tasa a la línea temporal de mayor tasa y calcula estadísticas de error.

### Comandos

```bash
# Comparar variable específica
python3 analysis/sensor_compare.py telemetria.csv camara.csv --var vth_est

# Comparar múltiples variables
python3 analysis/sensor_compare.py telemetria.csv camara.csv --var vx vy vth

# Comparar todas las variables comunes
python3 analysis/sensor_compare.py telemetria.csv camara.csv --all

# Exportar a JSON con estadísticas
python3 analysis/sensor_compare.py telemetria.csv camara.csv --var vth --export comp.json

# Especificar columna de tiempo
python3 analysis/sensor_compare.py telemetria.csv camara.csv --var vx --time-col timestamp

# Sin gráficos
python3 analysis/sensor_compare.py telemetria.csv camara.csv --var vth --no-plot
```

### Métricas calculadas

| Métrica              | Descripción                                  |
|----------------------|----------------------------------------------|
| `rms_error`          | Raíz del error cuadrático medio              |
| `mean_error`         | Error medio (bias sistemático)               |
| `std_error`          | Desviación estándar del error                |
| `max_abs_error`      | Error absoluto máximo                        |
| `correlation`        | Coeficiente de correlación de Pearson        |
| `delay_samples`      | Delay estimado por correlación cruzada       |
| `delay_seconds`      | Delay en segundos                            |

### Gráficos

- **Panel superior**: overlay de ambas señales con RMS/bias/corr
- **Panel medio**: error instantáneo con bandas de ±1σ
- **Panel inferior**: histograma de distribución del error

---

## 4. Position Loop Designer — `position_loop_design.py`

Diseña las ganancias del **lazo de posición X/Y del engine** (60 Hz) a partir
del plant de velocidad del robot (FOPDT K, L, T) y las valida en una simulación
discreta de la cascada completa:

```
field error -> [rotated by fused theta into body frame] ->
    engine position PID (60 Hz) -> vx/vy command (m/s, clamped ±2.0)
    -> robot velocity response (FOPDT K,L,T) -> integrate -> fused position
```

**Distinción clave de tasas**: el lazo de velocidad del robot corre a 200 Hz
(firmware `drive_task.c`); el lazo de posición del engine corre a ~60 Hz
(Ts = 16.67 ms) y vive en el **engine** (prototipo Lua T8 / `PositionPID` Rust T9).

```bash
# Diseño con el plant identificado (T2 dry-run sobre TestescalonVx.csv)
python3 analysis/position_loop_design.py

# Parámetros explícitos del plant
python3 analysis/position_loop_design.py --K 0.381 --L 0.095 --T 0.245 --lambda 1.2

# Exportar ganancias + gráfico
python3 analysis/position_loop_design.py --export tune.json --png step.png

# Sin gráficos
python3 analysis/position_loop_design.py --no-plot
```

Ganancias verificadas (T7, `robot/.omo/evidence/task-7-characterize-vxvy-close-xy-loop.json`):
`Kp=4.3745`, `Ki=1.8227`, `Kff=2.6247` (el loop de posición usa Kff=0.0),
`output_max=2.0 m/s`, `integral_max=1.0`.

## 5. Scripts de step open-loop — `run_openloop_vx.sh` / `run_openloop_vy.sh`

Un escalón open-loop por invocación (contrato "one step per run"): deshabilita
el PID del eje (`pid_vx_enable`/`pid_vy_enable`), verifica los gates, graba
(engine + stm32monitor), ejecuta el step Lua y re-habilita el PID.

```bash
./run_openloop_vx.sh 1.0             # +1.0 m/s vx step
./run_openloop_vx.sh -0.6           # -0.6 m/s vx step
./run_openloop_vx.sh --dry-run 1.0  # imprimir comandos sin ejecutar
./run_openloop_vy.sh 0.8            # +0.8 m/s vy step
```

El PID del otro eje lineal y el theta-hold **permanecen habilitados** durante el
step — ese es el propósito de los gates por eje.

---

## Flujo de calibración completo

```
1. Ajustar wheel PID (inner loop) → step, rise < 30ms, sin overshoot
2. Ajustar VTH (outer loop) → step 4 rad/s:
   python3 csv_analyzer.py Vth.csv -c v_vel[2] -a          # métricas actuales
   python3 pid_tuner.py Vth.csv -c v_vel[2] -m vth_est --current 2.0,6.0,0.0
   # → ajustar gains en drive_task.c, flashear, re-loggear
   python3 csv_analyzer.py Vth_v1.csv Vth_v2.csv --compare  # comparar
3. Ajustar VX/VY → step 1.0 m/s:
   python3 pid_tuner.py Vx.csv -c v_vel[0] -m vx_est --current 0.8,1.6,0.0
4. Validar estimaciones con cámara:
   python3 sensor_compare.py telemetria.csv camara.csv --var vx vy vth
5. Validar en movimiento combinado (vx+vy+vth simultáneo)
```

### Guía rápida de ajuste PID

| Síntoma                     | Acción                        |
|-----------------------------|-------------------------------|
| Respuesta lenta             | Aumentar Kp                   |
| Overshoot alto              | Reducir Kp o Ki               |
| Oscilaciones sostenidas     | Reducir Kp, aumentar Kd       |
| Error en estado estacionario| Aumentar Ki                   |
| Ruido en la salida          | Reducir Kd, aumentar filtro   |
