# Draft: Characterize Vx/Vy plant + Close X/Y loop in engine

**Slug**: characterize-vxvy-close-xy-loop
**Date**: 2026-08-03
**Status**: awaiting-user-answers (3 forks pending)
**Skill**: shared/ulw-plan (CLEAR intent path)

## User request (verbatim intent, Spanish)

"quiero caracterizar el movimiento de mi robot en vx y vy para posteriormente cerrar el
lazo de X e Y en el engine con la fusion de los datos de la camara y la telemetry del
robot. Tengo una duda sobre como deberia de caracterizar mi robot: 2 datos distintos que
puedo adquirir para realizar una planta en vx y vy — (1) live_telemetry (STM32Monitor
1kHz CSV), (2) el engine (creo que es lo mas optimo, la fusion es confiable) generando un
CSV con toda la info necesaria. Leer con detenimiento los archivos de analysis/ (robot),
las pruebas estan en engine/lua (velocity_steps_test.lua), y docs/ si es necesario."

## Exploration findings (2026-08-03)

### The plant chain to close
engine Lua send_velocity → radio (nRF24 30B frame) → robot STM32F767ZI: velocity PID
(200Hz, pid_vx Kp=0.8 Ki=1.6 outMax=1.5, pid_vy same, pid_vth Kp=10 Ki=5) → kinematic →
wheel PID (1kHz Kp=50 Ki=5) → motors → encoders → robot Kalman (vx_est=Kalman1D(enc,acc),
vy_est same) → 32B telemetry @~60Hz → engine radio.rs parse → **shared Tracker EKF
(receiver/tracker.rs, 7 states [x,y,sinθ,cosθ,vx,vy,ω])** → world.set_fused_robot_state →
Lua get_robot_state / charts / logger.

### Fusion already exists and works (user confirmed)
- Single shared `Tracker` (Arc<Mutex>) per robot, fed by BOTH:
  - vision.rs process_robot → track_vision(team,id,x,y,theta,dt) → world.update_robot (writes EKF-filtered fused pose)
  - main.rs run_engine telemetry loop → track_telemetry(team,id,x_est,y_est,theta_est,vx_est,vy_est,omega_est,gyro_z,dt) → world.set_fused_robot_state (writes fused pose+vel)
  - dt computed from last_telemetry_times HashMap, clamped [0.001,0.1], default 0.016
- MeasurementSource: Vision noise {x,y:1e-4, θ:1e-3} (3-state update); Telemetry noise
  {x,y:1e-3, θ:1e-2, vx,vy:1e-2, ω:1e-1} (5-state update). set_delays(0.015,0.010)
  vision/telemetry delay compensation (unit-tested).
- main.rs comment: "Shared EKF tracker — single source of truth for robot state fusion."

### Data sources for characterization (the user's doubt)
1. **STM32Monitor 1kHz CSV** (live_telemetry.py parses 32B packet; example
   analysis/TestescalonVx.csv: time,ax_ms2,v_vel[0],vx_enc,vx_est): robot-internal plant
   (cmd→vx_enc/vx_est) at high fidelity. NO radio latency, NO fusion. Per-axis PID
   disable via stm32monitor native write pid_vx_enable (open-loop option).
2. **Engine logger.rs frame CSV** (automatic, ~60Hz, per robot per frame, header:
   `ElapsedTime,RobotID,Team,Vx_Command,Vy_Command,Angular_Command,Pos_X,Pos_Y,Orientation,Vx_Actual,Vy_Actual`):
   command + FUSED state (position AND velocity) frame-synced. Started via GUI Start
   Recording → EngineCommand::StartRecording → logs/log_{timestamp}.csv (or custom path).
   csv_analyzer.py auto-detects command column (contains "command").
3. **Engine charts-panel CSV export** (gui/panels/charts.rs export_csv_async, header:
   time,x,y,theta,vx,vy,cmd_vx,cmd_vy,cmd_angular,ekf_x,ekf_y,ekf_theta,ekf_vx,ekf_vy,ekf_omega,gyro_z,ball_range):
   pushed on BOTH vision update and telemetry update (~2×/frame, interleaved); "vx" col =
   fused state at vision tick; "ekf_vx" = fused state at telemetry tick (note: main.rs
   TelemetryUpdate carries FUSED values, not raw robot EKF); cmd_* from world
   commanded_velocity (last sent command). Time = charts start_time elapsed.

### Analysis tools (robot/analysis/)
- csv_analyzer.py: step metrics (gain, rise_time, overshoot_pct, settle_2pct/5pct,
  tau_decay, latency_ms, clipped, noisy); --cmd auto-detect by name (v_vel|cmd|ref|setpoint|target|command), --all-steps, --export json.
- pid_tuner.py: closed-loop heuristic deltas (--current Kp,Ki,Kd) OR open-loop FOPDT fit
  (K,L,T) + Ziegler-Nichols/Tyreus-Luyben/IMC rules; --plant K=,L=,T= direct. PID
  convention Kp direct, Ki*error*dt integral, Kd on measurement, sampleTime 5ms (200Hz).
- sensor_compare.py: two CSV cross-comparison (interpolate low-rate onto high-rate;
  rms/mean/std/max error, correlation, delay via cross-correlation).
- live_telemetry.py: real-time monitor of STM32Monitor CSV (reads LATEST csv in /home/sysmic/log).

### Lua test surface (engine/engine/lua/)
- velocity_step_test.lua (current): ROBOT_ID=0, TEAM=0, DURATION=1.0; send_velocity(-1.0,-1.0,target_theta)
  for 1s then 0,0. **NOTE: commands vx=-1 AND vy=-1 SIMULTANEOUSLY (diagonal) — NOT per-axis; too short for multi-step ID.**
- Lua API (api/control.rs + api_globals.lua): send_velocity(id,team,vx,vy,omega),
  move_to, move_direct, plan_path, face_to(id,team,{x,y},kp?,ki?,kd?), kickx, kickz,
  dribbler, test_motor_step(id,motor_idx,voltage); get_robot_state(id,team) → {id,team,
  x,y,vel_x,vel_y,orientation,omega,active} (FUSED values), get_ball_state,
  get_blue/yellow_team_state, get_ref_message.
- motion module (Rust): ONLY bangbang (move_direct/move_to, BangBangControl::new(2.5,5.0))
  + face_to PID. **No X/Y position PID exists yet in engine** — closing X/Y is new work.

### Prior plan context
- robot/.omo/plans/implementation-plan.md (jun 30, "Plan Integral: Bugs + Code Smells +
  Caracterización") covered FIRMWARE-side work: per-axis pid_enable flags (3.1 — DONE,
  pid_vx_enable/pid_vy_enable/pid_vth_enable exist in system_globals), stm32monitor
  characterization flow docs (3.2/5.1 — DONE in docs/characterization-guide.md), bug
  fixes (#1-#30 mostly corrected per docs/bugs-and-improvements.md, audited 11-jul).
  This new request is the NEXT phase: ENGINE-side plant ID + X/Y loop. NOT covered before.

### Known firmware state (docs/bugs-and-improvements.md)
- #19/#20 EKF covariance/sin-cos fixes CORRECTED (11-jul). #8 PID_Init Kd not normalized
  by sampleTime (open). #21 PID integral reset uses cmd≈0 not err≈0 (open).

## Forks — USER DECISIONS (answered 2026-08-03)
1. **Characterization dataset**: ENGINE LOGGER + STM32Monitor cross-check (recommended).
   Primary: engine logger.rs CSV (Recording panel GUI → logs/log_*.csv, header
   ElapsedTime,RobotID,Team,Vx_Command,Vy_Command,Angular_Command,Pos_X,Pos_Y,Orientation,
   Vx_Actual,Vy_Actual, ~60Hz per robot per frame, command + FUSED state). Secondary:
   STM32Monitor 1kHz record (v_vel[0], vx_enc, vx_est) to validate robot dynamics aren't
   aliased at 60Hz and to separate robot dynamics from radio/fusion latency.
   Charts-panel CSV export (charts.rs) available as tertiary cross-check (has vision vs
   ekf split, gyro, ball_range; interleaved pushes).
2. **LAZO ABIERTO PRIMERO (re-tune del PID del robot)**: open-loop characterization of
   the robot velocity plant (pid_vx_enable=0, pid_vy_enable=0 via `stm32monitor native
   write`), FOPDT fit + re-tune pid_vx/pid_vy (IMC/ZN, sampleTime 5ms/200Hz), THEN
   closed-loop verification (pid enabled, rise<100ms OS<10%), THEN engine X/Y design.
3. **X/Y controller home**: PROTOTIPO EN LUA → PROMOVER A RUST (motion module). Lua
   first (get_robot_state fused + send_velocity, fast iteration), then promote to
   motion/controllers/pid PositionPID + Lua binding, re-validate.

## GUI recording mechanics (verified)
- Recording panel (gui/panels/recording.rs): filename text input (default "record.csv"),
  Start → EngineCommand::StartRecording{filename} → logger.rs writes CSV (BufWriter,
  flush every 60 frames) to GIVEN path verbatim (empty → logs/log_{timestamp}.csv);
  Stop → StopRecording. Header: ElapsedTime,RobotID,Team,Vx_Command,Vy_Command,
  Angular_Command,Pos_X,Pos_Y,Orientation,Vx_Actual,Vy_Actual. One row per (team,id)
  per frame + one ball row (-1,-1).
- Charts panel: ⏺ ToggleRecording (charts.rs) accumulates DataSample in RAM (max 60k),
  then export via rfd save dialog (export_csv_async): header time,x,y,theta,vx,vy,
  cmd_vx,cmd_vy,cmd_angular,ekf_x,ekf_y,ekf_theta,ekf_vx,ekf_vy,ekf_omega,gyro_z,ball_range.
- NOTE: logger log_frame captures robots BEFORE this frame's telemetry fusion
  (frame-start snapshot) and command_map AFTER Lua — cmd leads state by ~1 frame
  (16.7ms) + radio latency; fits within FOPDT delay estimation.
- pid_tuner.py CLI confirmed: `python3 pid_tuner.py CSV -c <cmdcol> -m <meascal> --open-loop`
  (FOPDT + ZN/TL/IMC), `--plant K,L,T`, `--export tune.json`; closed-loop mode with
  `--current Kp,Ki,Kd`. csv_analyzer auto-detects command col by name (Vx_Command matches).

## Adopted defaults (not asked)
- Controller form: cascade PID (engine position PID → vx/vy cmd → robot velocity loop →
  integrator → fused position). Tune via pid_tuner.py IMC/ZN from identified plant
  (L,T,K). Feedforward v_cmd = kff*pos_error_velocity optional.
- velocity_steps_test.lua must be REWRITTEN to per-axis step trains (vx-only, vy-only,
  symmetric ±steps, WAIT per docs: open-loop ±0.3..±1.2 m/s WAIT=50 frames, closed-loop
  ±0.5..±1.5 WAIT=70; docs Fase-3 targets rise<100ms OS<10%).
- Engine X/Y loop runs at ~60Hz (frame rate); gains must be tuned for 16.67ms sample time.

## Plan shape (5 waves, Architecture-scale)
- **Wave 0 — Harness**: rewrite velocity_steps_test.lua → parameterized per-axis step
  train (vx-only, vy-only, symmetric ±amps, configurable WAIT/duration, theta hold via
  get_robot_state().orientation on first frame, progress prints). Verify analysis tools
  accept engine logger CSV (Vx_Command auto-detect OK; add helper if needed).
- **Wave 1 — Open-loop robot plant ID + re-tune robot velocity PID**: stm32monitor
  disable pid_vx_enable/pid_vy_enable (keep pid_vy ON when characterizing vx so robot
  runs straight; disable per-axis); Lua open-loop step trains (±0.3..±1.2 m/s, WAIT=50);
  SIMULTANEOUS STM32Monitor 1kHz record + engine logger CSV; FOPDT fit (pid_tuner.py
  --open-loop) on 1kHz (primary) + engine CSV (cross-check); write new pid_vx/pid_vy
  Kp/Ki via stm32monitor native write; re-enable.
- **Wave 2 — Closed-loop verification + fusion validation**: pid enabled; step trains
  ±0.5..±1.5 m/s WAIT=70; engine logger CSV primary; accept rise<100ms OS<10% per axis;
  sensor_compare.py 1kHz vx_est vs engine Vx_Actual/ekf_vx → quantify radio/fusion delay
  and bias; iterate gains if fail.
- **Wave 3 — Position-loop plant + X/Y gains**: from closed-loop logger CSVs build
  position plant vx_cmd → x_fused (velocity plant + integrator, Ts=16.67ms); compute
  cascade position PID gains (IMC/pole placement @60Hz) + optional velocity feedforward.
- **Wave 4 — Lua X/Y prototype**: xy_loop_test.lua (position PID: vx_cmd=Kp*ex+Ki*∫ex
  (+kff*v_ref), theta hold, send_velocity); position step tests (0→0.5m, 0→-0.5m per
  axis, square); validate with engine logger CSV + csv_analyzer (rise/OS/settle); tune.
- **Wave 5 — Promote to Rust + docs**: PositionPID in motion/controllers/pid/, motion
  API + Lua binding, re-run validation; update docs/characterization-guide.md (engine
  CSV columns, X/Y loop procedure), analysis README, record new bugs/learnings.

## Approval gate
- status: **APPROVED + EXECUTION STARTED** (user: "SI Y COMIENZA", m0034)
- Plan written: .omo/plans/characterize-vxvy-close-xy-loop.md (full todos W0-W5, T1-T9).
- Mandatory Metis gap analysis DONE (lively-jade-lynx): G1-G11 integrated into plan todos.
- Corrected stale draft claims vs verified source: per-axis pid_enable flags did NOT exist in
  current firmware (old plan 3.1 never landed in code) — plan T3 creates them; current pid_vx
  gains are Kp=2.5 Ki=9.2 (NOT 0.8/1.6 as docs); rate limiters ramp vx/vy at 1.5 m/s².
- Dual-Momus high-accuracy review: optional (CLEAR path) — offer at delivery.
