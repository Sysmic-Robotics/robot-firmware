# characterize-vxvy-close-xy-loop - Work Plan

## TL;DR (For humans)
<!-- Fill this LAST, after the detailed plan below is written, so it summarizes the REAL plan. -->
<!-- Plain English for a non-engineer: NO file paths, NO todo numbers, NO wave/agent/tool names. -->

**What you'll get:** Your robot's vx/vy velocity response characterized (plant model + re-tuned velocity controllers), then a working X/Y position controller in the engine (Lua prototype first, then promoted to the Rust engine core) that closes the loop using the camera+telemetry fusion you already trust.

**Why this approach:** Open-loop characterization first — measuring the raw robot response with its velocity controllers off — so we re-tune the on-board controllers from real data instead of guessing (closed-loop data would bake the current gains into the model). A tiny firmware addition (per-axis PID on/off switches) makes this clean by letting the robot hold its heading while we excite one axis at a time. The position loop is prototyped in Lua for fast iteration, then ported to Rust with unit tests.

**What it will NOT do:** No changes to the radio protocol, the fusion EKF internals, the wheel-level PID, the camera/vision pipeline, or the GUI. No new dependencies. Not touching the θ/rotation controller.

**Effort:** Medium-Large
**Risk:** Medium - depends on hardware-in-the-loop runs (robot on field + stm32monitor) for final tuning; code and analysis are fully automated around those runs
**Decisions to sanity-check:** (1) firmware per-axis PID gates re-added (small code change + one reflash); (2) X/Y gains computed from plant identified on existing data first, then refined with fresh runs

Your next move: execution already approved ("SI Y COMIENZA"). Full execution detail follows below.

---

> TL;DR (machine): Medium-Large effort, Medium risk. W0 harness+tools, W0.5 firmware gates, W1 open-loop ID+retune, W2 closed-loop verify+fusion check, W3 position plant+gains, W4 Lua X/Y prototype, W5 Rust promotion+docs. Agent-executable analysis around operator data runs.

## Scope
### Must have
- **W0.1** Parameterized per-axis velocity step-test harness (rewrite `engine/engine/lua/velocity_steps_test.lua`): one clean step per run (axis, amplitude ±, duration), θ-hold via fused orientation, `[step]` progress prints; config block at top of file (ROBOT_ID, TEAM, AXIS, AMP, DURATION_FRAMES, THETA_HOLD).
- **W0.2** Analysis-tool verification: `csv_analyzer.py` + `pid_tuner.py --open-loop` proven on a synthetic engine-logger-style CSV fixture (known FOPDT K,L,T recovered within tolerance) so 60Hz command/fused-state CSVs are usable.
- **W0.5** Firmware per-axis PID passthrough gates `pid_vx_enable`/`pid_vy_enable`/`pid_vth_enable` (uint8, default 1) in `drive_task.c` + `system_globals.c/.h` — restores old-plan 3.1, enables clean open-loop ID while vy/θ hold stay active (Metis G7). Compiles clean.
- **W1** Open-loop vx & vy plant ID: operator runbook + one-command scripts (exact `stm32monitor native write` disable, Lua test config, STM32Monitor 1kHz record + engine logger record commands, data naming convention); FOPDT (K,L,T) fit on 1kHz primary + engine-logger cross-check; G3 acceptance (reject step if |Δvy|>0.1 m/s or |Δθ|>0.05 rad, average ±directions); new pid_vx/pid_vy gains via IMC/ZN @200Hz (sampleTime 5ms). Pipeline validated on existing `analysis/TestescalonVx.csv` as data dry-run.
- **W2** Closed-loop verification + fusion validation: pid enabled, step trains ±0.5..±1.5 m/s; acceptance rise<100ms, OS<10%, settle + steady-state-error on 1kHz; `sensor_compare.py` 1kHz vx_est vs engine Vx_Actual/ekf_vx → radio/fusion delay + bias report.
- **W3** Position plant `vx_cmd → x_fused` (velocity plant + integrator, Ts=16.67ms) + cascade X/Y position PID gains @60Hz (IMC/pole placement) + optional velocity feedforward; validated in a small Python simulation with the identified plant.
- **W4** Lua X/Y position PID prototype `xy_loop_test.lua`: field-error → robot-frame rotation via fused θ (pure-Lua math unit-tested), Kp/Ki/anti-windup/output clamp ±2.0 m/s, sends `send_velocity(id,team,vx_local,vy_local,θ_target=fused_θ wrapped [-π,π])`; position step config (setpoints list).
- **W5** Rust promotion: `PositionPID` in `motion/controllers/pid/position_pid.rs`, motion API + Lua binding (`api/control.rs`, `api_globals.lua`), Rust unit tests (rotation, PID math, θ-wrap, stale-state guard), re-validation via engine (grSim sim or robot).

### Must NOT have (guardrails, anti-slop, scope boundaries)
- No changes to radio packet format / `packet_serializer.rs` / 30B frame layout.
- No changes to the fusion EKF internals (`receiver/tracker.rs`, vision/telemetry noise, delays) — used as-is.
- No re-tune of wheel PID (Kp=50 Ki=5 @1kHz), no motor/DAC changes.
- No work on the θ/ω loop beyond keeping pid_theta intact; no new angular controller.
- No camera/vision pipeline changes (`receiver/vision.rs` read-only).
- No GUI changes (`gui/` read-only except reading CSV conventions).
- No new dependencies (analysis stays matplotlib+numpy; engine stays iced/mlua/serde).
- No bug #8/#21 firmware deep-fixes in this plan (current pid Kd=0 makes #8 moot; integral reset already err-based `|cmd-meas|<0.01` — verified in code, docs stale). If retune data shows Kd needed, flag as follow-up, do not fix here.
- No Lua→Rust port of existing bangbang/face_to.
- No multi-robot coordination, skills, or AI changes.

## Verification strategy
> Zero human intervention where possible - all analysis is agent-executed; hardware runs follow an exact runbook.
- Test decision: tests-after for Rust PositionPID (`cargo test` unit tests: rotation, PID accumulation, θ-wrap, stale guard); Lua rotation math unit-tested via standalone `lua5.4`/`luac` syntax+assert where available; analysis scripts verified against synthetic CSV fixtures with known ground truth.
- Evidence: `.omo/evidence/task-<N>-characterize-vxvy-close-xy-loop.<ext>` (json/png/txt) for every todo.
- Agent-executable everywhere EXCEPT the physical runs (robot on field + stm32monitor + engine recording) — those follow `robot/analysis/open_loop_recipe.md` runbook; post-processing is automated.
- Data dry-run on existing `analysis/TestescalonVx.csv` (real 1kHz step data, jul 8) exercises the whole W1 pipeline before any operator time is spent.

## Execution strategy
### Parallel execution waves
- **Wave 0 — Harness + tools (agent, no hardware, parallel):** T1 Lua step harness; T2 analysis-tool verification on synthetic fixture + W1 pipeline dry-run on TestescalonVx.csv.
- **Wave 0.5 — Firmware gates (agent writes code; operator flashes):** T3 per-axis PID gates.
- **Wave 1 — Open-loop ID (operator runs, agent analyzes):** T4 runbook+scripts; T5 fit+retune (blocked on operator data; pipeline itself pre-built and synthetic-tested in T2).
- **Wave 2 — Closed-loop verify + fusion check (operator runs, agent analyzes):** T6.
- **Wave 3 — Position plant + X/Y gains (agent, from T5 plant):** T7.
- **Wave 4 — Lua X/Y prototype (agent):** T8.
- **Wave 5 — Rust promotion + docs (agent):** T9.

### Dependency matrix
| Todo | Depends on | Blocks | Can parallelize with |
| --- | --- | --- | --- |
| 1 (Lua harness) | — | 8 (conventions) | 2, 3 |
| 2 (tools verify + W1 dry-run) | — | 5 (pipeline), 7 (plant) | 1, 3 |
| 3 (firmware gates) | — | 5 (open-loop run) | 1, 2 |
| 4 (runbook+scripts) | 2 (pipeline) | 5 (operator run) | 3 |
| 5 (fit+retune) | 2, 3, 4, operator data | 6, 7 | — |
| 6 (closed-loop verify + fusion) | 5 | 7 | — |
| 7 (position plant + gains) | 5 | 8 | 6 (can start from T5 plant) |
| 8 (Lua X/Y prototype) | 1, 7 | 9 | — |
| 9 (Rust promotion + docs) | 8 | — | — |

## Todos
> Implementation + Test = ONE todo. Never separate.
<!-- APPEND TASK BATCHES BELOW THIS LINE WITH edit/apply_patch - never rewrite the headers above. -->
- [x] 1. Rewrite engine/engine/lua/velocity_steps_test.lua into a parameterized per-axis velocity step harness
  What to do / Must NOT do: Rewrite the Lua test to produce ONE clean velocity step per run. Top-of-file config block: ROBOT_ID=0, TEAM=0, AXIS="vx"|"vy", AMP (+/- m/s, e.g. 1.0), DURATION_FRAMES (~60 frames = 1s at 60Hz), THETA_HOLD=true. In process(): on first call capture rs=get_robot_state(ROBOT_ID,TEAM), target_theta=rs.orientation (wrap [-π,π]); while elapsed frames < DURATION send send_velocity(ROBOT_ID,TEAM, AMP,0,target_theta) for vx or (0,AMP,target_theta) for vy; after duration send (0,0,target_theta) and stop (print [test] done). Print [step] <axis> <amp> <frame> markers each frame for console/telemetry alignment. MUST NOT: diagonal vx+vy simultaneously (current script bug), any theta changes mid-step, any motion module calls, any changes to other lua files, any new engine API usage beyond send_velocity + get_robot_state.
  Parallelization: Wave 0 | Blocked by: — | Blocks: 8
  References (executor has NO interview context - be exhaustive): engine/engine/lua/velocity_step_test.lua (current 30-line version, the file to REWRITE); engine/engine/lua/characterize.lua (existing multi-step open-loop style example); engine/engine/lua/api_globals.lua + engine/engine/src/lua_interface/api/control.rs (send_velocity(id,team,vx,vy,omega) signature — NOTE omega param is the absolute field-frame θ-target in radians, NOT ω; must wrap to [-π,π]); engine/engine/src/lua_interface/api/state.rs (get_robot_state → {id,team,x,y,vel_x,vel_y,orientation,omega,active}); docs/characterization-guide.md Fase 3 (open-loop ±0.3..±1.2 m/s, WAIT=50 frames). Engine loop is ~60FPS, process() called once per frame.
  Acceptance criteria (agent-executable): `lua5.4 -e "assert(loadfile('engine/engine/lua/velocity_step_test.lua'))"` or `luac -p engine/engine/lua/velocity_step_test.lua` exits 0 (if lua/luac absent, fall back to a syntax check via engine docs and report); file rewritten with config block + per-axis single-step logic; grep confirms no simultaneous vx&vy nonzero.
  QA scenarios (name the exact tool + invocation): happy: `lua5.4 -e "assert(loadfile('engine/engine/lua/velocity_step_test.lua')); print('ok')"` → prints ok, Evidence .omo/evidence/task-1-characterize-vxvy-close-xy-loop.txt. failure: script has syntax error → loadfile returns nil, fix and re-run. Also verify a config with AXIS="vx" AMP=1.0 only ever emits send_velocity(0,0,theta,vy=0) calls per the branch logic by static review.
  Commit: N | (not a git repo — record manual backup of original to .omo/backups/)

- [x] 2. Verify analysis tools on engine-logger-style CSV fixture + dry-run W1 pipeline on existing TestescalonVx.csv
  What to do / Must NOT do: (a) Create a synthetic CSV fixture in robot/analysis/tests/fixture_engine_vx_step.csv mimicking the engine logger header `ElapsedTime,RobotID,Team,Vx_Command,Vy_Command,Angular_Command,Pos_X,Pos_Y,Orientation,Vx_Actual,Vy_Actual` at 60Hz (16.667ms), containing a known first-order+delay response v(t)=K*A*(1-exp(-(t-L)/T)) with known K,L,T (e.g. K=0.95, L=0.03, T=0.12, A=1.0) plus small noise; run `python3 robot/analysis/pid_tuner.py <fixture> -c Vx_Command -m Vx_Actual --open-loop` and confirm recovered K,L,T within ±15%; run `python3 robot/analysis/csv_analyzer.py <fixture> --cmd Vx_Command --export <json>` and confirm step metrics non-NaN. (b) Dry-run the real pipeline on existing robot/analysis/TestescalonVx.csv (columns time,ax_ms2,v_vel[0],vx_enc,vx_est, 1kHz real step data): `pid_tuner.py TestescalonVx.csv -c v_vel[0] -m vx_est --open-loop` → record K,L,T + ZN/IMC gains to .omo/evidence; `csv_analyzer.py TestescalonVx.csv -c v_vel[0] -m vx_est --all-steps --export`. MUST NOT: modify the analysis tools' public CLI; write test CSVs outside robot/analysis/tests/; delete or alter TestescalonVx.csv. NOTE the firmware rate limiter (1.5 m/s² → a 1.0 m/s step ramps over 0.67s) makes raw FOPDT L,T biased — document ramp-aware interpretation (effective input is a ramp; K from steady state is reliable; L,T are upper bounds) in the evidence.
  Parallelization: Wave 0 | Blocked by: — | Blocks: 5, 7
  References (executor has NO interview context - be exhaustive): robot/analysis/pid_tuner.py (CLI: `python3 pid_tuner.py CSV -c <cmdcol> -m <meascal> --open-loop`, FOPDT K,L,T + ZN/TL/IMC, `--plant K,L,T`, `--export tune.json`; PID convention Kp direct, Ki*error*dt, sampleTime 5ms); robot/analysis/csv_analyzer.py (step metrics gain/rise_time/overshoot_pct/settle_2pct/tau_decay/latency_ms, --cmd auto-detect by name, --all-steps, --export json); robot/analysis/TestescalonVx.csv (real data, 7,118 rows, time,ax_ms2,v_vel[0],vx_enc,vx_est); robot/analysis/stm32_monitor_log.csv + telemetry_decoded.csv for column-reference only; engine logger header (gui/panels/recording.rs + logger.rs): ElapsedTime,RobotID,Team,Vx_Command,Vy_Command,Angular_Command,Pos_X,Pos_Y,Orientation,Vx_Actual,Vy_Actual; drive_task.c rate limiter facts (b3 context): ROBOT_MAX_LINEAR_ACC=1.5 m/s².
  Acceptance criteria (agent-executable): fixture test run above recovers K,L,T within ±15% (evidence json); TestescalonVx.csv dry-run produces a tune.json + step metrics json in .omo/evidence; both commands exit 0.
  QA scenarios: happy: fixture FOPDT recovery within tolerance → Evidence .omo/evidence/task-2-characterize-vxvy-close-xy-loop.json. failure: recovery out of tolerance → check fixture noise/sample-rate, fix fixture, re-run; report K,L,T for TestescalonVx with ramp caveat.
  Commit: N | (manual backup note)

- [x] 3. Add per-axis PID passthrough gates pid_vx_enable/pid_vy_enable/pid_vth_enable to robot firmware (drive_task.c + system_globals.c/.h)
  What to do / Must NOT do: In robot/Core/Src/system_globals.c add three uint8 globals pid_vx_enable=1, pid_vy_enable=1, pid_vth_enable=1; declare extern in robot/Core/Inc/system_globals.h (next to pid_vx/pid_vy/pid_vth/pid_theta handlers ~line 91-94). In robot/Core/Src/drive_task.c 200Hz loop (robot_pid_enable block ~line 273): wrap each per-axis controller so that when the enable flag is 1 the existing PID path runs, and when 0 the raw v_vel[i] command passes through: `if (pid_vx_enable){ PID_CloseLoop(&pid_vx,...); vx_cmd=pid_vx.output; } else { vx_cmd=v_vel[0]; }` (same pattern vy, and for vth gate the pid_theta block: enabled → existing pid_theta path; disabled → vth_cmd=v_vel[2]). Keep master robot_pid_enable watchdog logic untouched. Defaults MUST be 1 (enabled) so normal behavior is identical. MUST NOT: change any PID gains, sample times, rate limiters, kinematic matrix, radio parsing; do not rename existing globals; do not touch Tools/PID.c/h; do not alter the watchdog.
  Parallelization: Wave 0.5 | Blocked by: — | Blocks: 5
  References (executor has NO interview context - be exhaustive): robot/Core/Src/drive_task.c lines ~250-360 (200Hz loop: rate limiters 256-266 & 291-296, robot_pid_enable block line 273, vx/vy PID then pid_theta, kinematic mix 301-303, throttle limiter 353); robot/Core/Src/system_globals.c lines 33-37 (existing PID handler globals + robot_pid_enable); robot/Core/Inc/system_globals.h lines 15-18 (ROBOT_MAX_LINEAR_ACC/ANGULAR_ACC) and 91-94 (extern PID handlers); robot/Tools/PID.h lines 66-93 (PID_Params_t/PID_Handler_t structs — read-only); prior plan robot/.omo/plans/implementation-plan.md section 3.1 (intended per-axis gates); docs/characterization-guide.md (claims gates exist — stale; this todo CREATES them). Build: robot/Core/ Makefile or STM32CubeIDE project (check robot/Core/ for build entry, e.g. `make` in Core/ or .ioc project robot-firmware.ioc).
  Acceptance criteria (agent-executable): firmware source compiles cleanly (run the project's build command if toolchain present, e.g. `make -C robot/Core` — if ARM toolchain absent, report and mark operator-build); grep confirms the three globals defined default 1, extern declared, and each wrapped in drive_task.c; normal-path behavior unchanged when all flags=1 (code review).
  QA scenarios: happy: compile exit 0 + grep assertions pass → Evidence .omo/evidence/task-3-characterize-vxvy-close-xy-loop.txt. failure: compile errors → fix, re-run; stm32monitor read/write of pid_vx_enable is OPERATOR step (runbook T4), flag as pending.
  Commit: N | (manual backup of drive_task.c/system_globals.c/h)

- [x] 4. Write operator runbook robot/analysis/open_loop_recipe.md + one-command helper scripts for open-loop vx/vy characterization
  What to do / Must NOT do: Produce a runbook with EXACT commands: (a) stm32monitor disable: `stm32monitor native write pid_vx_enable 0` (keep pid_vy_enable=1, pid_vth_enable=1 so θ-hold + vy-hold stay active during vx steps; symmetric for vy runs); verify read-back via `stm32monitor live pid_vx_enable`; (b) record 1kHz: `stm32monitor record --vars time,ax_ms2,v_vel[0],v_vel[1],vx_enc,vx_est,vy_enc,vy_est,theta_est --output analysis/data_openloop_vx_p1.0.csv` (adjust var names to what stm32monitor exposes — verify against existing analysis/TestescalonVx.csv columns); (c) engine logger record: GUI Recording panel filename per convention `record_vx_p1.0.csv` (Start before test run, Stop after); (d) run Lua test with velocity_step_test.lua config AXIS=vx AMP=+1.0 DURATION=60; (e) step matrix table: vx ±0.3, ±0.6, ±1.0, ±1.2; vy ±0.3, ±0.6, ±1.0, ±1.2 (open-loop, WAIT ~50 frames rest between), file-naming convention `<kind>_<axis>_<sign>_<amp>.csv`; (f) re-enable: `stm32monitor native write pid_vx_enable 1`. Include G3 acceptance reminder (reject step if |Δvy|>0.1 or |Δθ|>0.05; repeat both directions, average). Also include stm32monitor struct-write syntax verification step (Metis G4): try `stm32monitor native write pid_vx.Kp 2.5` + read-back on bench BEFORE field runs, record whether syntax works. MUST NOT: fabricate stm32monitor var names not evidenced; recommend flashing during field runs (flash W0.5 first); include engine changes.
  Parallelization: Wave 1 | Blocked by: 2 (pipeline proven) | Blocks: 5 (operator run)
  References (executor has NO interview context - be exhaustive): robot/analysis/TestescalonVx.csv columns (evidence of stm32monitor var names); docs/characterization-guide.md (stm32monitor usage: `stm32monitor live <vars>`, `stm32monitor native write <var> <val>`, `stm32monitor record --vars a,b --output f.csv`; Fase 3 step matrices ±0.3..±1.2 open-loop WAIT=50); robot/analysis/live_telemetry.py (reads latest CSV in /home/sysmic/log — the STM32Monitor output dir); engine Recording panel mechanics (gui/panels/recording.rs: filename input default record.csv → logger writes path verbatim, header ElapsedTime,RobotID,Team,Vx_Command,Vy_Command,Angular_Command,Pos_X,Pos_Y,Orientation,Vx_Actual,Vy_Actual); G3/G4 from Metis (b3 context).
  Acceptance criteria (agent-executable): robot/analysis/open_loop_recipe.md exists with every command copy-pasteable; helper script (e.g. robot/analysis/run_openloop_vx.sh) wraps disable→record→enable for one step; bash -n passes.
  QA scenarios: happy: `bash -n robot/analysis/run_openloop_vx.sh` + runbook section for every step matrix entry present → Evidence .omo/evidence/task-4-characterize-vxvy-close-xy-loop.txt. failure: missing var names → mark as TODO-verify in runbook, do not guess.
  Commit: N

- [x] 5. Fit FOPDT plant + compute new pid_vx/pid_vy gains from operator open-loop data (BLOCKED on operator run; agent pipeline + report template ready in T2)
  What to do / Must NOT do: When operator data lands (analysis/data_openloop_vx_*.csv 1kHz + engine record_vx_*.csv 60Hz): run `pid_tuner.py <1kHz csv> -c v_vel[0] -m vx_est --open-loop` per step; apply G3 filter (drop steps where |Δvy|>0.1 m/s or |Δθ|>0.05 rad over step); average K,L,T across accepted steps (both directions); cross-check steady-state K against engine logger CSV (`csv_analyzer.py record_vx_p1.0.csv -c Vx_Command -m Vx_Actual --export`); produce recommended pid_vx Kp/Ki via IMC (λ≥3-5×T) and ZN @200Hz (sampleTime 5ms), write to .omo/evidence/task-5-characterize-vxvy-close-xy-loop.json + a short report (gains, plant summary, ramp caveat, residual θ/vy disturbance). Repeat for vy. If 1kHz data unavailable, fit on engine 60Hz with explicit sample-time caveat (only K reliable). MUST NOT: apply gains to firmware (operator writes via stm32monitor per runbook), silently average across rejected steps, change analysis tools.
  Parallelization: Wave 1 | Blocked by: 2, 3, 4 + operator data | Blocks: 6, 7
  References: same as T2 + runbook robot/analysis/open_loop_recipe.md; PID convention docs/characterization-guide.md (pid_vx Kp direct Ki*err*dt sampleTime 5ms outputMax 1.5); current gains Kp=2.5 Ki=9.2 (verified in drive_task.c, differs from docs 0.8/1.6).
  Acceptance criteria (agent-executable): evidence json + report file exist with per-step table, accepted set, averaged plant, recommended gains for both axes; commands exit 0.
  QA scenarios: happy: report generated → evidence json. failure: no operator data yet → document BLOCKED state with the exact command sequence the operator must run; pipeline remains tested via T2 dry-run.
  Commit: N

- [ ] 6. Closed-loop verification + fusion validation (operator runs closed-loop steps; agent analyzes)
  > **Estado (27-Ago-2026)**: PARCIAL — root cause identificado en la evidencia task-6. El análisis de `VxMultiTestCL.csv` reveló un **limit cycle por integral-reset**: `drive_task.c:283` (`if (fabsf(vx_cmd - vx_meas) < 0.01f) pid_vx.integral = 0.0f;`) borra el integral en cuanto el robot se acerca al setpoint (<0.01 m/s), colapsando la salida → el loop se reconstruye → overshoot → repite. Fix propuesto: `if (fabsf(vx_cmd) < 0.01f && fabsf(vx_meas) < 0.01f) pid_vx.integral = 0.0f;` (preserva el guard de ruido en idle, nunca borra el integral con setpoint no-cero). **PENDIENTE**: el usuario debe aplicar el fix en `drive_task.c:283` y `:294`, reflashear, y re-ejecutar `velocity_multistep_test.lua` para validar que `ekf_vx` sigue 0.3/0.6/1.0. La identificación de planta VY también sigue pendiente (necesita `VyTest.csv`).
  What to do / Must NOT do: Runbook addendum: re-enable all pid flags (default 1), closed-loop step trains ±0.5..±1.5 m/s (WAIT=70 frames) per docs Fase 3, engine logger + 1kHz STM32Monitor both recording. Agent analysis: per-step `csv_analyzer.py` on 1kHz (rise_time, overshoot_pct, settle_2pct, steady-state error) + engine CSV; acceptance rise<100ms OS<10%; if fail, compute gain correction from step metrics and re-issue gains (operator re-write). Fusion validation: `sensor_compare.py <1kHz vx_est csv> <engine record csv> --var vx` → rms/bias/correlation/delay between robot vx_est and engine Vx_Actual and ekf_vx; report radio/fusion latency + bias; classify acceptable (<50ms, |bias|<0.05 m/s) or recommend tracker delay/noise adjustments as follow-up (OUT of scope to change). MUST NOT: change tracker/vision code; edit engine; apply gains silently.
  Parallelization: Wave 2 | Blocked by: 5 | Blocks: 7
  References: robot/analysis/sensor_compare.py CLI (two CSVs, --var, interpolate + cross-correlation delay); robot/analysis/csv_analyzer.py; docs/characterization-guide.md Fase 3 closed-loop targets (rise<100ms OS<10%, WAIT=70, ±0.5..±1.5); engine logger + charts CSV headers; tracker MeasurementSource noise/delays (b3 context, read-only reference).
  Acceptance criteria (agent-executable): evidence json with per-step metrics table, pass/fail vs rise<100ms OS<10%, fusion delay/bias report; commands exit 0.
  QA scenarios: happy: all steps pass + fusion report → evidence. failure: steps fail acceptance → output corrected gains + rationale, mark re-verify pending operator.
  Commit: N

- [x] 7. Build position plant vx_cmd→x_fused + compute cascade X/Y position PID gains @60Hz + sim validation
  What to do / Must NOT do: From the T5 closed-loop velocity plant (or T2 dry-run plant if real data pending): position plant = velocity plant + discrete integrator at Ts=16.67ms. Write robot/analysis/position_loop_design.py: takes (K,L,T or closed-loop velocity response params) → computes cascade position PID gains (IMC/pole placement, sample-time 16.67ms, robot-frame output, output clamp ±2.0 m/s) + optional velocity feedforward kff; runs a small discrete-time simulation (velocity plant model + position PID + ideal θ-hold) for step references (0→0.5m, 0→-0.5m per axis) and reports predicted rise/OS/settle; exports gains to .omo/evidence/task-7-characterize-vxvy-close-xy-loop.json. MUST NOT: change engine code; produce gains for the wrong loop rate (position PID is 60Hz engine-side, NOT 200Hz); skip the rotation-to-robot-frame note (gains act on field-frame error rotated by fused θ into robot-frame vx/vy commands).
  Parallelization: Wave 3 | Blocked by: 5 (plant) | Blocks: 8
  References: robot/analysis/pid_tuner.py FOPDT/tuning internals (pattern to follow); engine loop ~60FPS (16.67ms) facts (b3 context); docs/characterization-guide.md cascade structure; motion module location engine/engine/src/motion/ (read-only for this todo).
  Acceptance criteria (agent-executable): position_loop_design.py runs with default params → prints gains + sim metrics; evidence json + PNG (step response plot) saved; exit 0.
  QA scenarios: happy: sim shows rise<0.6s OS<15% with computed gains → evidence. failure: unstable/divergent sim → tighten gains (smaller Kp), re-run, report.
  Commit: N

- [x] 8. Lua X/Y position PID prototype xy_loop_test.lua with pure-Lua rotation unit test
  What to do / Must NOT do: Write engine/engine/lua/xy_loop_test.lua: config block (ROBOT_ID, TEAM, TARGETS list of {x,y} field-frame setpoints, DURATION per target, KP, KI, KFF optional, OUTPUT_MAX=2.0); on each process(): rs=get_robot_state(ROBOT_ID,TEAM); if rs.active and fresh (reject >2×period stale per G6, timestamp via engine frame count); ex=tx-rs.x, ey=ty-rs.y; rotate to robot frame: vx_local = cos(θ)*ex + sin(θ)*ey; vy_local = -sin(θ)*ex + cos(θ)*ey (θ=rs.orientation); integral accumulation with anti-windup clamp ±1.0; cmd_vx = clamp(KP*vx_local + KI*int_vx, -OUTPUT_MAX, OUTPUT_MAX) (+KFF*0 default); send_velocity(ROBOT_ID,TEAM, cmd_vx, cmd_vy, θ_target=wrap(θ)); θ_target MUST be rs.orientation wrapped to [-π,π] (protocol clamps ±3.13 rad). Also write engine/engine/lua/tests/test_xy_rotation.lua: asserts the rotation function maps field-frame error (1,0) with θ=90° → local (0,-1) etc. (pure math, runnable with standalone lua). MUST NOT: use motion()/move_to/move_direct; send ω instead of θ-target; modify existing skills or api_globals; commit gain values that differ from T7 evidence.
  Parallelization: Wave 4 | Blocked by: 1, 7 | Blocks: 9
  References: engine/engine/lua/api_globals.lua + control.rs (send_velocity(id,team,vx,vy,omega) — omega = absolute θ-target radians); state.rs get_robot_state fields; protocol θ semantics (b3: packet theta ×163 ±3.13 rad absolute field-frame angle; pid_theta on robot holds it); T7 gains json; robot frame rotation convention: body-frame vx/vy = R(-θ)*field error.
  Acceptance criteria (agent-executable): `lua5.4 tests/test_xy_rotation.lua` (or luac -p + manual eval) passes all rotation asserts; xy_loop_test.lua loads clean; grep confirms θ_target=wrap(rs.orientation) and no ω use.
  QA scenarios: happy: rotation test prints ok → Evidence .omo/evidence/task-8-characterize-vxvy-close-xy-loop.txt. failure: assertion mismatch → fix math, re-run. Field validation (engine+grSim or robot) is operator/later — noted pending.
  Commit: N

- [x] 9. Promote X/Y position PID to Rust (motion/controllers/pid/position_pid.rs) + Lua binding + unit tests + docs update
  What to do / Must NOT do: Add PositionPID struct in engine/engine/src/motion/controllers/pid/position_pid.rs (fields kp,ki,kff,output_max,integral,integral_max,last_fused_state,last_seen_frame; methods: reset(), set_target(x,y), step(&RobotState)->(vx_local,vy_local,theta_target) implementing field-error→robot-frame rotation via fused θ, integral+anti-windup, stale guard rejecting state older than 2×period, θ-target wrap to [-π,π]); export via motion/mod.rs; add Lua binding in lua_interface/api/control.rs (e.g. set_position_target(id,team,x,y,kp?,ki?) + process internally or per-call step — follow existing binding style) + register in api_globals.lua; write cargo unit tests (rotation mapping, integral accumulation/anti-windup, θ-wrap, stale rejection) — `cargo test` in engine/engine passes; re-run validation via the T8 Lua flow calling the Rust binding. Update docs/characterization-guide.md (engine CSV columns + X/Y loop procedure + new Lua/Rust API) and robot/analysis/README.md (new tools/scripts). MUST NOT: touch existing bangbang/face_to/PID.rs; break the 30B radio protocol; change tracker/vision; leave `as any`/unwrap panics; add dependencies.
  Parallelization: Wave 5 | Blocked by: 8 | Blocks: —
  References: engine/engine/src/motion/controllers/pid/pid.rs (existing PID style to mirror); engine/engine/src/motion/mod.rs (export wiring); engine/engine/src/lua_interface/api/control.rs + api_globals.lua (binding pattern: register function, read args, call engine fn, return values); engine/engine/src/world.rs RobotState struct fields (x,y,orientation,vel_x,vel_y,active,last_update); receiver/tracker.rs (stale/delay semantics G6); docs/characterization-guide.md + robot/analysis/README.md (docs to update).
  Acceptance criteria (agent-executable): `cargo test` in /home/sysmic/Desktop/SysmicProyect/engine/engine passes including new PositionPID tests (exit 0, tests run); `cargo build` exit 0; docs updated.
  QA scenarios: happy: cargo test all green → Evidence .omo/evidence/task-9-characterize-vxvy-close-xy-loop.txt. failure: test failure → fix, re-run; report engine build status.
  Commit: N | (manual backup; suggest user init git if desired)

## Final verification wave
> Runs in parallel after ALL todos. ALL must APPROVE. Surface results and wait for the user's explicit okay before declaring complete.
- [ ] F1. Plan compliance audit
- [ ] F2. Code quality review
- [ ] F3. Real manual QA
- [ ] F4. Scope fidelity

## Commit strategy
- **NOT a git repo** (robot/ and SysmicProyect/ both unversioned). No commits.
- Every todo touching existing files first backs up the original to `.omo/backups/` (manual `cp`), recorded in the todo evidence.
- Recommend (out of scope, optional): `git init` at SysmicProyect root before the next hardware round so W1/W2 data runs can be versioned.

## Success criteria
- Engine `velocity_step_test.lua` produces clean per-axis velocity steps; analysis tools verified on synthetic + real (TestescalonVx) data.
- Firmware has working per-axis PID gates (pid_vx_enable/pid_vy_enable/pid_vth_enable, default 1), compiles, operator-flashable.
- Open-loop vx & vy plant identified (K,L,T per axis, ramp-caveat documented) on 1kHz data with engine-logger cross-check; new pid_vx/pid_vy gains produced via IMC/ZN @200Hz.
- Closed-loop verification meets rise<100ms OS<10% per axis (or explicit documented gap + corrected gains); fusion delay/bias quantified via sensor_compare.
- Position plant + X/Y cascade gains computed and sim-validated @60Hz; Lua X/Y prototype passes rotation unit tests; Rust PositionPID promoted with passing cargo tests + Lua binding; docs updated.
- All evidence in `.omo/evidence/task-*-characterize-vxvy-close-xy-loop.*`.
