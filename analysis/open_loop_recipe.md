# Open-Loop Vx/Vy Velocity Characterization — Operator Runbook

**Task:** Wave 1 of the vx/vy characterization plan (T4) — produce the open-loop
step-response dataset that T5 will fit with FOPDT and use to retune the X/Y loop.

**Scope:** open-loop (PID-bypass) steps on **one linear axis at a time** using the
per-axis firmware gates `pid_vx_enable` / `pid_vy_enable` / `pid_vth_enable`
(`robot/Core/Src/system_globals.c:42-44`, defaults all `1`). The *other* linear
PID and the theta-hold stay **enabled** — that is the whole point of the gates:
during a vx step, `vy_cmd=0` so the vy-PID holds `vy≈0`, and the Lua harness holds
the captured orientation.

**Ground truth for variable names** (do NOT invent others): `analysis/TestescalonVx.csv`
header row is exactly:

```
time,ax_ms2,v_vel[0],vx_enc,vx_est,
```

So `time`, `ax_ms2`, `v_vel[0]`, `vx_enc`, `vx_est` are **confirmed** as
stm32monitor record variables. Anything else in the record lists below carries a
**VERIFY** marker — confirm it at the bench before relying on it.

**Sanity reference (dry-run fit already done on real data):** TestescalonVx.csv
gave FOPDT **K=0.381, L=95 ms, T=245 ms**. Use that as the ballpark — a wildly
different fit after a clean step means the run was contaminated (see G3 §5).

---

## 1. Pre-requisites — build + flash the firmware with the per-axis gates

The gates **already exist in source** (`system_globals.c:42-44`), but the firmware
**has NOT been flashed yet**. This machine has **no Makefile and no
`arm-none-eabi-gcc`** — the build **must** happen in the **STM32CubeIDE GUI**:

1. Open **STM32CubeIDE**, import the project from the repo root
   (`robot-firmware.ioc`, at `/home/sysmic/Desktop/SysmicProyect/robot/`).
2. **Build** (hammer icon). Fix any compile errors before continuing.
3. **Flash** via ST-Link: *Run ▸ Run* (or *Debug*) targeting the robot.
   Robot must be powered (battery or bench supply) and the ST-Link connected.

After a successful flash, power-cycle the robot and verify the gates are visible
to stm32monitor (each line must show `1`):

```bash
stm32monitor live pid_vx_enable,pid_vy_enable,pid_vth_enable
```

Expected stream (all defaults = 1; Ctrl+C to stop the live watch):

```
pid_vx_enable  : 1
pid_vy_enable  : 1
pid_vth_enable : 1
```

> **If a gate does not appear in the live watch**, the flashed binary predates
> the gates — re-build/re-flash and re-verify before touching the field.

---

## 2. METIS G4 CHECK — stm32monitor struct-member write syntax (BENCH, before field)

This bench check decides **how you will later write retuned gains in §6/retune**.
Do it **on the bench**, robot wheels free, BEFORE going to the field.

Run exactly these two commands:

```bash
stm32monitor native write pid_vx.Kp 2.5
stm32monitor live pid_vx.Kp
```

Watch the second command's output:

- **If the live watch shows `pid_vx.Kp : 2.5`** → the `.Kp` struct-member syntax
  works. Record this in your evidence; use `stm32monitor native write pid_vx.Kp <v>`
  for all live retuning.
- **If the write or the live watch fails** (unknown variable / parse error) →
  the PID struct member is likely nested. The GUI column in
  `docs/characterization-guide.md` uses `pid_vx.params.Kp`, and
  `drive_task.c:214` accesses `motor[i].pid.params.sampleTime` — i.e. the struct
  layout is `pid.params.*`. Try:

  ```bash
  stm32monitor native write pid_vx.params.Kp 2.5
  stm32monitor live pid_vx.params.Kp
  ```

  **VERIFY** which of the two spellings stm32monitor actually accepts — the guide
  documents both and they may not both work.
- **If both spellings fail** → live retuning is unavailable. **Fallback path:**
  edit the gain tables in `robot/Core/Src/drive_task.c`
  (`robotPidParams` / `thetaPidParams`, around lines 108–137), rebuild + reflash in
  STM32CubeIDE (§1). The FOPDT analysis in §6 still works the same; you just pay
  a flash cycle per retune.

**Cleanup — always restore the gain you poked:**

```bash
stm32monitor native write pid_vx.Kp 0.8     # or pid_vx.params.Kp 0.8 (whichever worked)
stm32monitor live pid_vx.Kp                 # confirm back to 0.8, then Ctrl+C
```

Current production defaults (from `docs/characterization-guide.md` §3.5):
`pid_vx/vy: Kp=0.8, Ki=1.6, outputMax=1.5`; `pid_vth: Kp=10.0, Ki=5.0`.

---

## 3. Open-loop vx run (FIELD) — the exact one-step procedure

**Setup:** robot on the field, batteries charged, engine connected, camera/vision
not required for this dataset (theta comes from the robot's own `theta_est`).

> The vy-PID **stays ON** (`pid_vy_enable=1`) and the theta-hold **stays ON**
> (`pid_vth_enable=1`, plus `THETA_HOLD=true` in the Lua harness). We only open
> the vx loop. **Do not** disable the other gates.

### 3.1 Disable the vx PID

```bash
stm32monitor native write pid_vx_enable 0
```

Verify the read-back (expect `0,1,1`; Ctrl+C to stop):

```bash
stm32monitor live pid_vx_enable,pid_vy_enable,pid_vth_enable
```

### 3.2 Start the engine logger (GUI Recording panel)

In the engine GUI **Recording panel**: type the filename `record_vx_p1.0.csv`
(default text input is `record.csv` — overwrite it), then press **Start**.
Engine CSV header (fixed): `ElapsedTime,RobotID,Team,Vx_Command,Vy_Command,Angular_Command,Pos_X,Pos_Y,Orientation,Vx_Actual,Vy_Actual`.

> **Engine CSV is multi-robot** (`logger.rs` writes 12 robot rows + 1 ball
> row per frame, all at the same `ElapsedTime` in seconds). Before feeding a
> `record_*.csv` to `sensor_compare.py` (T6), filter it to the commanded
> robot — otherwise the comparison is corrupted (proven: 782 Hz apparent
> rate, corr 0.87 instead of 0.999):
>
> ```bash
> awk -F, 'NR==1 || ($2==0 && $3==0)' record_vx_p1.0.csv > record_vx_p1.0_robot0.csv
> ```

### 3.3 Start the STM32Monitor recorder

```bash
stm32monitor record --vars time,ax_ms2,v_vel[0],v_vel[1],vx_enc,vx_est,vy_enc,vy_est,theta_est --output /home/sysmic/log/data_openloop_vx_p1.0.csv
```

- **VERIFY the var list at the bench first** (G4 session): `time, ax_ms2, v_vel[0], vx_enc, vx_est`
  are confirmed (TestescalonVx.csv columns). `v_vel[1], vy_enc, vy_est, theta_est`
  are **not** evidenced in that file — if stm32monitor rejects the list, trim it to
  exactly what it exposes (the `vy_*` / `theta_est` columns are needed for the G3
  cross-coupling check in §5, so prefer keeping them if available).
- **`/home/sysmic/log` is the STM32Monitor CSV output dir** — `analysis/live_telemetry.py`
  polls that directory and auto-loads the *latest* CSV. Use the absolute path above
  (or `cd /home/sysmic/log` first and use a relative name).
- The recorder streams until **Ctrl+C** (or the wrapper script kills it).
- **Fallback — raw telemetry capture**: if stm32monitor rejects the decoded-var list,
  record the raw 32-byte buffer instead (`telemetry_buf[0..31]`, as in the existing
  `analysis/stm32_monitor_log.csv`), then decode with
  `python3 analysis/telemetry_csv.py` (run from `robot/analysis/`; reads
  `stm32_monitor_log.csv` → writes `telemetry_decoded.csv` with
  `vx_est, vy_est, theta_est, omega_est, x_est, y_est` + `time` in seconds).
  **Verified 2026-08-06**: 10,892 rows decoded, exit 0, zero NaNs, values sane
  (static robot: vx/vy ≈ ±0.1 m/s, θ ∈ [-1.7, 1.5] rad). Output is directly
  `sensor_compare.py`-compatible (`time` is in the auto-detect set).

### 3.4 Configure and run the Lua step

Edit the CONFIG block at the top of
`/home/sysmic/Desktop/SysmicProyect/engine/engine/lua/velocity_step_test.lua`:

```lua
local ROBOT_ID        = 0      -- robot id to command
local TEAM            = 0      -- 0=blue, 1=yellow
local AXIS            = "vx"   -- "vx" or "vy": which axis to excite
local AMP             = 1.0    -- signed step amplitude in m/s (+1.0 or -1.0)
local DURATION_FRAMES = 60     -- step length in frames (~1 s at 60 FPS)
local THETA_HOLD      = true   -- hold the captured orientation as theta target
```

Load it: engine **Lua ▸ Load Script ▸ velocity_step_test.lua**. The harness
prints `[test] theta=...` on first call, then a `[step] vx 1.0 frame=N` line every
frame, then the completion marker:

```
[test] done vx amp=1.0
```

**Wait for that `[test] done` print before touching anything** (the step is
idempotent — re-running `process()` after `done` sends nothing).

### 3.5 Stop both recordings, re-enable the PID

1. Stop the STM32Monitor recorder: **Ctrl+C** (the recorder in 3.3).
2. Stop the engine GUI recording: **Recording panel ▸ Stop**.
3. Re-enable the vx PID:

```bash
stm32monitor native write pid_vx_enable 1
```

Verify read-back is back to `1,1,1`:

```bash
stm32monitor live pid_vx_enable,pid_vy_enable,pid_vth_enable
```

### 3.6 Automated equivalent — the wrapper script

Steps 3.1→3.5 (minus the two GUI actions) are wrapped in
`analysis/run_openloop_vx.sh`, which takes the amplitude as `$1`, echoes every
command, and offers `--dry-run` (prints without executing):

```bash
cd /home/sysmic/Desktop/SysmicProyect/robot/analysis
./run_openloop_vx.sh --dry-run 1.0      # preview the full sequence (safe)
./run_openloop_vx.sh 1.0                # real +1.0 m/s vx step
./run_openloop_vx.sh -1.0               # real -1.0 m/s vx step
```

The script pauses at the two manual steps (start engine recording; run the Lua
step) with `read` prompts, runs the STM32Monitor recorder in the background, and
**always restores `pid_vx_enable=1` via a trap** even if you Ctrl+C mid-run.

---

## 4. Step matrix — full vx/vy sweep

**Matrix** (open-loop, `pid_<axis>_enable=0`; other linear PID + theta-hold ON):

| # | axis | AMP [m/s] | DURATION_FRAMES (recommended) | engine CSV            | stm32monitor CSV                 |
|---|------|-----------|-------------------------------|-----------------------|----------------------------------|
| 1 | vx   | +0.3      | 60                            | `record_vx_p0.3.csv`  | `data_openloop_vx_p0.3.csv`      |
| 2 | vx   | -0.3      | 60                            | `record_vx_n0.3.csv`  | `data_openloop_vx_n0.3.csv`      |
| 3 | vx   | +0.6      | 60                            | `record_vx_p0.6.csv`  | `data_openloop_vx_p0.6.csv`      |
| 4 | vx   | -0.6      | 60                            | `record_vx_n0.6.csv`  | `data_openloop_vx_n0.6.csv`      |
| 5 | vx   | +1.0      | 75                            | `record_vx_p1.0.csv`  | `data_openloop_vx_p1.0.csv`      |
| 6 | vx   | -1.0      | 75                            | `record_vx_n1.0.csv`  | `data_openloop_vx_n1.0.csv`      |
| 7 | vx   | +1.2      | 90                            | `record_vx_p1.2.csv`  | `data_openloop_vx_p1.2.csv`      |
| 8 | vx   | -1.2      | 90                            | `record_vx_n1.2.csv`  | `data_openloop_vx_n1.2.csv`      |
| 9 | vy   | +0.3      | 60                            | `record_vy_p0.3.csv`  | `data_openloop_vy_p0.3.csv`      |
|10 | vy   | -0.3      | 60                            | `record_vy_n0.3.csv`  | `data_openloop_vy_n0.3.csv`      |
|11 | vy   | +0.6      | 60                            | `record_vy_p0.6.csv`  | `data_openloop_vy_p0.6.csv`      |
|12 | vy   | -0.6      | 60                            | `record_vy_n0.6.csv`  | `data_openloop_vy_n0.6.csv`      |
|13 | vy   | +1.0      | 75                            | `record_vy_p1.0.csv`  | `data_openloop_vy_p1.0.csv`      |
|14 | vy   | -1.0      | 75                            | `record_vy_n1.0.csv`  | `data_openloop_vy_n1.0.csv`      |
|15 | vy   | +1.2      | 90                            | `record_vy_p1.2.csv`  | `data_openloop_vy_p1.2.csv`      |
|16 | vy   | -1.2      | 90                            | `record_vy_n1.2.csv`  | `data_openloop_vy_n1.2.csv`      |

**File naming convention** `<kind>_<axis>_<sign>_<amp>.csv`:
`record_*` = engine GUI logger, `data_openloop_*` = stm32monitor recorder;
`sign` = `p` (positive) / `n` (negative); `amp` keeps the decimal point (`p1.0`).

**Rest between runs:** ~50 frames (~0.83 s at 60 FPS) of zero velocity after the
`[test] done` print before the next run (same WAIT=50 used by the multi-step
scripts in `docs/characterization-guide.md` §Fase 3). Let the robot come to rest
and the Kalman filters settle before the next step.

**DURATION_FRAMES rationale (see §6 ramp caveat):** the firmware rate-limits
linear accel at `ROBOT_MAX_LINEAR_ACC = 1.5 m/s²`
(`robot/Core/Inc/system_globals.h:15`), so a ±1.0 step needs ~0.67 s just to
finish ramping. Frames 60/75/90 above leave ≥0.5 s of true steady state so the
FOPDT **K is valid**; using 60 at ±1.0/±1.2 would leave only ~0.3 s of steady
state — marginal/bad. The `[step]` frame counter in the engine console tells you
when the step ends; the tail of the CSV tells you whether steady state was reached.

---

## 5. G3 ACCEPTANCE — cross-coupling check (reject contaminated steps)

For **every** step, verify there was no kinematic cross-coupling into the closed
loop kept alive on the other axis:

- **vx step:** reject the run if **|Δvy| > 0.1 m/s** or **|Δθ| > 0.05 rad**
  measured over the step window (from `[test] theta=` print until `[test] done`).
- **vy step:** reject the run if **|Δvx| > 0.1 m/s** or **|Δθ| > 0.05 rad** over
  the step window.

Δ means (max − min) of the column inside the step window — check `vy_est` (vx
runs) / `vx_est` (vy runs) and `theta_est` from the stm32monitor CSV. Quick check
with the analyzer (var names per §3.3 — **VERIFY** they exist in your CSV):

```bash
cd /home/sysmic/Desktop/SysmicProyect/robot
python3 analysis/csv_analyzer.py /home/sysmic/log/data_openloop_vx_p1.0.csv -c vy_est -a
python3 analysis/csv_analyzer.py /home/sysmic/log/data_openloop_vx_p1.0.csv -c theta_est -a
```

If a step fails G3 (e.g. robot bumped, theta drifted, one wheel lost traction),
**discard that CSV and re-run that cell of §4**. Since the other linear PID is
active with a 0 command, mild coupling should be suppressed — a large Δvy/Δθ is a
hardware/motion problem, not measurement noise.

**Repeat both directions, then average:** FOPDT fits per direction (e.g.
`p1.0` and `n1.0`) should agree within noise; use the mean of `K`, `L`, `T` from
the two signs as the plant for the retune (sign-invariant — the same plant governs
both directions).

---

## 6. Analysis — FOPDT fit + retune, with the RAMP CAVEAT

### 6.1 FOPDT fit per run

```bash
cd /home/sysmic/Desktop/SysmicProyect/robot

# Open-loop plant: command -> raw encoder velocity (no Kalman)
python3 analysis/pid_tuner.py /home/sysmic/log/data_openloop_vx_p1.0.csv -c v_vel[0] -m vx_enc --open-loop

# Same step, Kalman-estimated measurement
python3 analysis/pid_tuner.py /home/sysmic/log/data_openloop_vx_p1.0.csv -c v_vel[0] -m vx_est --open-loop

# vy runs: swap the measurement column
python3 analysis/pid_tuner.py /home/sysmic/log/data_openloop_vy_p1.0.csv -c v_vel[1] -m vy_enc --open-loop
python3 analysis/pid_tuner.py /home/sysmic/log/data_openloop_vy_p1.0.csv -c v_vel[1] -m vy_est --open-loop

# Batch all vx/vy runs at once (glob expands to every p*/n* file)
python3 analysis/pid_tuner.py /home/sysmic/log/data_openloop_vx_p*.csv -c v_vel[0] -m vx_enc --open-loop
python3 analysis/pid_tuner.py /home/sysmic/log/data_openloop_vy_p*.csv -c v_vel[1] -m vy_enc --open-loop
```

Optional JSON export for T5:

```bash
python3 analysis/pid_tuner.py /home/sysmic/log/data_openloop_vx_p1.0.csv -c v_vel[0] -m vx_enc --open-loop -e /home/sysmic/Desktop/SysmicProyect/robot/.omo/evidence/fopdt_vx_p1.0.json
```

### 6.2 ⚠️ RAMP CAVEAT — read before trusting K/L/T

The firmware **rate-limits linear velocity commands** at
`ROBOT_MAX_LINEAR_ACC = 1.5 m/s²` (`system_globals.h:15`, enforced in
`drive_task.c:376-378` via acc-magnitude scaling):

- per 5 ms control tick: 1.5 × 0.005 = **7.5 mm/s per tick**;
- a **1.0 m/s step ramps over ~0.67 s** (1.0 ÷ 1.5);
- a 1.2 m/s step ramps over ~0.80 s; a 0.3 m/s step ramps in ~0.20 s.

Consequences:

1. The input to the plant is **not a true step** — it is a ramp followed by a
   plateau. The FOPDT **L and T extracted from this data are UPPER BOUNDS**
   (the fit absorbs part of the ramp into apparent dead time / lag).
2. **K is only valid if steady state is actually reached** before the step ends.
   If the step ends mid-ramp, the fitted K underestimates the true plant gain.
   Prefer **short, high-amplitude** steps (large Δv in the shortest time that
   still reaches steady state — hence the 75/90-frame cells at ±1.0/±1.2).
3. When reporting to T5, state the amp used per fit; if you must pick one run per
   axis for the retune, prefer the **±1.2 m/s (or ±1.0) runs** — cleanest ramp-
   to-steady-state ratio.

### 6.3 Applying retuned gains

Using the syntax that **passed the G4 check in §2**:

```bash
# If pid_vx.Kp worked (or pid_vx.params.Kp / pid_vy.params.Kp — VERIFY at bench):
stm32monitor native write pid_vx.Kp <Kp>
stm32monitor native write pid_vx.Ki <Ki>
stm32monitor native write pid_vx.Kd 0.0

stm32monitor native write pid_vy.Kp <Kp>
stm32monitor native write pid_vy.Ki <Ki>
stm32monitor native write pid_vy.Kd 0.0
```

**Fallback if live writes are unavailable:** edit `robotPidParams`
(`drive_task.c:108-119`, Kp/Ki) for vx/vy, rebuild + reflash in STM32CubeIDE (§1).
Persist final values in `system_globals.c` and reflash once the closed-loop
verification (Wave 2, `characterize_vx_closed.lua`-style runs) passes.

---

## 7. Operator checklist (quick scan before leaving the field)

- [ ] `stm32monitor live pid_vx_enable,pid_vy_enable,pid_vth_enable` shows `1,1,1`
      (all gates restored after the last run).
- [ ] All 16 CSVs present under `/home/sysmic/log/` (`data_openloop_*`) + 16
      engine CSVs (`record_*`) per the §4 matrix.
- [ ] G4 bench result recorded (which gain-write syntax works).
- [ ] Every step passed G3 (no `|Δvy/Δvx| > 0.1 m/s`, no `|Δθ| > 0.05 rad`).
- [ ] Both signs captured per amp; no mid-ramp terminations (steady state visible
      in the CSV tail).
- [ ] Robot parked, batteries disconnected if done.

---

## Appendix A — command reference (copy-paste)

```bash
# Gates (all 3 exist in system_globals.c:42-44; default 1)
stm32monitor live pid_vx_enable,pid_vy_enable,pid_vth_enable
stm32monitor native write pid_vx_enable 0        # open vx loop (keep vy+theta closed)
stm32monitor native write pid_vy_enable 0        # open vy loop (keep vx+theta closed)
stm32monitor native write pid_vx_enable 1        # re-close vx loop
stm32monitor native write pid_vy_enable 1        # re-close vy loop

# G4 bench: struct-member write syntax probe
stm32monitor native write pid_vx.Kp 2.5
stm32monitor live pid_vx.Kp
stm32monitor native write pid_vx.params.Kp 2.5   # VERIFY: alternative spelling
stm32monitor live pid_vx.params.Kp
stm32monitor native write pid_vx.Kp 0.8          # restore

# Record (var list best-effort — trim to what stm32monitor exposes)
stm32monitor record --vars time,ax_ms2,v_vel[0],v_vel[1],vx_enc,vx_est,vy_enc,vy_est,theta_est \
    --output /home/sysmic/log/data_openloop_vx_p1.0.csv

# Wrapper (one step per run; --dry-run to preview)
cd /home/sysmic/Desktop/SysmicProyect/robot/analysis
./run_openloop_vx.sh --dry-run 1.0
./run_openloop_vx.sh 1.0
./run_openloop_vx.sh -1.2
./run_openloop_vy.sh 1.0

# Analysis
cd /home/sysmic/Desktop/SysmicProyect/robot
python3 analysis/pid_tuner.py /home/sysmic/log/data_openloop_vx_p1.0.csv -c v_vel[0] -m vx_enc --open-loop
python3 analysis/pid_tuner.py /home/sysmic/log/data_openloop_vy_p1.0.csv -c v_vel[1] -m vy_enc --open-loop
python3 analysis/csv_analyzer.py /home/sysmic/log/data_openloop_vx_p1.0.csv -c theta_est -a
```
