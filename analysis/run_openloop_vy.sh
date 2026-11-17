#!/usr/bin/env bash
# run_openloop_vy.sh -- one open-loop Vy step per invocation.
#
# Usage:
#   ./run_openloop_vy.sh 1.0             # +1.0 m/s vy step (engine logger + stm32monitor)
#   ./run_openloop_vy.sh -0.6            # -0.6 m/s vy step
#   ./run_openloop_vy.sh --dry-run 1.0   # print every command, execute nothing (safety)
#   ./run_openloop_vy.sh --help
#
# Sequence per run (the "one step per run" contract):
#   1. disable the vy PID      : stm32monitor native write pid_vy_enable 0
#   2. read-back verification  : live watch pid_vy_enable,pid_vx_enable,pid_vth_enable  -> 0,1,1
#   3. record                  : engine GUI Recording panel + stm32monitor record (background)
#   4. run Lua step            : velocity_step_test.lua (edited: AXIS="vy", AMP=$1)
#   5. re-enable the vy PID    : stm32monitor native write pid_vy_enable 1
#
# The vx PID and theta-hold STAY ENABLED during the vy step -- that is the whole
# point of the per-axis gates: vx-cmd=0 so vx-PID holds vx~0, THETA_HOLD holds
# the captured orientation. Only vy is open-loop.
#
# This script is OPERATOR TOOLING for hardware-in-the-loop use. It requires
# stm32monitor on PATH and a flashed robot connected via ST-Link. It will
# actually spin the wheels -- do not run it on the bench unless intended.

set -u

AXIS="vy"             # which axis this script excites (do not change here)
GATE="pid_vy_enable"  # the per-axis gate this script toggles (system_globals.c)
OTHER_GATE="pid_vx_enable"   # must STAY 1 (vx-PID keeps vx~0 during a vy step)
THETA_GATE="pid_vth_enable"  # must STAY 1 (theta-hold active)

# Recorded vars: best-effort list, cross-referenced with TestescalonVx.csv
# columns (CONFIRMED in that file: time, ax_ms2, v_vel[0], vx_enc, vx_est).
# v_vel[1], vy_enc, vy_est, theta_est are NOT evidenced in TestescalonVx.csv --
# VERIFY at the bench (G4 check) that stm32monitor exposes them and trim the
# list to exactly what it accepts. Order is irrelevant to the recorder.
VARS="time,ax_ms2,v_vel[0],v_vel[1],vx_enc,vx_est,vy_enc,vy_est,theta_est"

# STM32Monitor CSV output dir -- /home/sysmic/log is the dir live_telemetry.py
# polls for the LATEST csv. Override with:  STM32_LOG_DIR=/path ./run_openloop_vy.sh 1.0
LOG_DIR="${STM32_LOG_DIR:-/home/sysmic/log}"
LIVE_TIMEOUT=3        # seconds for the (streaming) live read-back probe

usage() {
    echo "Usage: ./run_openloop_vy.sh <AMP> [--dry-run]"
    echo ""
    echo "  <AMP>        signed step amplitude in m/s, e.g. 1.0 or -0.6"
    echo "  --dry-run    print every command without executing (operator safety)"
    echo "  -h, --help   show this help"
    echo ""
    echo "Example:  ./run_openloop_vy.sh --dry-run 1.0"
    echo "          ./run_openloop_vy.sh -1.2"
    echo ""
    echo "Edit the CONFIG block in engine/engine/lua/velocity_step_test.lua"
    echo "(AXIS=\"vy\", AMP=<same value>, DURATION_FRAMES=<see runbook matrix>)"
    echo "before running -- this script only handles the stm32monitor side."
    exit 0
}

DRY_RUN=0
AMP=""
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        -h|--help) usage ;;
        *) AMP="$arg" ;;
    esac
done

if [ -z "$AMP" ]; then
    echo "ERROR: no step amplitude given (try --help)." >&2
    usage
fi

case "$AMP" in
    -[0-9]*.[0-9]*|[0-9]*.[0-9]*|-[0-9]*|[0-9]*) : ;;
    *) echo "ERROR: AMP='$AMP' is not a number (e.g. 1.0 or -0.6)." >&2; exit 2 ;;
esac

# File naming convention <kind>_<axis>_<sign>_<amp>.csv  (sign: p = +, n = -)
if [ "${AMP#-}" = "${AMP}" ]; then SIGN="p"; else SIGN="n"; fi
AMPS="${AMP#-}"

# DURATION_FRAMES per runbook matrix (60/75/90 by |amp|): faster steps need
# longer windows because the firmware rate-limits at 1.5 m/s^2 (ramp caveat).
case "$AMPS" in
    0.3|0.6) DURATION_FRAMES=60 ;;
    1.0)     DURATION_FRAMES=75 ;;
    *)       DURATION_FRAMES=90 ;;
esac
OUT_FILE="data_openloop_${AXIS}_${SIGN}${AMPS}.csv"   # stm32monitor record output
REC_FILE="record_${AXIS}_${SIGN}${AMPS}.csv"          # engine GUI logger filename
OUT_PATH="${LOG_DIR}/${OUT_FILE}"
REC_LOG="${LOG_DIR}/${OUT_FILE}.rec.log"

echo "=== Open-loop ${AXIS} step: AMP=${AMP} m/s (sign=${SIGN}) ==="
echo "STM32Monitor CSV -> ${OUT_PATH}"
echo "Engine GUI CSV   -> ${REC_FILE}  (type this name in the Recording panel)"

# run(): echo the exact command, then execute it unless --dry-run.
run() {
    echo ">> $*"
    [ "$DRY_RUN" = "1" ] && return 0
    "$@"
}

# SAFETY: whatever happens (Ctrl+C, failure), restore the gate so the robot is
# never left in open-loop mode. A trap-restore is idempotent thanks to RESTORED.
RESTORED=0
cleanup() {
    [ "$DRY_RUN" = "1" ] && return 0
    [ "$RESTORED" = "1" ] && return 0
    RESTORED=1
    echo ">> SAFETY (trap): re-enabling ${GATE}=1"
    stm32monitor native write "$GATE" 1 >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# 1) Disable ONLY the vy PID. vx-PID + theta-hold stay active.
# ---------------------------------------------------------------------------
run stm32monitor native write "$GATE" 0

# 2) Read-back: expect 0,1,1 in the stream. `timeout` stops the streaming live
#    watch after LIVE_TIMEOUT seconds so the script cannot hang.
run timeout "$LIVE_TIMEOUT" stm32monitor live "$GATE","$OTHER_GATE","$THETA_GATE"

# ---------------------------------------------------------------------------
# 3) Operator (manual): start the ENGINE GUI recording FIRST.
# ---------------------------------------------------------------------------
echo ">> STEP 3 (manual): engine Recording panel -> filename '${REC_FILE}' -> Start"
read -r -p "   Press Enter once the engine logger is recording... " _

# ---------------------------------------------------------------------------
# 4) Start the STM32Monitor recorder in the background.
# ---------------------------------------------------------------------------
echo ">> stm32monitor record --vars ${VARS} --output ${OUT_PATH}   (background)"
if [ "$DRY_RUN" = "1" ]; then
    echo ">>   [dry-run] background recorder would start here; PID + log: ${REC_LOG}"
else
    stm32monitor record --vars "$VARS" --output "$OUT_PATH" >"$REC_LOG" 2>&1 &
    REC_PID=$!
    echo ">> recorder PID ${REC_PID} -> ${OUT_PATH} (log: ${REC_LOG})"
fi

# ---------------------------------------------------------------------------
# 5) Operator (manual): load velocity_step_test.lua and let the step complete.
# ---------------------------------------------------------------------------
echo ">> STEP 5 (manual): engine Lua > Load Script > velocity_step_test.lua"
echo ">>   CONFIG must read:  AXIS=\"${AXIS}\"  AMP=${AMP}  DURATION_FRAMES=${DURATION_FRAMES}"
echo ">>   watch the '[step]' diagnostics and the final '[test] done' print"
read -r -p "   Press Enter after the engine prints '[test] done ...'... " _

# ---------------------------------------------------------------------------
# 6) Stop the STM32Monitor recorder.
# ---------------------------------------------------------------------------
if [ "$DRY_RUN" = "1" ]; then
    echo ">> kill \$REC_PID   (stop the stm32monitor recorder)"
else
    kill "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
    echo ">> recorder stopped (${OUT_PATH})"
fi

# ---------------------------------------------------------------------------
# 7) Operator (manual): stop the ENGINE GUI recording.
# ---------------------------------------------------------------------------
echo ">> STEP 7 (manual): engine Recording panel -> Stop"

# ---------------------------------------------------------------------------
# 8) Re-enable the vy PID.
# ---------------------------------------------------------------------------
run stm32monitor native write "$GATE" 1
RESTORED=1

echo "=== ${AXIS} step done. Data: ${OUT_PATH} ==="
echo "=== Next: check G3 acceptance (|dvx|<=0.1 m/s, |dtheta|<=0.05 rad), ==="
echo "=== then repeat with the opposite sign and average (see runbook). ==="
