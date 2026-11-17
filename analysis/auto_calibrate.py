#!/usr/bin/env python3
"""auto_calibrate.py — Wheel PID auto-calibration over radio.

Drives each motor with open-loop DAC voltage steps (test mode), records
motor[i].measSpeed via calibration telemetry, extracts FOPDT models and
proposes PID gains (Ziegler-Nichols, Tyreus-Luyben, IMC).

The firmware must be built with the calibration-telemetry branch
(telemetry_calib_mode) and flashed. The robot must be powered and in radio
range of the basestation.

Usage:
  python3 auto_calibrate.py --id 0
  python3 auto_calibrate.py --id 0 --steps 300,-300,600,-600
  python3 auto_calibrate.py --dry-run          # simulate, no hardware
"""

import argparse
import math
import random
import sys
import time

try:
    import serial
except ImportError:
    serial = None

from radio_control import (RadioLink, CONFIG_REG_TELEMETRY_CALIB,
                           DEFAULT_PORT, DEFAULT_BAUD)

# ── ANSI ───────────────────────────────────────────────────────────────────
def _supports_color():
    return hasattr(sys.stdout, 'isatty') and sys.stdout.isatty()

def _c(code, text):
    return f'\033[{code}m{text}\033[0m' if _supports_color() else text

def green(t):  return _c('32', t)
def yellow(t): return _c('33', t)
def red(t):    return _c('31', t)
def cyan(t):   return _c('36', t)
def bold(t):   return _c('1', t)
def dim(t):    return _c('2', t)

# ── DEFAULTS ───────────────────────────────────────────────────────────────
DEFAULT_STEPS     = [300, -300, 600, -600, 900, -900, 1200, -1200]
DEFAULT_HOLD_S    = 1.8    # step hold time (steady state)
DEFAULT_SETTLE_S  = 0.6    # zero-voltage gap between steps
DEFAULT_BASELINE_S = 0.5   # pre-step recording at 0 V
NUM_MOTORS        = 4
CMD_PERIOD_S      = 0.02   # command refresh (matches radio_control CLI)


# ── RECORDING ──────────────────────────────────────────────────────────────
def record_step(link, robot_id, motor_idx, voltage, hold_s, settle_s,
                baseline_s=DEFAULT_BASELINE_S):
    """
    Record motor[i].measSpeed around one open-loop voltage step.

    Returns (t, speeds, step_start, step_end) where t is seconds since the
    start of the baseline and speeds are the motor's measSpeed (rad/s).
    """
    t, speeds = [], []
    t0 = time.monotonic()

    def sample():
        tel = link.read_telemetry_calib(timeout=0.05)
        if tel is None or tel['id'] != robot_id:
            return
        t.append(time.monotonic() - t0)
        speeds.append(tel['motor_speed'][motor_idx])

    # Baseline at 0 V
    link.send_test_motor(robot_id, motor_idx, 0)
    base_deadline = time.monotonic() + baseline_s
    while time.monotonic() < base_deadline:
        sample()

    # Step
    step_start = time.monotonic() - t0
    step_deadline = time.monotonic() + hold_s
    while time.monotonic() < step_deadline:
        link.send_test_motor(robot_id, motor_idx, voltage)
        sample()
        time.sleep(CMD_PERIOD_S)
    step_end = time.monotonic() - t0

    # Settle at 0 V
    link.send_test_motor(robot_id, motor_idx, 0)
    settle_deadline = time.monotonic() + settle_s
    while time.monotonic() < settle_deadline:
        sample()

    return t, speeds, step_start, step_end


def simulate_step(voltage, hold_s, settle_s, baseline_s=DEFAULT_BASELINE_S,
                  seed=0):
    """
    Dry-run replacement for record_step: a known FOPDT plant + noise.

    Plant: K=0.05 (rad/s)/DAC, T=0.12 s, L=0.02 s. Returns the same tuple
    shape so the analysis pipeline is exercised without hardware.
    """
    rng = random.Random(seed)
    K, T, L = 0.05, 0.12, 0.02
    t, speeds = [], []
    t0 = time.monotonic()
    step_t0 = None

    def plant(tt):
        if step_t0 is None:
            return 0.0
        rel = tt - step_t0
        if rel < L:
            return 0.0
        return K * voltage * (1.0 - math.exp(-(rel - L) / T))

    def sample():
        tt = time.monotonic() - t0
        t.append(tt)
        speeds.append(plant(tt) + rng.gauss(0.0, 0.02))

    link_sleep = 0.03   # ~33 Hz, matching the real radio link
    for _ in range(int(baseline_s / link_sleep)):
        sample()
        time.sleep(link_sleep)
    step_start = time.monotonic() - t0
    step_t0 = step_start
    for _ in range(int(hold_s / link_sleep)):
        sample()
        time.sleep(link_sleep)
    step_end = time.monotonic() - t0
    for _ in range(int(settle_s / link_sleep)):
        sample()
        time.sleep(link_sleep)

    return t, speeds, step_start, step_end


# ── ANALYSIS ───────────────────────────────────────────────────────────────
def analyze_step(t, speeds, step_start, step_end, voltage):
    """Extract FOPDT + tuning proposals for one step."""
    from pid_tuner import (extract_fopdt, zn_open_loop, tyreus_luyben,
                           imc_tuning)
    fopdt = extract_fopdt(t, speeds, step_start, step_end, 0.0, float(voltage))
    if fopdt is None:
        return None
    K, L, T = fopdt['K'], fopdt['L'], fopdt['T']
    methods = {}
    for name, fn in (('ZN', zn_open_loop), ('TL', tyreus_luyben),
                     ('IMC', imc_tuning)):
        try:
            methods[name] = fn(K, L, T)
        except Exception:
            methods[name] = None
    return {'voltage': voltage, 'K': K, 'L': L, 'T': T,
            'ss_meas': fopdt['ss_meas'], 'methods': methods}


def median_gains(results):
    """
    Median Kp/Ki across accepted steps per method, per motor.

    Returns {method: {'Kp': float, 'Ki': float}} or None if no results.
    """
    if not results:
        return None
    out = {}
    for method in ('ZN', 'TL', 'IMC'):
        kps, kis = [], []
        for r in results:
            m = r['methods'].get(method)
            if not m or 'PI' not in m:
                continue
            pi = m['PI']
            if pi['Kp'] > 0:
                kps.append(pi['Kp'])
                kis.append(pi['Ki'])
        if kps:
            kps.sort()
            kis.sort()
            out[method] = {'Kp': kps[len(kps) // 2], 'Ki': kis[len(kis) // 2]}
    return out or None


# ── REPORT ─────────────────────────────────────────────────────────────────
def _fmt(v, d=4):
    return f'{v:.{d}f}' if v is not None else '—'


def print_step_result(idx, r):
    if r is None:
        print(f'  step {idx}: {red("no FOPDT (flat/noisy response)")}')
        return
    print(f'  step {idx}: V={r["voltage"]:+5d}  '
          f'K={_fmt(r["K"])}  L={_fmt(r["L"] * 1000, 1)}ms  '
          f'T={_fmt(r["T"] * 1000, 1)}ms  ss={_fmt(r["ss_meas"], 2)} rad/s')
    for method, m in r['methods'].items():
        if not m or 'PI' not in m:
            continue
        pi = m['PI']
        print(f'      {method} PI: Kp={_fmt(pi["Kp"])}  Ki={_fmt(pi["Ki"])}')


def print_motor_summary(motor_idx, results, gains):
    print(bold(f'\nMotor {motor_idx} — {len(results)} accepted steps'))
    if gains is None:
        print(red('  No usable steps — check robot power, radio link, '
                  'and that calibration telemetry is enabled.'))
        return
    for method, g in gains.items():
        print(f'  {method} PI (median): Kp={_fmt(g["Kp"])}  '
              f'Ki={_fmt(g["Ki"])}   →  '
              f'wheel_pid_params[{motor_idx}].Kp={_fmt(g["Kp"], 1)}  '
              f'.Ki={_fmt(g["Ki"], 1)}')


def save_results(path, all_results):
    with open(path, 'w') as f:
        f.write('motor,step,voltage,K,L,T,ss_meas\n')
        for m, results in all_results.items():
            for i, r in enumerate(results):
                if r is None:
                    continue
                f.write(f'{m},{i},{r["voltage"]},{r["K"]:.6f},'
                        f'{r["L"]:.6f},{r["T"]:.6f},'
                        f'{r["ss_meas"]:.6f}\n')
    print(dim(f'Saved {path}'))


# ── MAIN ───────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description='Wheel PID auto-calibration over radio')
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--id', type=int, default=0, help='robot id (default 0)')
    ap.add_argument('--steps', default=','.join(map(str, DEFAULT_STEPS)),
                    help='comma-separated DAC voltages (default '
                         f'{DEFAULT_STEPS})')
    ap.add_argument('--hold', type=float, default=DEFAULT_HOLD_S,
                    help=f'step hold seconds (default {DEFAULT_HOLD_S})')
    ap.add_argument('--settle', type=float, default=DEFAULT_SETTLE_S,
                    help=f'settle seconds (default {DEFAULT_SETTLE_S})')
    ap.add_argument('--motors', default='0,1,2,3',
                    help='comma-separated motor indices (default 0,1,2,3)')
    ap.add_argument('--dry-run', action='store_true',
                    help='simulate the plant, no hardware')
    ap.add_argument('--out', metavar='CSV', help='save per-step results CSV')
    args = ap.parse_args()

    steps = [int(x) for x in args.steps.split(',') if x.strip()]
    motors = [int(x) for x in args.motors.split(',') if x.strip()]
    if not steps or not motors:
        print(red('--steps and --motors must be non-empty'))
        return 1

    if args.dry_run:
        print(cyan('DRY RUN — simulated plant (K=0.05, T=0.12s, L=0.02s)'))
        all_results = {}
        for motor_idx in motors:
            results = []
            for i, v in enumerate(steps):
                t, speeds, ss, se = simulate_step(v, args.hold, args.settle,
                                                  seed=motor_idx * 100 + i)
                r = analyze_step(t, speeds, ss, se, v)
                print_step_result(i, r)
                if r is not None:
                    results.append(r)
            all_results[motor_idx] = results
            print_motor_summary(motor_idx, results, median_gains(results))
        if args.out:
            save_results(args.out, all_results)
        return 0

    if serial is None:
        print(red('pyserial is not installed: pip install pyserial'))
        return 1

    print(cyan(f'Opening {args.port} @ {args.baud}...'))
    with RadioLink(args.port, args.baud) as link:
        print(yellow(f'Enabling calibration telemetry on robot {args.id}...'))
        link.send_config(args.id, CONFIG_REG_TELEMETRY_CALIB, 1)
        time.sleep(0.3)
        link.send_config(args.id, CONFIG_REG_TELEMETRY_CALIB, 1)

        all_results = {}
        try:
            for motor_idx in motors:
                print(bold(f'\n=== Motor {motor_idx} ==='))
                results = []
                for i, v in enumerate(steps):
                    print(dim(f'  step {i}: V={v:+5d} for {args.hold}s...'))
                    t, speeds, ss, se = record_step(
                        link, args.id, motor_idx, v, args.hold, args.settle)
                    r = analyze_step(t, speeds, ss, se, v)
                    print_step_result(i, r)
                    if r is not None:
                        results.append(r)
                all_results[motor_idx] = results
                print_motor_summary(motor_idx, results,
                                    median_gains(results))
        except KeyboardInterrupt:
            print(yellow('\nInterrupted — stopping motors.'))
        finally:
            for m in motors:
                link.send_test_motor(args.id, m, 0)
            link.send_config(args.id, CONFIG_REG_TELEMETRY_CALIB, 0)
            print(dim('Calibration telemetry disabled, motors at 0 V.'))

        if args.out:
            save_results(args.out, all_results)

    return 0


if __name__ == '__main__':
    sys.exit(main())