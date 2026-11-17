#!/usr/bin/env python3
"""
PID Tuner for Sysmic Robotics.

Two modes:

  --closed-loop  (default)  Analyze closed-loop step response, compute
    performance metrics (rise time, overshoot, ss_error, settling time),
    and suggest heuristic gain adjustments.  Requires --current gains.

  --open-loop  Extract FOPDT plant model from step response and apply
    Ziegler-Nichols, Tyreus-Luyben and IMC tuning rules.

  --plant K,L,T  Skip CSV loading; apply tuning rules directly to
    the given FOPDT plant parameters.

Usage:
    # Closed-loop: diagnose current response + heuristic deltas
    python3 pid_tuner.py Vth-40+4.csv --cmd v_vel[2] --meas vth_est --current 2.0,6.0,0.0

    # Open-loop: extract FOPDT + tuning rules
    python3 pid_tuner.py Vth_openloop.csv --cmd v_vel[2] --meas vth_est --open-loop

    # Plant parameters directly
    python3 pid_tuner.py --plant K=1.5,L=0.02,T=0.15

    # Export to JSON
    python3 pid_tuner.py Vth-40+4.csv -c v_vel[2] -m vth_est --current 2.0,6.0,0.0 --export tune.json
"""
import csv
import json
import math
import os
import re
import sys
from collections import defaultdict

try:
    import numpy as np
except ImportError:
    np = None

try:
    from matplotlib import pyplot as plt
    from matplotlib import gridspec
except ImportError:
    plt = None


# ── ANSI ───────────────────────────────────────────────────────────────────
def _supports_color():
    if not sys.stdout.isatty():
        return False
    return True

_USE_COLOR = _supports_color()

def _c(code, text):
    return f'\033[{code}m{text}\033[0m' if _USE_COLOR else text

def green(t):  return _c('32', t)
def yellow(t): return _c('33', t)
def red(t):    return _c('31', t)
def cyan(t):   return _c('36', t)
def bold(t):   return _c('1', t)
def dim(t):    return _c('2', t)


# ── CSV LOADER ─────────────────────────────────────────────────────────────
def load_csv(path):
    with open(path) as f:
        reader = csv.reader(f)
        raw_header = next(reader)
    headers = [h.strip() for h in raw_header if h.strip()]
    time_col = None
    for i, h in enumerate(headers):
        if h.lower() in ("time", "timestamp", "t[s]", "t"):
            time_col = i
            break
    if time_col is None:
        time_col = 0

    col_data = defaultdict(list)
    with open(path) as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            if len(row) < len(headers):
                continue
            try:
                t = float(row[time_col])
            except (ValueError, IndexError):
                continue
            for i, h in enumerate(headers):
                if i == time_col:
                    continue
                try:
                    col_data[h].append((t, float(row[i])))
                except (ValueError, IndexError):
                    pass
    return headers, col_data, time_col


# ── STEP DETECTION ─────────────────────────────────────────────────────────
def find_steps(t, v, min_samples=20, hysteresis=None):
    if len(v) < 2:
        return []
    if hysteresis is None:
        rng = max(v) - min(v)
        hysteresis = max(rng * 0.01, 1e-6)
    steps = []
    prev_val = v[0]
    start_i = 0
    for i in range(1, len(v)):
        if abs(v[i] - prev_val) > hysteresis:
            seg_len = i - start_i
            if seg_len >= min_samples:
                steps.append((t[start_i], t[i - 1], prev_val, v[i]))
            start_i = i
            prev_val = v[i]
    if len(v) - start_i >= min_samples:
        steps.append((t[start_i], t[-1], prev_val, v[-1]))
    return steps


def filter_forward_steps(steps, min_delta=0.5):
    """Keep only steps that go *to* a new non-zero level (ignore return-to-zero)."""
    if not steps:
        return []
    forward = [steps[0]]
    for i in range(1, len(steps)):
        _, _, _, val_after = steps[i]
        if abs(val_after) > 0.1 or abs(val_after - forward[-1][3]) > min_delta:
            forward.append(steps[i])
    return forward


# ── CLOSED-LOOP METRICS ────────────────────────────────────────────────────
def closed_loop_metrics(t, meas, step_start, step_end, cmd_before, cmd_after):
    """
    Extract performance metrics from a closed-loop step response segment.
    """
    i_start = None
    for i, tt in enumerate(t):
        if tt >= step_start:
            i_start = i
            break
    i_end = None
    for i, tt in enumerate(t):
        if tt >= step_end:
            i_end = i
            break
    if i_start is None or i_end is None or i_end - i_start < 5:
        return None

    t_seg = t[i_start:i_end]
    m_seg = meas[i_start:i_end]
    delta = cmd_after - cmd_before
    if abs(delta) < 1e-6:
        return None

    # Steady-state: mean of last 30%
    ss_start = int(len(m_seg) * 0.7)
    ss_meas = (sum(m_seg[ss_start:]) / len(m_seg[ss_start:])
               if len(m_seg) > ss_start else m_seg[-1])
    total_change = ss_meas - cmd_before
    ss_error = cmd_after - ss_meas

    # Dead time / latency (5% crossing)
    thresh_5 = 0.05 * total_change
    L = None
    for i in range(len(m_seg)):
        if (total_change > 0 and m_seg[i] - cmd_before >= thresh_5) or \
           (total_change < 0 and m_seg[i] - cmd_before <= thresh_5):
            L = t_seg[i] - t_seg[0]
            break

    # Rise time (63%)
    target_63 = cmd_before + 0.632 * total_change
    rise_time = None
    for i in range(len(m_seg)):
        if (total_change > 0 and m_seg[i] >= target_63) or \
           (total_change < 0 and m_seg[i] <= target_63):
            rise_time = t_seg[i] - t_seg[0]
            break

    # Overshoot
    if delta > 0:
        peak = max(m_seg)
        overshoot_pct = max(0, (peak - ss_meas) / abs(total_change) * 100) if abs(total_change) > 1e-6 else 0
    else:
        peak = min(m_seg)
        overshoot_pct = max(0, (ss_meas - peak) / abs(total_change) * 100) if abs(total_change) > 1e-6 else 0

    # Settling time (2%, reverse scan)
    settle_2pct = None
    band = 0.02 * abs(total_change)
    if band > 1e-9:
        for i in range(len(m_seg) - 1, -1, -1):
            if abs(m_seg[i] - ss_meas) > band:
                settle_2pct = max(0, t_seg[min(i + 1, len(m_seg) - 1)] - t_seg[0])
                break

    # Noise in steady-state
    noisy = False
    if len(m_seg) > ss_start + 10:
        tail = m_seg[ss_start:]
        mean_tail = sum(tail) / len(tail)
        if abs(mean_tail) > 1e-6:
            var = sum((v - mean_tail) ** 2 for v in tail) / len(tail)
            noisy = math.sqrt(var) / abs(mean_tail) > 0.05

    return {
        'cmd_before': cmd_before,
        'cmd_after': cmd_after,
        'ss_meas': ss_meas,
        'ss_error': ss_error,
        'total_change': total_change,
        'latency': L,
        'rise_time': rise_time,
        'overshoot_pct': overshoot_pct,
        'settle_time_2pct': settle_2pct,
        'peak': peak,
        'noisy': noisy,
    }


# ── FOPDT EXTRACTION (open-loop) ───────────────────────────────────────────
def extract_fopdt(t, meas, step_start, step_end, cmd_before, cmd_after):
    """
    Extract FOPDT (K, L, T) from an open-loop step response.
    """
    i_start = None
    for i, tt in enumerate(t):
        if tt >= step_start:
            i_start = i
            break
    i_end = None
    for i, tt in enumerate(t):
        if tt >= step_end:
            i_end = i
            break
    if i_start is None or i_end is None or i_end - i_start < 5:
        return None

    t_seg = t[i_start:i_end]
    m_seg = meas[i_start:i_end]
    delta = cmd_after - cmd_before
    if abs(delta) < 1e-6:
        return None

    # Steady-state
    ss_start = int(len(m_seg) * 0.7)
    ss_meas = (sum(m_seg[ss_start:]) / len(m_seg[ss_start:])
               if len(m_seg) > ss_start else m_seg[-1])
    # Use actual initial measurement, not cmd_before (which may be a different
    # physical unit in open-loop mode, e.g. voltage vs speed).
    initial_meas = m_seg[0] if m_seg else ss_meas
    total_change = ss_meas - initial_meas

    # Process gain  (Δoutput / Δinput)
    K = total_change / delta if abs(delta) > 1e-9 else 0.0

    # Dead time (first crossing of 5% of total change)
    L = 0.0
    thresh_5 = 0.05 * total_change
    for i in range(len(m_seg)):
        if (total_change > 0 and m_seg[i] - initial_meas >= thresh_5) or \
           (total_change < 0 and m_seg[i] - initial_meas <= thresh_5):
            L = t_seg[i] - t_seg[0]
            break

    # Time constant: L → 63.2%
    target_63 = initial_meas + 0.632 * total_change
    T = None
    for i in range(len(m_seg)):
        if t_seg[i] - t_seg[0] >= L:
            if (total_change > 0 and m_seg[i] >= target_63) or \
               (total_change < 0 and m_seg[i] <= target_63):
                T = t_seg[i] - t_seg[0] - L
                break
    if T is None or T <= 0:
        T = (t_seg[-1] - t_seg[0] - L) if len(t_seg) > 1 else 0.1

    return {'K': K, 'L': L, 'T': T, 'ss_meas': ss_meas}


# ── OPEN-LOOP TUNING RULES ─────────────────────────────────────────────────
def zn_open_loop(K, L, T):
    if K <= 0 or L <= 1e-9 or T <= 1e-9:
        return None
    a = K * L / T
    return {
        'P':  {'Kp': 1.0 / a, 'Ki': 0.0, 'Kd': 0.0},
        'PI': {'Kp': 0.9 / a, 'Ki': 0.9 / a / (3.33 * L), 'Kd': 0.0},
        'PID':{'Kp': 1.2 / a, 'Ki': 1.2 / a / (2.0 * L), 'Kd': 1.2 / a * L * 0.5},
    }

def tyreus_luyben(K, L, T):
    if K <= 0 or L <= 1e-9 or T <= 1e-9:
        return None
    a = K * L / T
    return {
        'PI': {'Kp': 0.31 / a, 'Ki': 0.31 / a / (3.33 * L), 'Kd': 0.0},
        'PID':{'Kp': 0.45 / a, 'Ki': 0.45 / a / (2.3 * L), 'Kd': 0.45 / a * (0.42 * L)},
    }

def imc_tuning(K, L, T, lam=None):
    if K <= 0 or L <= 1e-9 or T <= 1e-9:
        return None
    if lam is None:
        lam = max(0.25 * T, 2.0 * L)
    return {
        'P':  {'Kp': T / (K * (L + lam)), 'Ki': 0.0, 'Kd': 0.0},
        'PI': {'Kp': T / (K * (L + lam)), 'Ki': 1.0 / (K * (L + lam)), 'Kd': 0.0},
        'PID':{'Kp': T / (K * (L + lam)), 'Ki': 1.0 / (K * (L + lam)),
               'Kd': T / (K * (L + lam)) * L * 0.5},
    }


# ── HEURISTIC (closed-loop) ────────────────────────────────────────────────
def heuristic_tuning(current_gains, metrics):
    Kp, Ki, Kd = current_gains
    suggestions = []
    warnings = []
    dKp = dKi = dKd = 0.0

    os = metrics.get('overshoot_pct', 0.0)
    rt = metrics.get('rise_time')
    ss_err = metrics.get('ss_error', 0.0)
    clipped = metrics.get('clipped', False)
    noisy = metrics.get('noisy', False)

    if clipped:
        warnings.append(red('Saturacion detectada — reducir Kp o aumentar outputMax'))

    if os > 30:
        f = 1.0 - min(0.5, (os - 30) / 100.0)
        dKp -= Kp * (1.0 - f)
        suggestions.append(f'Overshoot alto ({os:.0f}%): reducir Kp ~{(1-f)*100:.0f}%')
        if Ki > 0:
            dKi -= Ki * 0.2
            suggestions.append(f'  Tambien reducir Ki ~20%')
    elif os > 15:
        f = 1.0 - min(0.3, (os - 15) / 100.0)
        dKp -= Kp * (1.0 - f)
        suggestions.append(f'Overshoot moderado ({os:.0f}%): reducir Kp ~{(1-f)*100:.0f}%')
    elif os < 5 and rt is not None:
        if rt > 0.20:
            dKp += Kp * 0.30
            suggestions.append(f'Respuesta lenta ({rt*1000:.0f}ms): aumentar Kp ~30%')
        elif rt > 0.10:
            dKp += Kp * 0.15
            suggestions.append(f'Respuesta algo lenta ({rt*1000:.0f}ms): aumentar Kp ~15%')
        else:
            suggestions.append(green(f'Overshoot y velocidad aceptables'))
    else:
        suggestions.append(green(f'Overshoot aceptable ({os:.1f}%)'))

    if abs(ss_err) > 0.1 and Ki > 0:
        pct = min(100, abs(ss_err) / max(abs(metrics.get('cmd_after', 1)), 0.01) * 100)
        dKi += abs(ss_err) * 1.5
        suggestions.append(f'Error estacionario ({ss_err:.3f}, ~{pct:.0f}%): aumentar Ki')
    elif abs(ss_err) > 0.05 and Ki > 0:
        dKi += Ki * 0.15
        suggestions.append(f'Error estacionario leve ({ss_err:.3f}): aumentar Ki ~15%')
    elif abs(ss_err) > 0.1 and Ki == 0:
        dKi += 0.1
        suggestions.append(f'Error estacionario ({ss_err:.3f}) con Ki=0: aniadir Ki>0')

    if noisy and Kd > 0:
        dKd -= Kd * 0.5
        suggestions.append('Ruido en salida: reducir Kd ~50% o aumentar filtro')
    elif noisy:
        warnings.append(yellow('Ruido detectado — considerar filtro IIR'))

    new_Kp = max(Kp * 0.1, Kp + dKp)
    new_Ki = max(0.0, Ki + dKi)
    new_Kd = max(0.0, Kd + dKd)

    return {
        'current': {'Kp': Kp, 'Ki': Ki, 'Kd': Kd},
        'suggested': {'Kp': round(new_Kp, 4), 'Ki': round(new_Ki, 4), 'Kd': round(new_Kd, 4)},
        'delta': {'Kp': round(Kp + dKp - Kp, 4), 'Ki': round(Ki + dKi - Ki, 4), 'Kd': round(Kd + dKd - Kd, 4)},
        'suggestions': suggestions,
        'warnings': warnings,
    }


# ── REPORT HELPERS ─────────────────────────────────────────────────────────
def _fmt_ms(val):
    if val is None:
        return dim('--')
    return f'{val*1000:.0f}ms'

def _fmt(val, d=4):
    if val is None:
        return dim('--')
    return f'{val:.{d}f}'

def _fmt_gain(val, d=4):
    t = f'{val:.{d}f}'
    return bold(cyan(t)) if val > 0 else dim(t)

def _sep(label):
    print(f"\n  {bold(f'── {label} ──')}")


# ── REPORT: CLOSED-LOOP ────────────────────────────────────────────────────
def report_closed_loop(metrics, heuristic, fname, cmd_col, meas_col, step_no):
    print(f"\n{'=' * 70}")
    print(f"  {bold('PID Tuner — Closed-Loop Analysis')}")
    print(f"  {os.path.basename(fname)}  |  Step {step_no}: {metrics['cmd_before']:.1f} -> {metrics['cmd_after']:.1f}")
    print(f"{'=' * 70}")

    _sep('Current Performance')
    print(f"    Rise time (63%):     {_fmt_ms(metrics['rise_time'])}")
    print(f"    Overshoot:           {metrics['overshoot_pct']:.1f}%  (peak={metrics['peak']:.3f})")
    print(f"    Settling time (2%):  {_fmt_ms(metrics['settle_time_2pct'])}")
    print(f"    Steady-state err:    {metrics['ss_error']:.4f}  (cmd={metrics['cmd_after']}, meas={metrics['ss_meas']:.3f})")
    print(f"    Latency:             {_fmt_ms(metrics['latency'])}")
    if metrics['noisy']:
        print(f"    Noise:               {yellow('HIGH')}")

    _sep('Heuristic Adjustment')
    cur = heuristic['current']
    sug = heuristic['suggested']
    delta = heuristic['delta']

    print(f"    Current:   Kp={cur['Kp']:.4f}  Ki={cur['Ki']:.4f}  Kd={cur['Kd']:.4f}")
    print(f"    Suggested: Kp={sug['Kp']:.4f}  Ki={sug['Ki']:.4f}  Kd={sug['Kd']:.4f}")
    print(f"    Delta:     {delta['Kp']:+9.4f}  {delta['Ki']:+9.4f}  {delta['Kd']:+9.4f}")
    for s in heuristic['suggestions']:
        print(f"    {s}")
    for w in heuristic['warnings']:
        print(f"    {w}")

    _sep('Copy to drive_task.c (lines 146-151)')
    print(f"    robotPidParams.Kp = {sug['Kp']:.4f}f;")
    print(f"    robotPidParams.Ki = {sug['Ki']:.4f}f;")
    print(f"    robotPidParams.Kd = {sug['Kd']:.4f}f;")

    _sep('Next Iteration')
    print(f"    python3 pid_tuner.py <new_csv> -c {cmd_col} -m {meas_col} --current {sug['Kp']},{sug['Ki']},{sug['Kd']}")
    print(f"    python3 csv_analyzer.py <prev_csv> <new_csv> --compare")
    print()


# ── REPORT: OPEN-LOOP ──────────────────────────────────────────────────────
def report_open_loop(fopdt, methods, fname, cmd_col, meas_col, step_no):
    print(f"\n{'=' * 70}")
    print(f"  {bold('PID Tuner — Open-Loop Tuning')}")
    print(f"  {os.path.basename(fname)}  |  Step {step_no}")
    print(f"{'=' * 70}")

    K, L, T = fopdt['K'], fopdt['L'], fopdt['T']
    tau_over_L = T / L if L > 1e-6 else float('inf')

    _sep('FOPDT Plant Model')
    print(f"    Process gain K:  {K:.4f}")
    print(f"    Dead time  L:    {L*1000:.1f} ms")
    print(f"    Time const T:    {T*1000:.1f} ms")
    print(f"    T / L ratio:     {tau_over_L:.1f}  (>{'3' if tau_over_L > 3 else '3'}: {'controllable' if tau_over_L > 3 else 'difficult'})")

    if tau_over_L < 3:
        print(f"    {yellow('Warning: low T/L — plant is dead-time dominated, PID may struggle')}")

    _sep('Tuning Rules')
    hdr = f"    {'Method':<18s} {'Type':<8s} {'Kp':>10s} {'Ki':>12s} {'Kd':>12s}"
    print(hdr)
    print(f"    {'-'*18} {'-'*8} {'-'*10} {'-'*12} {'-'*12}")

    all_pid = {}
    for m_name, result in methods.items():
        if result is None:
            continue
        for ctrl, gains in result.items():
            if gains is None:
                continue
            all_pid[f'{m_name}-{ctrl}'] = gains
            print(f"    {m_name:<18s} {ctrl:<8s}  {_fmt_gain(gains['Kp']):>10s}  "
                  f"{_fmt(gains['Ki'],6):>12s}  {_fmt(gains['Kd'],6):>12s}")

    _sep('Recommended (IMC-PI conservative)')
    imc_pi = all_pid.get('IMC-PI')
    if imc_pi:
        print(f"    robotPidParams.Kp = {imc_pi['Kp']:.4f}f;")
        print(f"    robotPidParams.Ki = {imc_pi['Ki']:.4f}f;")
        print(f"    robotPidParams.Kd = 0.0f;")
        print(f"    {dim('(sampleTime = 0.005s for outer loop)')}")

    print()


# ── REPORT: PLANT (direct) ─────────────────────────────────────────────────
def report_plant_only(K, L, T):
    print(f"\n{'=' * 70}")
    print(f"  {bold('PID Tuner — Plant Parameters')}")
    print(f"{'=' * 70}")

    fopdt = {'K': K, 'L': L, 'T': T}
    methods = {
        'Ziegler-Nichols': zn_open_loop(K, L, T),
        'Tyreus-Luyben': tyreus_luyben(K, L, T),
        'IMC': imc_tuning(K, L, T),
    }
    report_open_loop(fopdt, methods, '(direct)', '', '', 1)


# ── PLOTTING ───────────────────────────────────────────────────────────────
def plot_closed_loop(fname, cmd_col, meas_col, metrics):
    if plt is None:
        return
    headers, data, _ = load_csv(fname)
    cmd_arr = np.array(data[cmd_col])
    meas_arr = np.array(data[meas_col])
    t0 = cmd_arr[0, 0]
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(cmd_arr[:, 0] - t0, cmd_arr[:, 1], '--', color='gray', alpha=0.6, label=cmd_col)
    ax.plot(meas_arr[:, 0] - t0, meas_arr[:, 1], '-', color='C0', label=meas_col)
    ax.axhline(metrics['cmd_after'], color='gray', linestyle=':', alpha=0.3)
    ax.axhline(metrics['ss_meas'], color='C0', linestyle=':', alpha=0.4,
               label=f"SS={metrics['ss_meas']:.3f}")
    if metrics['latency']:
        ax.axvline(metrics['latency'], color='red', linestyle='--', alpha=0.4)
    if metrics['settle_time_2pct']:
        ax.axvline(metrics['settle_time_2pct'], color='green', linestyle='--', alpha=0.4)
    ax.set_ylabel('Value')
    ax.set_title(f'Closed-Loop Step: {os.path.basename(fname)}')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

def plot_open_loop(fname, cmd_col, meas_col, fopdt):
    if plt is None:
        return
    headers, data, _ = load_csv(fname)
    cmd_arr = np.array(data[cmd_col])
    meas_arr = np.array(data[meas_col])
    t0 = cmd_arr[0, 0]
    t_m = meas_arr[:, 0] - t0

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), gridspec_kw={'height_ratios': [2, 1]})
    ax1.plot(t_m, meas_arr[:, 1], '-', color='C0', label=meas_col)
    if fopdt['L'] > 0:
        ax1.axvline(fopdt['L'], color='red', linestyle='--', alpha=0.5,
                    label=f"L={fopdt['L']*1000:.0f}ms")
    if fopdt['T'] > 0:
        ax1.axvline(fopdt['L'] + fopdt['T'], color='green', linestyle='--', alpha=0.5,
                    label=f"L+T={fopdt['L']+fopdt['T']:.3f}s")
    ax1.set_title(f'Open-Loop Step: {os.path.basename(fname)}')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    t_fit = np.linspace(0, t_m[-1], 500)
    y_fit = np.zeros_like(t_fit)
    cmd0 = meas_arr[0, 1]
    for i, tt in enumerate(t_fit):
        if tt < fopdt['L']:
            y_fit[i] = cmd0
        else:
            y_fit[i] = cmd0 + fopdt['K'] * (1 - math.exp(-(tt - fopdt['L']) / fopdt['T']))
    ax2.plot(t_m, meas_arr[:, 1], '-', alpha=0.5, label='Real')
    ax2.plot(t_fit, y_fit, '--', color='red', linewidth=1.5, label='FOPDT fit')
    ax2.set_xlabel('Time (s)')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


# ── MAIN ───────────────────────────────────────────────────────────────────
def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='PID Tuner — Sysmic Robotics',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)

    parser.add_argument('files', nargs='*', help='CSV file(s)')
    parser.add_argument('--cmd', '-c', help='Command column (auto-detect)')
    parser.add_argument('--meas', '-m', help='Measurement column (auto-detect)')
    parser.add_argument('--current', '-g', help='Current gains Kp,Ki,Kd (e.g. 2.0,6.0,0.0)')
    parser.add_argument('--open-loop', '-o', action='store_true',
                        help='Open-loop mode: extract FOPDT + tuning rules')
    parser.add_argument('--plant', type=str,
                        help='Direct FOPDT: K=val,L=val,T=val (skip CSV)')
    parser.add_argument('--no-plot', '-n', action='store_true', help='Skip plotting')
    parser.add_argument('--export', '-e', metavar='FILE.json', help='Export to JSON')
    args = parser.parse_args()

    if not args.no_plot and plt is None:
        print("Warning: matplotlib not installed.")
        args.no_plot = True

    # ── Mode: direct plant parameters ──
    if args.plant:
        m = re.match(r'K=([\d.]+),\s*L=([\d.]+),\s*T=([\d.]+)', args.plant)
        if not m:
            print(red("Error: --plant format must be K=val,L=val,T=val"))
            sys.exit(1)
        K, L, T = float(m[1]), float(m[2]), float(m[3])
        report_plant_only(K, L, T)
        return

    if not args.files:
        parser.print_help()
        sys.exit(1)

    # ── Load CSV ──
    for fname in args.files:
        if not os.path.exists(fname):
            print(f"File not found: {fname}")
            continue

        headers, data, _ = load_csv(fname)
        data_cols = [h for h in headers]

        # Auto-detect command
        cmd_col = args.cmd
        if cmd_col is None:
            possible = [h for h in data_cols if any(k in h.lower() for k in
                        ('v_vel', 'cmd', 'ref', 'setpoint', 'target', 'command'))]
            cmd_col = possible[0] if possible else data_cols[1]

        # Auto-detect measurement
        meas_col = args.meas
        if meas_col is None:
            skip = ('time', 'timestamp', 't[s]', 't')
            cand = [h for h in data_cols if h != cmd_col and h.lower() not in skip]
            est = [h for h in cand if 'est' in h.lower()]
            meas_col = (est or cand)[0]

        cmd_arr = data[cmd_col]
        t_cmd = [x[0] for x in cmd_arr]
        v_cmd = [x[1] for x in cmd_arr]
        meas_arr = data[meas_col]
        t_meas = [x[0] for x in meas_arr]
        v_meas = [x[1] for x in meas_arr]

        steps = find_steps(t_cmd, v_cmd)
        steps = filter_forward_steps(steps)

        if len(steps) < 2:
            print(yellow(f"No step transitions found in {fname}"))
            continue

        all_results = []

        for si in range(len(steps) - 1):
            _, s_end, val_before, val_after = steps[si]
            _, s_next, _, _ = steps[si + 1]
            step_start = s_end
            step_end = s_next

            # Skip steps where command change is small
            if abs(val_after - val_before) < 0.5:
                continue

            if args.open_loop:
                # ── Open-loop analysis ──
                fopdt = extract_fopdt(t_meas, v_meas, step_start, step_end,
                                      val_before, val_after)
                if fopdt is None:
                    continue
                methods = {
                    'Ziegler-Nichols': zn_open_loop(fopdt['K'], fopdt['L'], fopdt['T']),
                    'Tyreus-Luyben': tyreus_luyben(fopdt['K'], fopdt['L'], fopdt['T']),
                    'IMC': imc_tuning(fopdt['K'], fopdt['L'], fopdt['T']),
                }
                report_open_loop(fopdt, methods, fname, cmd_col, meas_col, si + 1)
                all_results.append({
                    'step': si + 1, 'from': val_before, 'to': val_after,
                    'mode': 'open-loop', 'fopdt': fopdt,
                })
                if not args.no_plot:
                    plot_open_loop(fname, cmd_col, meas_col, fopdt)

            else:
                # ── Closed-loop analysis ──
                metrics = closed_loop_metrics(t_meas, v_meas, step_start, step_end,
                                              val_before, val_after)
                if metrics is None:
                    continue

                # Parse current gains
                current_gains = None
                if args.current:
                    try:
                        current_gains = tuple(float(x) for x in args.current.split(','))
                    except ValueError:
                        pass

                if current_gains:
                    heuristic = heuristic_tuning(current_gains, metrics)
                else:
                    heuristic = None
                    print(f"\n  {yellow('Tip: add --current Kp,Ki,Kd to get heuristic suggestions')}")

                report_closed_loop(metrics, heuristic or {'current': {}, 'suggested': {}, 'delta': {},
                                   'suggestions': [], 'warnings': []},
                                   fname, cmd_col, meas_col, si + 1)

                if not args.no_plot:
                    plot_closed_loop(fname, cmd_col, meas_col, metrics)

                all_results.append({
                    'step': si + 1, 'from': val_before, 'to': val_after,
                    'mode': 'closed-loop', 'metrics': {k: v for k, v in metrics.items()
                                                       if k != 'cmd_before'},
                    'heuristic': heuristic,
                })

        # Export
        if args.export and all_results:
            export = {
                'file': os.path.basename(fname),
                'command_column': cmd_col,
                'measurement_column': meas_col,
                'mode': 'open-loop' if args.open_loop else 'closed-loop',
                'steps': all_results,
            }
            with open(args.export, 'w') as f:
                json.dump(export, f, indent=2)
            print(f"{green('OK')} Exported to {args.export}")


if __name__ == '__main__':
    main()
