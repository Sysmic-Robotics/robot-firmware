#!/usr/bin/env python3
"""
CSV Analyzer for STM32Monitor logs — Sysmic Robotics.

Analiza logs de robot capturados con STM32Monitor, detecta escalones
en la señal de comando y calcula métricas de respuesta temporal.

Usage:
    # Análisis simple
    python3 csv_analyzer.py Log_myVariables_2026-06-03_19h17m00s.csv

    # Comparar dos archivos
    python3 csv_analyzer.py log1.csv log2.csv --compare

    # Especificar columna de comando
    python3 csv_analyzer.py log.csv --cmd v_vel[2]

    # Exportar métricas a JSON
    python3 csv_analyzer.py log.csv --export results.json

    # Analizar todos los escalones (no solo el primero)
    python3 csv_analyzer.py log.csv --all-steps

    # Sin plots (solo texto)
    python3 csv_analyzer.py log.csv --no-plot
"""
import csv
import json
import math
import os
import sys
from collections import defaultdict
from datetime import datetime

try:
    import numpy as np
except ImportError:
    np = None

try:
    from matplotlib import pyplot as plt
    from matplotlib import gridspec
except ImportError:
    plt = None


# ── Constants ──────────────────────────────────────────────────────────────
ANALOG_MAX = 300       # max DAC value (ffVoltage saturation point)
ENCODER_MAX = 4096     # encoder counts per rev (approximate)
DEFAULT_SETTLE_BAND = 0.02  # 2% band for settling time
DEFAULT_HYSTERESIS_FRAC = 0.01  # 1% of range for step detection
MIN_SAMPLES = 20       # minimum samples per segment


# ── ANSI helpers ───────────────────────────────────────────────────────────
def _supports_color():
    """Detect if stdout supports ANSI color codes."""
    if not sys.stdout.isatty():
        return False
    if os.name == 'nt':
        import platform
        return platform.release() >= '10'
    return True

_USE_COLOR = _supports_color()

def _c(code, text):
    """Wrap *text* in ANSI escape *code* if color enabled."""
    if not _USE_COLOR:
        return text
    return f'\033[{code}m{text}\033[0m'

def green(t):  return _c('32', t)
def yellow(t): return _c('33', t)
def red(t):    return _c('31', t)
def cyan(t):   return _c('36', t)
def bold(t):   return _c('1', t)
def dim(t):    return _c('2', t)


# ── 1. LOADER ──────────────────────────────────────────────────────────────
def load_csv(path):
    """
    Parse a STM32Monitor CSV file.

    Returns (headers, data_dict, time_col_idx) where:
        headers: list of column names (strings)
        data_dict: {header: [(time, value), ...], ...}
        time_col_idx: index of the time column in the header

    Auto-detects the time column by looking for common names
    ("time", "timestamp", "t[s]", "t"). Falls back to column 0.
    """
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
                    col_data[h].append((t, float('nan')))

    return headers, col_data, time_col


# ── 2. STEP DETECTION ──────────────────────────────────────────────────────
def find_steps(t, v, min_samples=MIN_SAMPLES, hysteresis=None):
    """
    Detect step transitions in a command signal.

    A step is registered when |v[i] - prev_val| exceeds *hysteresis*.
    When *hysteresis* is None, it auto-computes as 1% of the signal range
    (minimum 1e-6).

    Returns [(t_start, t_end, val_before, val_after), ...].
    """
    if len(v) < 2:
        return []

    if hysteresis is None:
        rng = max(v) - min(v)
        hysteresis = max(rng * DEFAULT_HYSTERESIS_FRAC, 1e-6)

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


def find_steps_clean(t, v, min_samples=MIN_SAMPLES, hysteresis=None):
    """
    Like find_steps but filters out segments where the "step" is actually
    just returning to a previous baseline (these are not new commands).
    This gives cleaner step detection for multi-step analysis.
    """
    raw = find_steps(t, v, min_samples, hysteresis)
    if not raw:
        return raw
    # Keep only segments where value changes to a *new* level
    clean = [raw[0]]
    for i in range(1, len(raw)):
        # Keep if value is significantly different from last kept segment's after-value
        last_val = clean[-1][3]
        _, _, _, curr_val = raw[i]
        if abs(curr_val - last_val) > 1e-4:
            clean.append(raw[i])
    return clean


# ── 3. METRICS ─────────────────────────────────────────────────────────────
def step_response_metrics(t, cmd, meas, step_start, step_end,
                          cmd_before, cmd_after, settle_band=DEFAULT_SETTLE_BAND):
    """
    Compute step response metrics for a single step transition.

    Args:
        t: time array for measurement signal
        cmd: command array (full signal)
        meas: measurement array
        step_start, step_end: time bounds of the step response window
        cmd_before, cmd_after: command values before / after the step
        settle_band: fractional band for settling time (default 2%)

    Returns dict with keys:
        cmd, cmd_before, ss_meas, gain, error,
        rise_time, peak_time, peak_value, overshoot_pct,
        settle_time_2pct, settle_time_5pct, tau_decay
    """
    # Find indices
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
    if i_start is None or i_end is None or i_end <= i_start:
        return None

    t_seg = t[i_start:i_end]
    m_seg = meas[i_start:i_end]
    delta = cmd_after - cmd_before

    if abs(delta) < 1e-6:
        return None

    # Steady-state: last 30% of segment
    ss_start = int(len(m_seg) * 0.7)
    if ss_start >= len(m_seg):
        ss_start = len(m_seg) // 2
    ss_meas = (sum(m_seg[ss_start:]) / len(m_seg[ss_start:])
               if len(m_seg) > ss_start else m_seg[-1])

    gain = (ss_meas - cmd_before) / delta if abs(delta) > 1e-6 else 0.0
    error = cmd_after - ss_meas
    total_change = abs(ss_meas - cmd_before)

    # Rise time: 63% of steady-state change
    target_change = (ss_meas - cmd_before) * 0.63
    target_val = cmd_before + target_change
    rise_time = None
    for i in range(len(m_seg)):
        if (delta > 0 and m_seg[i] >= target_val) or \
           (delta < 0 and m_seg[i] <= target_val):
            rise_time = t_seg[i] - t_seg[0]
            break

    # Peak time & overshoot
    if delta > 0:
        peak_idx = max(range(len(m_seg)), key=lambda i: m_seg[i])
    else:
        peak_idx = min(range(len(m_seg)), key=lambda i: m_seg[i])
    peak_value = m_seg[peak_idx]
    peak_time = t_seg[peak_idx] - t_seg[0]

    if total_change > 1e-6:
        if delta > 0:
            overshoot_pct = max(0.0, (peak_value - ss_meas) / total_change * 100)
        else:
            overshoot_pct = max(0.0, (ss_meas - peak_value) / total_change * 100)
    else:
        overshoot_pct = 0.0

    # Settling time: reverse scan — find last excursion outside band
    def _settle_time(band_frac):
        """Compute settling time for a given fractional band."""
        band = band_frac * total_change if total_change > 1e-9 else 0.0
        if band <= 1e-9:
            return None
        in_band = any(abs(v - ss_meas) <= band for v in m_seg)
        if not in_band:
            return None
        for i in range(len(m_seg) - 1, -1, -1):
            if abs(m_seg[i] - ss_meas) > band:
                idx = min(i + 1, len(m_seg) - 1)
                return max(0.0, t_seg[idx] - t_seg[0])
        return None

    settle_time_2pct = _settle_time(settle_band)
    settle_time_5pct = _settle_time(0.05)

    # Decay time constant: response to step returning to baseline
    tau_decay = None
    if i_end < len(meas):
        decay_meas = meas[i_end:min(i_end + 500, len(meas))]
        decay_t = t[i_end:min(i_end + 500, len(meas))]
        base = meas[i_start] if abs(cmd_before) < 1e-6 else cmd_before
        decay_target = base + (ss_meas - base) * 0.37
        for i in range(len(decay_meas)):
            if (delta > 0 and decay_meas[i] <= decay_target) or \
               (delta < 0 and decay_meas[i] >= decay_target):
                tau_decay = decay_t[i] - decay_t[0]
                break

    # Clipping & noise detection
    clipped = _detect_clipping(m_seg, cmd_before, cmd_after)
    noisy = _detect_noisy(m_seg, ss_start)

    return {
        'cmd': cmd_after,
        'cmd_before': cmd_before,
        'ss_meas': ss_meas,
        'gain': gain,
        'error': error,
        'rise_time': rise_time,
        'peak_time': peak_time,
        'peak_value': peak_value,
        'overshoot_pct': overshoot_pct,
        'settle_time_2pct': settle_time_2pct,
        'settle_time_5pct': settle_time_5pct,
        'tau_decay': tau_decay,
        'clipped': clipped,
        'noisy': noisy,
    }


def _detect_clipping(segment, cmd_before, cmd_after):
    """
    Heuristic: if response is stuck near a boundary (0 or ANALOG_MAX)
    while command is non-zero, the signal is likely clipped/saturated.
    """
    if abs(cmd_after - cmd_before) < 1e-6:
        return False
    # Check if steady-state portion sits at boundary
    n = len(segment)
    if n < 5:
        return False
    tail = segment[int(n * 0.7):]
    if not tail:
        return False
    mean_tail = sum(tail) / len(tail)
    return abs(mean_tail) < 0.01 or abs(mean_tail - ANALOG_MAX) < 0.01


def _detect_noisy(segment, ss_start_idx):
    """
    Heuristic: if the noise in the steady-state portion exceeds 5%
    of the total change, flag as noisy.
    """
    n = len(segment)
    if n < 10 or ss_start_idx >= n:
        return False
    tail = segment[ss_start_idx:]
    if len(tail) < 5:
        return False
    mean_tail = sum(tail) / len(tail)
    variance = sum((v - mean_tail) ** 2 for v in tail) / len(tail)
    std_tail = math.sqrt(variance)
    if abs(mean_tail) > 1e-6:
        return std_tail / abs(mean_tail) > 0.05
    return False


def latency_analysis(t_cmd, t_meas, cmd, meas, step_start):
    """
    Estimate command-to-response latency.

    Finds the time from the command edge to the first measurable
    sensor response (first crossing of 5% of the total change).
    """
    try:
        i_cmd = next(i for i, tt in enumerate(t_cmd) if tt >= step_start)
    except StopIteration:
        return None
    try:
        i_meas = next(i for i, tt in enumerate(t_meas) if tt >= step_start)
    except StopIteration:
        return None

    look_ahead = meas[i_meas:min(i_meas + 100, len(meas))]
    if len(look_ahead) < 5:
        return None
    baseline = sum(look_ahead[:min(10, len(look_ahead))]) / min(10, len(look_ahead))
    max_change = max(abs(v - baseline) for v in look_ahead) if look_ahead else 0.0
    if max_change < 1e-6:
        return None
    threshold = max_change * 0.05
    for i, v in enumerate(look_ahead):
        if abs(v - baseline) >= threshold:
            return t_meas[i_meas + i] - t_cmd[i_cmd]
    return None


# ── 4. PLOTTING ────────────────────────────────────────────────────────────
def plot_file(path, cmd_col=None, fig=None, subplot_spec=None, label_suffix="",
              all_steps=False):
    """
    Plot CSV data with matplotlib.

    If *fig* is None, creates a new figure.  Returns
    (fig, data, cmd_col, steps, t0).
    """
    if plt is None:
        print("matplotlib not installed. Install with: pip install matplotlib")
        return None, None, None, None, None

    headers, data, time_col_idx = load_csv(path)
    fname = os.path.splitext(os.path.basename(path))[0]
    data_cols = [h for h in headers if h != headers[time_col_idx]]

    # auto-detect command column
    if cmd_col is None:
        possible_cmd = [h for h in data_cols if any(k in h.lower() for k in
                        ('v_vel', 'cmd', 'ref', 'setpoint', 'target', 'command'))]
        cmd_col = possible_cmd[0] if possible_cmd else data_cols[0]

    n_cols = len(data_cols)
    figsize = (14, max(4, 1.5 * n_cols))

    if fig is None:
        fig = plt.figure(figsize=figsize)
        gs = gridspec.GridSpec(n_cols + 1, 1, height_ratios=[3] + [1] * n_cols,
                               hspace=0.35)
    else:
        if subplot_spec is not None:
            import matplotlib.gridspec as mgs
            inner = mgs.GridSpecFromSubplotSpec(
                n_cols + 1, 1, subplot_spec=subplot_spec,
                height_ratios=[3] + [1] * n_cols, hspace=0.35)
            gs = inner
        else:
            gs = gridspec.GridSpec(n_cols + 1, 1, height_ratios=[3] + [1] * n_cols,
                                   hspace=0.35)

    # Top: overlaid signals
    ax0 = fig.add_subplot(gs[0])
    ccolors = plt.cm.Set1(np.linspace(0, 1, n_cols)) if np else \
              plt.cm.rainbow([i / n_cols for i in range(n_cols)])
    for j, h in enumerate(data_cols):
        arr = np.array(data[h])
        t_arr = arr[:, 0] - arr[0, 0]
        v_arr = arr[:, 1]
        ls = '-' if h == cmd_col else '--'
        lw = 2.5 if h == cmd_col else 1.2
        aa = 1.0 if h == cmd_col else 0.8
        ax0.plot(t_arr, v_arr, label=f'{h}{label_suffix}',
                 linestyle=ls, linewidth=lw, alpha=aa)

    cmd_arr = np.array(data[cmd_col])
    t0 = cmd_arr[0, 0]
    steps = find_steps_clean(cmd_arr[:, 0], cmd_arr[:, 1]) if all_steps else \
            find_steps(cmd_arr[:, 0], cmd_arr[:, 1])

    for s in steps:
        ax0.axvline(s[0] - t0, color='gray', linestyle=':', alpha=0.5)
        ax0.axvline(s[1] - t0, color='gray', linestyle=':', alpha=0.3)

    ax0.set_ylabel('Value')
    ax0.set_title(f'{fname}', fontsize=11)
    ax0.legend(fontsize=8, ncol=min(3, max(2, n_cols)))
    ax0.grid(True, alpha=0.3)

    # Per-signal subplots
    for j, h in enumerate(data_cols):
        ax = fig.add_subplot(gs[j + 1])
        arr = np.array(data[h])
        t_arr = arr[:, 0] - t0
        v_arr = arr[:, 1]
        color = f'C{j}'
        ax.plot(t_arr, v_arr, color=color, linewidth=1.0)
        ax.set_ylabel(h, fontsize=8)
        ax.grid(True, alpha=0.3)
        if j == n_cols - 1:
            ax.set_xlabel('Time (s)')
        ax.tick_params(labelsize=8)

    return fig, data, cmd_col, steps, t0


# ── 5. REPORT ──────────────────────────────────────────────────────────────
def _fmt_ms(val):
    """Format value in seconds to a human-readable string (ms or s)."""
    if val is None:
        return dim('—')
    if val < 1.0:
        return f'{val * 1000:.0f}ms'
    return f'{val:.3f}s'


def _fmt_gain(g):
    """Color-code gain value."""
    text = f'{g:.3f}'
    if 0.9 <= g <= 1.1:
        return green(text)
    if 0.5 <= g <= 1.5:
        return yellow(text)
    return red(text)


def _fmt_overshoot(os_pct):
    """Color-code overshoot percentage."""
    text = f'{os_pct:.1f}%'
    if os_pct <= 5:
        return green(text)
    if os_pct <= 20:
        return yellow(text)
    return red(text)


def _fmt_ok(val, threshold=0.1):
    """Color a value green if within threshold, yellow otherwise."""
    text = f'{val:.3f}'
    if abs(val) <= threshold:
        return green(text)
    return yellow(text)


def _format_metric_row(m, m_col):
    """
    Format a single metric row for a measurement column.

    *m* can be either the raw dict from step_response_metrics (keys:
    'error', 'rise_time', etc.) or the processed dict from analyze_csv
    (keys: 'ss_error', 'rise_time_ms', etc.).  We handle both.
    """
    if m is None:
        return dim(f'    {m_col:20s}  (no step data)'), None

    lat = m.get('latency_ms')
    lat_str = f'  lat={lat:.0f}ms' if lat is not None else ''

    # Normalise keys (handle both raw and processed dicts)
    # Helper: fetch value, converting processed (ms) -> seconds
    def _v_sec(pkey, rkey, default=None):
        val = m.get(pkey)
        if val is not None:
            return val / 1000.0  # processed dict stores ms
        return m.get(rkey, default)  # raw dict stores seconds

    def _v(pkey, rkey, default=None):
        return m.get(pkey, m.get(rkey, default))

    gain = _v('gain', 'gain', 0.0)
    ss_meas = _v('ss_meas', 'ss_meas', 0.0)
    ss_err = _v('ss_error', 'error', 0.0)
    rise = _v_sec('rise_time_ms', 'rise_time', default=None)
    overshoot = _v('overshoot_pct', 'overshoot_pct', 0.0)
    peak_t = _v_sec('peak_time_ms', 'peak_time', default=None)
    settle2 = _v_sec('settle_time_2pct_ms', 'settle_time_2pct', default=None)
    settle5 = _v_sec('settle_time_5pct_ms', 'settle_time_5pct', default=None)

    parts = [
        f'  gain={_fmt_gain(gain)}',
        f'  ss={_fmt_ok(ss_meas, 0.5)}',
        f'  err={_fmt_ok(ss_err, 0.5)}',
        f'  {_fmt_ms(rise)}',
    ]
    if lat_str:
        parts.append(lat_str)
    parts.append(f'  OS={_fmt_overshoot(overshoot)}')
    parts.append(f'  Tp={_fmt_ms(peak_t)}')
    parts.append(f'  Ts2={_fmt_ms(settle2)}')
    parts.append(f'  Ts5={_fmt_ms(settle5)}')

    flags = []
    if m.get('clipped'):
        flags.append(red('CLIPPED'))
    if m.get('noisy'):
        flags.append(yellow('NOISY'))
    if flags:
        parts.append('  \u26a0 ' + ' '.join(flags))

    text = f'    {m_col:20s}' + ' '.join(parts)

    return text, None


def analyze_csv(path, cmd_col=None, all_steps=False):
    """
    Full analysis pipeline for a single CSV file.

    Returns a dict with the full analysis results (JSON-serializable).
    """
    headers, data, _ = load_csv(path)
    data_cols = [h for h in headers]

    time_col_name = 'time'
    time_data = data.get(time_col_name, next(iter(data.values())))
    t0 = time_data[0][0]
    duration = time_data[-1][0] - time_data[0][0]

    if cmd_col is None:
        possible_cmd = [h for h in data_cols if any(k in h.lower() for k in
                        ('v_vel', 'cmd', 'ref', 'setpoint', 'target', 'command'))]
        cmd_col = possible_cmd[0] if possible_cmd else data_cols[1]

    t_cmd = [x[0] for x in data[cmd_col]]
    v_cmd = [x[1] for x in data[cmd_col]]

    if all_steps:
        steps = find_steps_clean(t_cmd, v_cmd)
    else:
        steps = find_steps(t_cmd, v_cmd)

    result = {
        'file': os.path.basename(path),
        'duration_s': round(duration, 3),
        'num_samples': len(time_data),
        'columns': [c for c in data_cols if c != time_col_name],
        'command_column': cmd_col,
        'steps': [],
    }

    if len(steps) < 2:
        result['warning'] = 'No step commands detected (need ≥2 distinct values).'
        return result

    measure_cols = [c for c in data_cols if c not in (cmd_col, time_col_name)]

    for si in range(len(steps) - 1):
        _, s_end, val_before, val_after = steps[si]
        _, s_next, _, _ = steps[si + 1]
        step_start = s_end
        step_end = s_next

        step_info = {
            'step': si + 1,
            'from': val_before,
            'to': val_after,
            'duration_s': round(step_end - step_start, 4),
            'measurements': {},
        }

        for m_col in measure_cols:
            t_meas = [x[0] for x in data[m_col]]
            v_meas = [x[1] for x in data[m_col]]
            m = step_response_metrics(t_meas, t_cmd if np else v_cmd,
                                       v_meas, step_start, step_end,
                                       val_before, val_after)
            if m:
                lat = latency_analysis(t_cmd, t_meas, v_cmd, v_meas, step_start)
                m['latency_ms'] = lat * 1000 if lat is not None else None

                step_info['measurements'][m_col] = {
                    'gain': m['gain'],
                    'ss_error': m['error'],
                    'ss_meas': m['ss_meas'],
                    'rise_time_ms': m['rise_time'] * 1000 if m['rise_time'] is not None else None,
                    'overshoot_pct': m['overshoot_pct'],
                    'peak_time_ms': m['peak_time'] * 1000 if m['peak_time'] is not None else None,
                    'peak_value': m['peak_value'],
                    'settle_time_2pct_ms': m['settle_time_2pct'] * 1000 if m['settle_time_2pct'] is not None else None,
                    'settle_time_5pct_ms': m['settle_time_5pct'] * 1000 if m['settle_time_5pct'] is not None else None,
                    'tau_decay_ms': m['tau_decay'] * 1000 if m['tau_decay'] is not None else None,
                    'latency_ms': m.get('latency_ms'),
                    'clipped': m.get('clipped', False),
                    'noisy': m.get('noisy', False),
                }

        result['steps'].append(step_info)

    return result


def print_report(path, cmd_col=None, all_steps=False):
    """
    Analyze and print a formatted report to stdout.
    Returns the analysis dict (for programmatic use / export).
    """
    result = analyze_csv(path, cmd_col, all_steps)

    print(f"\n{'=' * 70}")
    print(f"  {bold(os.path.basename(path))}")
    print(f"  Duration: {result['duration_s']:.1f}s  |  "
          f"Samples: {result['num_samples']}  |  "
          f"Columns: {len(result['columns'])}")
    print(f"  Command col: {cyan(result['command_column'])}")
    print(f"{'=' * 70}")

    if 'warning' in result:
        print(f"  {yellow(result['warning'])}")
        return result

    steps = result['steps']
    for step_info in steps:
        s = step_info
        step_label = f'Step {s["step"]}: {s["from"]:.1f} → {s["to"]:.1f}'
        dur_label = f'{s["duration_s"]:.3f}s'
        print(f'\n  {bold(f"── {step_label}")}  '
              f'({dim(dur_label)}) ──')

        for m_col, m in s['measurements'].items():
            row_text, _ = _format_metric_row(m, m_col)
            print(row_text)

    return result


# ── 6. COMPARISON ──────────────────────────────────────────────────────────
def plot_comparison(files, cmd_col=None, all_steps=False):
    """
    Enhanced comparison plot: grid with overlay on top and per-signal panels.

    Each row after the first shows one measurement signal from all files.
    """
    if plt is None:
        print("Cannot plot: matplotlib not installed.")
        return

    # Load all files first to determine common columns
    file_data = []
    all_meas_cols = set()
    for f in files:
        headers, data, _ = load_csv(f)
        data_cols = [h for h in headers]
        if cmd_col is None:
            possible = [h for h in data_cols if any(k in h.lower() for k in
                        ('v_vel', 'cmd', 'ref', 'setpoint', 'target', 'command'))]
            ccol = possible[0] if possible else data_cols[1]
        else:
            ccol = cmd_col
        meas_cols = [h for h in data_cols if h not in (ccol, headers[0])]
        all_meas_cols.update(meas_cols)
        file_data.append((f, data, ccol, meas_cols))

    all_meas_cols = sorted(all_meas_cols)
    n_files = len(files)
    n_meas = len(all_meas_cols)

    # Grid: 1 overlay row + n_meas per-signal rows
    fig = plt.figure(figsize=(14, 3 + 2 * n_meas))
    gs = gridspec.GridSpec(n_meas + 1, 1, height_ratios=[3] + [2] * n_meas,
                           hspace=0.4)

    colors = ['C0', 'C1', 'C2', 'C3', 'C4', 'C5']

    # Top: overlay with command + first measurement
    ax0 = fig.add_subplot(gs[0])
    for i, (f, data, ccol, meas_cols) in enumerate(file_data):
        fname = os.path.splitext(os.path.basename(f))[0]
        # Plot command
        cmd_arr = np.array(data[ccol])
        t_cmd = cmd_arr[:, 0] - cmd_arr[0, 0]
        ax0.plot(t_cmd, cmd_arr[:, 1], color=colors[i % len(colors)],
                 linestyle='--', alpha=0.6, linewidth=1.5,
                 label=f'{fname}: {ccol}')
        # Plot first measurement
        if meas_cols:
            m_col = meas_cols[0]
            arr = np.array(data[m_col])
            t_m = arr[:, 0] - arr[0, 0]
            ax0.plot(t_m, arr[:, 1], color=colors[i % len(colors)],
                     linestyle='-', alpha=0.9, linewidth=1.2,
                     label=f'{fname}: {m_col}')
    ax0.set_title('Comparison')
    ax0.legend(fontsize=7, ncol=min(2, n_files))
    ax0.grid(True, alpha=0.3)

    # Per-measurement rows
    for j, m_col in enumerate(all_meas_cols):
        ax = fig.add_subplot(gs[j + 1])
        for i, (f, data, ccol, meas_cols) in enumerate(file_data):
            fname = os.path.splitext(os.path.basename(f))[0]
            if m_col in data:
                arr = np.array(data[m_col])
                t_m = arr[:, 0] - arr[0, 0]
                ax.plot(t_m, arr[:, 1], color=colors[i % len(colors)],
                        label=f'{fname}', linewidth=1.0, alpha=0.8)
        ax.set_ylabel(m_col, fontsize=8)
        ax.legend(fontsize=6)
        ax.grid(True, alpha=0.3)
        if j == n_meas - 1:
            ax.set_xlabel('Time (s)')
        ax.tick_params(labelsize=8)

    plt.tight_layout()
    plt.show()


# ── 7. MAIN ────────────────────────────────────────────────────────────────
def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='Analyze STM32Monitor CSV logs — Sysmic Robotics',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)

    parser.add_argument('files', nargs='+', help='CSV files to analyze')
    parser.add_argument('--cmd', '-c', help='Command column name (auto-detect)')
    parser.add_argument('--no-plot', '-n', action='store_true',
                        help='Skip plotting')
    parser.add_argument('--compare', action='store_true',
                        help='Overlay plots for comparison')
    parser.add_argument('--all-steps', '-a', action='store_true',
                        help='Analyze all step transitions (not just first)')
    parser.add_argument('--export', '-e', metavar='FILE.json',
                        help='Export metrics to JSON file')
    args = parser.parse_args()

    if not args.no_plot and plt is None:
        print("Warning: matplotlib not installed. Install with: pip install matplotlib")
        args.no_plot = True

    all_results = []

    for f in args.files:
        if not os.path.exists(f):
            print(f"File not found: {f}")
            continue

        result = print_report(f, args.cmd, all_steps=args.all_steps)
        all_results.append(result)

        if not args.no_plot and not args.compare:
            plot_file(f, args.cmd, all_steps=args.all_steps)
            plt.tight_layout()

    # Enhanced comparison
    if not args.no_plot and args.compare and len(args.files) >= 2:
        plot_comparison(args.files, args.cmd, all_steps=args.all_steps)

    # Export
    if args.export and all_results:
        with open(args.export, 'w') as f:
            json.dump(all_results if len(all_results) > 1 else all_results[0],
                      f, indent=2)
        print(f"\n{green('✓')} Metrics exported to {args.export}")

    if not args.no_plot and not args.compare:
        plt.show()


if __name__ == '__main__':
    main()
