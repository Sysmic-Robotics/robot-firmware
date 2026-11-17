#!/usr/bin/env python3
"""
Sensor Compare — Sysmic Robotics.

Compares two CSV data sources (e.g. telemetry @1kHz vs camera @60Hz)
by interpolating the lower-rate signal onto the higher-rate timeline
and computing RMS error, cross-correlation delay, bias, and per-sample
difference statistics.

Supports variable name mapping between sources (e.g. vx_est ↔ vx).

Usage:
    # Compare specific variable (exact name match)
    python3 sensor_compare.py telemetry.csv camera.csv --var vth_est

    # Map names between sources: robot vx_est ↔ camera vx
    python3 sensor_compare.py telemetry.csv camera.csv --var vx_est --map vx_est=vx

    # Multiple mapped variables
    python3 sensor_compare.py telemetry.csv camera.csv --map vx_est=vx vy_est=vy

    # Auto-detect all with fuzzy matching (vx_est ↔ vx, etc.)
    python3 sensor_compare.py telemetry.csv camera.csv --all

    # Derive angular velocity from camera theta
    python3 sensor_compare.py telemetry.csv camera.csv --map vth_est=vth --derive-vth

    # Export results
    python3 sensor_compare.py telemetry.csv camera.csv --map vx_est=vx --export comp.json
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
    print("numpy is required. Install with: pip install numpy")
    sys.exit(1)

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


# ── 1. CSV LOADER ──────────────────────────────────────────────────────────
def load_csv(path, time_col_name=None):
    """Load CSV, return (headers, {name: np.array[(t, val)]}, time_col_name)."""
    with open(path) as f:
        reader = csv.reader(f)
        raw_header = next(reader)
    headers = [h.strip() for h in raw_header if h.strip()]

    if time_col_name is None:
        for h in headers:
            if h.lower() in ("time", "timestamp", "t[s]", "t"):
                time_col_name = h
                break
    if time_col_name is None:
        time_col_name = headers[0]

    col_data = defaultdict(list)
    with open(path) as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            if len(row) < len(headers):
                continue
            try:
                t = float(row[headers.index(time_col_name)])
            except (ValueError, IndexError):
                continue
            for h in headers:
                if h == time_col_name:
                    continue
                try:
                    col_data[h].append((t, float(row[headers.index(h)])))
                except (ValueError, IndexError):
                    pass

    data = {}
    for h, vals in col_data.items():
        if vals:
            data[h] = np.array(vals)
    return headers, data, time_col_name


# ── 2. DERIVE VTH FROM THETA ──────────────────────────────────────────────
def derive_vth_from_theta(data, theta_col='theta', vth_col='vth'):
    """Compute angular velocity from theta using finite differences.

    Filters out spikes from duplicate/jittered timestamps by clamping
    the minimum dt to 1 ms and applying a simple moving average (3-tap).
    """
    if theta_col not in data:
        return False
    arr = data[theta_col]
    t = arr[:, 0]
    th = arr[:, 1]

    dt_raw = np.diff(t)
    dt_safe = np.maximum(dt_raw, 0.001)  # clamp to 1ms min
    dth = np.diff(th)
    omega = dth / dt_safe

    # Pad first sample
    dt0 = max(t[1] - t[0], 0.001) if len(t) > 1 else 0.001
    omega_pad = np.concatenate([[dth[0] / dt0], omega])

    # 3-tap moving average to smooth jitter
    kernel = np.array([0.25, 0.5, 0.25])
    omega_smooth = np.convolve(omega_pad, kernel, mode='same')
    omega_smooth[0] = omega_pad[0]
    omega_smooth[-1] = omega_pad[-1]

    data[vth_col] = np.column_stack((t, omega_smooth))
    return True


# ── 3. VARIABLE MATCHING ───────────────────────────────────────────────────
_SUFFIXES = ('_est', '_enc', '_filt', '_raw', '_ms2', '_dps', '_rads')
_PREFIXES = ('cmd_', 'v_', 'v_vel')

def _strip_name(name):
    """Strip common suffixes/prefixes to get base variable name."""
    n = name.lower()
    for s in _SUFFIXES:
        if n.endswith(s):
            base = n[:-len(s)]
            return base, name[:-len(s)]
    for p in _PREFIXES:
        if n.startswith(p):
            base = n[len(p):]
            return base, name[len(p):]
    return n, name


def fuzzy_match(data_a, data_b):
    """
    Find matching variables with name normalization.
    Returns list of (name_a, name_b) tuples.
    """
    keys_a = list(data_a.keys())
    keys_b = list(data_b.keys())

    # Build index by base name
    base_to_a = defaultdict(list)
    for k in keys_a:
        base, _ = _strip_name(k)
        base_to_a[base].append(k)
    base_to_b = defaultdict(list)
    for k in keys_b:
        base, _ = _strip_name(k)
        base_to_b[base].append(k)

    matched = []
    used_a = set()
    used_b = set()

    skip = {'time', 'timestamp', 't[s]', 't', 'x', 'y', 'theta', 'cmd_angular'}

    for base in sorted(base_to_a):
        if base in skip:
            continue
        if base in base_to_b:
            candidates_a = [k for k in base_to_a[base] if k not in used_a]
            candidates_b = [k for k in base_to_b[base] if k not in used_b]
            if candidates_a and candidates_b:
                # Prefer _est for A, unadorned for B
                pick_a = candidates_a[0]
                pick_b = candidates_b[0]
                # If A has _est and B has bare name, use that
                est_a = [k for k in candidates_a if k.endswith('_est')]
                bare_b = [k for k in candidates_b
                          if not any(k.endswith(s) for s in _SUFFIXES)]
                if est_a and bare_b:
                    pick_a = est_a[0]
                    pick_b = bare_b[0]
                matched.append((pick_a, pick_b))
                used_a.add(pick_a)
                used_b.add(pick_b)

    return matched


def match_variables(data_a, data_b, var_names=None, var_map=None, fuzzy=False):
    """
    Find matching variables between two datasets.

    var_map: dict of {name_in_a: name_in_b}
    var_names: list — names to find (exact + map lookup)
    fuzzy: bool — use fuzzy matching

    Returns list of (name_a, name_b) tuples.
    """
    matched = []

    # User-provided explicit map
    if var_map:
        for name_a, name_b in var_map.items():
            if name_a in data_a and name_b in data_b:
                matched.append((name_a, name_b))
            else:
                missing = []
                if name_a not in data_a:
                    missing.append(f"A missing '{name_a}'")
                if name_b not in data_b:
                    missing.append(f"B missing '{name_b}'")
                print(f"  {yellow('?')} Map {name_a}→{name_b}: {', '.join(missing)}")

    # Specific variable names — try exact, then map reverse, then fuzzy
    if var_names:
        # Build reverse map (name_b → name_a)
        rev_map = {v: k for k, v in (var_map or {}).items()}
        for v in var_names:
            # Already handled by map?
            if var_map and v in var_map:
                continue
            if rev_map and v in rev_map:
                a_name, b_name = rev_map[v], v
                if (a_name, b_name) not in matched:
                    matched.append((a_name, b_name))
                continue
            # Exact match
            if v in data_a and v in data_b:
                if (v, v) not in matched:
                    matched.append((v, v))
                continue
            # Case-insensitive
            match_a = [k for k in data_a if k.lower() == v.lower()]
            match_b = [k for k in data_b if k.lower() == v.lower()]
            if match_a and match_b:
                if (match_a[0], match_b[0]) not in matched:
                    matched.append((match_a[0], match_b[0]))
                continue
            print(f"  {yellow('?')} Variable '{v}' not found")

    # Fuzzy auto-detect
    if fuzzy and not var_names and not var_map:
        matched = fuzzy_match(data_a, data_b)

    return matched


# ── 4. STATISTICS ──────────────────────────────────────────────────────────
def compute_stats(t_a, v_a, t_b, v_b):
    """Compute comparison statistics between two signals."""
    rate_a = len(t_a) / (t_a[-1] - t_a[0]) if t_a[-1] > t_a[0] else 0
    rate_b = len(t_b) / (t_b[-1] - t_b[0]) if t_b[-1] > t_b[0] else 0

    if rate_a >= rate_b:
        t_high, v_high = t_a, v_a
        t_low, v_low = t_b, v_b
        label_high, label_low = 'A', 'B'
    else:
        t_high, v_high = t_b, v_b
        t_low, v_low = t_a, v_a
        label_high, label_low = 'B', 'A'

    t_min = max(t_high[0], t_low[0])
    t_max = min(t_high[-1], t_low[-1])

    if t_max <= t_min:
        return {'_error': 'No overlapping time range'}

    mask_high = (t_high >= t_min) & (t_high <= t_max)
    t_common = t_high[mask_high]
    v_high_common = v_high[mask_high]
    v_low_interp = np.interp(t_common, t_low, v_low)

    error = v_high_common - v_low_interp
    rms_error = math.sqrt(np.mean(error ** 2))
    mean_error = float(np.mean(error))
    std_error = float(np.std(error))
    max_abs_error = float(np.max(np.abs(error)))

    corr_matrix = np.corrcoef(v_high_common, v_low_interp)
    correlation = float(corr_matrix[0, 1]) if not np.isnan(corr_matrix[0, 1]) else 0.0

    n = len(v_high_common)
    max_lag = min(500, n // 4)
    delay_samples = 0
    delay_seconds = 0.0
    if max_lag > 10 and n > 100 and np.std(v_high_common) > 1e-9 and np.std(v_low_interp) > 1e-9:
        v1 = (v_high_common - np.mean(v_high_common)) / np.std(v_high_common)
        v2 = (v_low_interp - np.mean(v_low_interp)) / np.std(v_low_interp)
        cross_corr = np.correlate(v1, v2, mode='same')
        mid = len(cross_corr) // 2
        half = min(max_lag, mid)
        lag_range = np.arange(-half, half + 1)
        peak_idx = np.argmax(np.abs(cross_corr[mid - half:mid + half + 1]))
        delay_samples = int(lag_range[peak_idx])
        dt = (t_common[-1] - t_common[0]) / len(t_common) if len(t_common) > 1 else 0
        delay_seconds = delay_samples * dt

        # Recompute stats after applying the recovered delay so correlation /
        # RMS / bias reflect the time-aligned signals (raw ones are shifted).
        if delay_seconds != 0.0:
            t_align = t_common - delay_seconds
            valid = (t_align >= t_low[0]) & (t_align <= t_low[-1])
            if valid.sum() > 10:
                v_low_aligned = np.interp(t_align[valid], t_low, v_low)
                v_high_al = v_high_common[valid]
                error = v_high_al - v_low_aligned
                rms_error = math.sqrt(np.mean(error ** 2))
                mean_error = float(np.mean(error))
                std_error = float(np.std(error))
                max_abs_error = float(np.max(np.abs(error)))
                corr_matrix = np.corrcoef(v_high_al, v_low_aligned)
                correlation = float(corr_matrix[0, 1]) if not np.isnan(corr_matrix[0, 1]) else 0.0
                n = len(v_high_al)

    rate_high = len(t_high) / (t_high[-1] - t_high[0]) if len(t_high) > 1 else 0
    rate_low = len(t_low) / (t_low[-1] - t_low[0]) if len(t_low) > 1 else 0

    return {
        'rms_error': rms_error,
        'mean_error': mean_error,
        'std_error': std_error,
        'max_abs_error': max_abs_error,
        'correlation': correlation,
        'delay_samples': delay_samples,
        'delay_seconds': delay_seconds,
        'n_points': len(v_high_common),
        'rate_higher_hz': round(rate_high, 1),
        'rate_lower_hz': round(rate_low, 1),
        'overlap_duration_s': round(t_max - t_min, 3),
        'label_higher': label_high,
        'label_lower': label_low,
        't_common': t_common.tolist(),
        'error': error.tolist(),
        'v_high': v_high_common.tolist(),
        'v_low': v_low_interp.tolist(),
    }


# ── 5. REPORT ──────────────────────────────────────────────────────────────
def _fmt_rate(hz):
    if hz >= 1000:
        return f'{hz/1000:.1f}kHz'
    return f'{hz:.0f}Hz'


def print_stats(stats, var_a, var_b, file_a, file_b):
    """Print formatted comparison stats for one variable pair."""
    if '_error' in stats:
        print(f"  {red('!')} {var_a} vs {var_b}: {stats['_error']}")
        return

    name_a = os.path.basename(file_a)
    name_b = os.path.basename(file_b)

    rms   = stats.get('rms_error', 0)
    mean  = stats.get('mean_error', 0)
    std   = stats.get('std_error', 0)
    maxe  = stats.get('max_abs_error', 0)
    corr  = stats.get('correlation', 0)
    delay = stats.get('delay_seconds', 0)
    n     = stats.get('n_points', 0)
    overlap = stats.get('overlap_duration_s', 0)

    corr_color = green if corr > 0.95 else (yellow if corr > 0.8 else red)
    rms_color  = green if rms < 0.05 else (yellow if rms < 0.2 else red)

    print(f"\n  {bold(f'{var_a} ↔ {var_b}')}")
    print(f"    RMS error:      {rms_color(f'{rms:.4f}')}")
    print(f"    Mean error:     {mean:+.4f}  (bias)")
    print(f"    Std error:      {std:.4f}")
    print(f"    Max |error|:    {maxe:.4f}")
    print(f"    Correlation:    {corr_color(f'{corr:.6f}')}")
    print(f"    Delay:          {delay*1000:+.1f} ms")
    print(f"    Points:         {n}  |  overlap: {overlap:.2f}s")


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='Compare two CSV data sources — Sysmic Robotics',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)

    parser.add_argument('file_a', help='First CSV file (higher rate preferred)')
    parser.add_argument('file_b', help='Second CSV file (lower rate)')
    parser.add_argument('--var', '-v', nargs='+',
                        help='Variable name(s) to compare')
    parser.add_argument('--map', '-m', nargs='+', metavar='A=B',
                        help='Name mapping between sources (e.g. vx_est=vx)')
    parser.add_argument('--all', '-a', action='store_true',
                        help='Auto-detect all matching variables (fuzzy)')
    parser.add_argument('--derive-vth', action='store_true',
                        help='Derive angular velocity from theta in B')
    parser.add_argument('--time-col', '-t', help='Time column name (auto-detect)')
    parser.add_argument('--no-plot', '-n', action='store_true',
                        help='Skip plotting')
    parser.add_argument('--export', '-e', metavar='FILE.json',
                        help='Export results to JSON')
    args = parser.parse_args()

    if not os.path.exists(args.file_a):
        print(f"File not found: {args.file_a}")
        sys.exit(1)
    if not os.path.exists(args.file_b):
        print(f"File not found: {args.file_b}")
        sys.exit(1)

    if not args.no_plot and plt is None:
        print("Warning: matplotlib not installed. Install with: pip install matplotlib")
        args.no_plot = True

    var_map = {}
    if args.map:
        for pair in args.map:
            if '=' in pair:
                a_name, b_name = pair.split('=', 1)
                var_map[a_name.strip()] = b_name.strip()

    # Load CSVs
    headers_a, data_a, _ = load_csv(args.file_a, args.time_col)
    headers_b, data_b, _ = load_csv(args.file_b, args.time_col)

    # Derive vth from theta if requested
    if args.derive_vth:
        if derive_vth_from_theta(data_b):
            print(f"  {green('✓')} Derived 'vth' (angular velocity) from 'theta' in B")
        else:
            print(f"  {yellow('?')} Cannot derive vth: no 'theta' column in B")

    print(f"\n{'=' * 70}")
    print(f"  {bold('Sensor Compare — Sysmic Robotics')}")
    if data_a:
        n_a = len(data_a[next(iter(data_a))])
        print(f"  A: {os.path.basename(args.file_a)} ({n_a} samples)")
    else:
        print(f"  A: {os.path.basename(args.file_a)} (empty)")
    if data_b:
        n_b = len(data_b[next(iter(data_b))])
        print(f"  B: {os.path.basename(args.file_b)} ({n_b} samples)")
    else:
        print(f"  B: {os.path.basename(args.file_b)} (empty)")
    print(f"{'=' * 70}")

    # Available variables
    skip_names = {'time', 'timestamp', 't[s]', 't'}
    print(f"\n  {bold('Available variables')}:")
    for h in headers_a:
        if h.lower() not in skip_names:
            n = len(data_a.get(h, []))
            print(f"    A: {cyan(h):30s} ({n} samples)", end='')
            if h in data_b:
                m = len(data_b[h])
                print(f"  B: OK ({m} samples)")
            else:
                print(f"  {yellow('B: not found')}")
    for h in headers_b:
        if h.lower() not in skip_names and h not in data_a:
            n = len(data_b.get(h, []))
            extra = ' (derived)' if h == 'vth' and args.derive_vth else ''
            print(f"    B: {cyan(h):30s} ({n} samples{extra})  {'A: not found':>12s}")

    # Match variables
    var_pairs = []
    if args.all:
        var_pairs = match_variables(data_a, data_b, fuzzy=True)
        if var_map:
            for a_name, b_name in var_map.items():
                if a_name in data_a and b_name in data_b:
                    if (a_name, b_name) not in var_pairs:
                        var_pairs.append((a_name, b_name))
    else:
        var_pairs = match_variables(data_a, data_b, args.var, var_map)

    if not var_pairs:
        var_pairs = fuzzy_match(data_a, data_b)
        if var_pairs:
            print(f"\n  {yellow('No exact matches — using fuzzy matching')}:")
            for a, b in var_pairs:
                print(f"    {a} ↔ {b}")

    if not var_pairs:
        print(f"\n  {red('No matching variables found.')}")
        print(f"  Use --var or --map to specify, or --all for fuzzy matching.")
        sys.exit(1)

    print(f"\n  {bold(f'Comparing {len(var_pairs)} variable(s)')}: "
          f"{', '.join(f'{a}↔{b}' for a, b in var_pairs)}")

    all_results = {}
    for pair_a, pair_b in var_pairs:
        arr_a = data_a[pair_a]
        arr_b = data_b[pair_b]
        stats = compute_stats(arr_a[:, 0], arr_a[:, 1],
                              arr_b[:, 0], arr_b[:, 1])

        print_stats(stats, pair_a, pair_b, args.file_a, args.file_b)
        all_results[f'{pair_a}_vs_{pair_b}'] = {
            k: v for k, v in stats.items()
            if k not in ('t_common', 'error', 'v_high', 'v_low')
        }
        if not args.no_plot:
            plot_comparison(stats, pair_a, pair_b, args.file_a, args.file_b)

    if args.export and all_results:
        with open(args.export, 'w') as f:
            json.dump(all_results, f, indent=2)
        print(f"\n{green('✓')} Exported to {args.export}")

    print(f"\n  {'─' * 50}")
    ok = sum(1 for r in all_results.values() if r.get('correlation', 0) > 0.95)
    total = len(all_results)
    print(f"  {bold(f'{ok}/{total}')} variables with good correlation (>0.95)")


if __name__ == '__main__':
    main()
