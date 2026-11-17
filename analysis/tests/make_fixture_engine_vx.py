#!/usr/bin/env python3
"""
Generate synthetic engine-logger-style CSV fixtures for Part A of the
velocity-characterization verification.

Format mimics the Engine logger exactly:
    ElapsedTime,RobotID,Team,Vx_Command,Vy_Command,Angular_Command,
    Pos_X,Pos_Y,Orientation,Vx_Actual,Vy_Actual

Plant: known first-order-plus-dead-time (FOPDT)
    Vx_Actual(t) = 0                                for t < t_step + L
    Vx_Actual(t) = K*A*(1 - exp(-(t - t_step - L)/T)) + noise   else
with K = 0.95, A = 1.0 and per-fixture (L, T, dt).

Fixtures produced (all with the engine-logger header):
  * fixture_engine_vx_step.csv        - L=0.030, T=0.120 @ 60 Hz  (task spec)
  * fixture_engine_vx_step_verif.csv  - L=0.300, T=0.200 @ 60 Hz
                                        (dead time / time constant resolvable
                                        on the 16.7 ms 60 Hz grid -> proves the
                                        tools recover a known plant)
  * fixture_engine_vx_step_1khz.csv   - L=0.030, T=0.120 @ 1 kHz
                                        (same plant as spec; isolates the
                                        sampling-rate contribution to the
                                        dead-time estimate bias)

Command timeline (tools need >= 2 detected steps and >= 20 samples per
pre-step plateau; a short scaffolding second transition closes the pair):
  * t = 0.000 .. just before t_step1 : Vx_Command = 0
  * t = t_step1 .. t_step2 - dt       : Vx_Command = 1.0   (step 0 -> 1.0)
  * t = t_step2 .. t_end              : Vx_Command = 0.5   (tail < 20 samples
                                                             -> exactly 2 steps)

Noise: zero-mean Gaussian, std = 0.01 on Vx_Actual only (seeded).
Pos_X = cumsum(Vx_Actual * dt).
"""
import csv
import math

import numpy as np

K_TRUE = 0.95
A = 1.0
NOISE_STD = 0.01
T_STEP1 = 0.350          # >= 20 pre-step samples at every rate used here
SEED = 42

HEADER = ("ElapsedTime,RobotID,Team,Vx_Command,Vy_Command,Angular_Command,"
          "Pos_X,Pos_Y,Orientation,Vx_Actual,Vy_Actual")


def make_fixture(path, l_true, t_true, dt, seed=SEED):
    rng = np.random.default_rng(seed)

    # Second step placed after full settling of the first response,
    # tail kept < 20 samples so only two steps are detected.
    t_step2 = T_STEP1 + l_true + 6.0 * t_true + 0.05
    t_end = t_step2 + 15.0 * dt

    n = int(round(t_end / dt)) + 1
    t = np.arange(n) * dt

    cmd = np.zeros(n)
    cmd[t >= T_STEP1] = A
    cmd[t >= t_step2] = 0.5

    vx = np.zeros(n)
    for i, tt in enumerate(t):
        if tt >= T_STEP1 + l_true:
            vx[i] = K_TRUE * A * (1.0 - math.exp(-(tt - T_STEP1 - l_true) / t_true))
    vx += rng.normal(0.0, NOISE_STD, n)

    pos_x = np.cumsum(vx * dt)

    n_pre = int(round(T_STEP1 / dt))         # samples strictly before step 1
    n_tail = n - int(round(t_step2 / dt))    # samples after step 2
    assert n_pre >= 20, f"pre-step plateau {n_pre} < 20 -> step undetected"
    assert 1 <= n_tail < 20, f"tail plateau {n_tail} -> extra step appended"

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER.split(","))
        for i in range(n):
            w.writerow([
                f"{t[i]:.6f}", "0", "0",
                f"{cmd[i]:.6f}", "0", "0",
                f"{pos_x[i]:.6f}", "0", "0",
                f"{vx[i]:.6f}", "0",
            ])

    print(f"{path}: n={n} rows, dt={dt:.6f}s, t_end={t[-1]:.3f}s, "
          f"L_true={l_true}, T_true={t_true}, pre={n_pre}, tail={n_tail}")


if __name__ == "__main__":
    make_fixture("fixture_engine_vx_step.csv", 0.030, 0.120, 1.0 / 60.0)
    make_fixture("fixture_engine_vx_step_verif.csv", 0.300, 0.200, 1.0 / 60.0)
    make_fixture("fixture_engine_vx_step_1khz.csv", 0.030, 0.120, 1.0 / 1000.0)
