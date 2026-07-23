#!/usr/bin/env python3
"""Compare a flightsim2 run against NESC 6-DOF check-case reference data.

The NESC check-cases (NASA/TM-2015-218675, nescacademy.nasa.gov/flightsim)
publish the same trajectory from ~5 independent simulations. For each mapped
channel this tool measures, on our time grid, the worst-over-time distance
from our value to the NEAREST reference simulation, and judges it against
the reference sims' own mutual scatter: PASS when we sit inside (a multiple
of) the disagreement the independent references already have with each
other. Angle channels use wrapped distances.

Units: ours are SI (CsvLogger); references are imperial (ft, deg, psf).

Usage:
  python3 tools/nesc_compare.py <ours.csv> <reference_case_dir> [--tol-mult 1.5]
Exit 0 when every channel passes.
"""

import argparse
import csv
import glob
import math
import os
import subprocess
import sys

M2FT = 1.0 / 0.3048
R2D = 180.0 / math.pi
PA2PSF = 0.3048 * 0.3048 / 4.4482216152605   # Pa -> lbf/ft^2

# ours-column -> (theirs-column, ours->imperial factor, wrapped?, abs floor)
CHANNELS = [
    ("altitude", "altitudeMsl_ft",                    M2FT,    False, 2.0),
    ("vx",       "feVelocity_ft_s_X",                 M2FT,    False, 1.0),
    ("vy",       "feVelocity_ft_s_Y",                 M2FT,    False, 1.0),
    ("vz",       "feVelocity_ft_s_Z",                 M2FT,    False, 1.0),
    ("p",        "bodyAngularRateWrtEi_deg_s_Roll",   R2D,     True,  0.05),
    ("q",        "bodyAngularRateWrtEi_deg_s_Pitch",  R2D,     True,  0.05),
    ("r",        "bodyAngularRateWrtEi_deg_s_Yaw",    R2D,     True,  0.05),
    ("phi",      "eulerAngle_deg_Roll",               R2D,     True,  0.5),
    ("theta",    "eulerAngle_deg_Pitch",              R2D,     True,  0.5),
    ("psi",      "eulerAngle_deg_Yaw",                R2D,     True,  0.5),
    ("mach",     "mach",                              1.0,     False, 0.003),
    ("qbar",     "dynamicPressure_lbf_ft2",           PA2PSF,  False, 0.05),
]


def load(path):
    with open(path, newline="") as f:
        rows = [r for r in csv.DictReader(f)]
    cols = {}
    for k in rows[0]:
        try:
            cols[k] = [float(r[k]) for r in rows]
        except (ValueError, TypeError):
            pass
    return cols


def interp(ts, ys, t):
    if t <= ts[0]:
        return ys[0]
    if t >= ts[-1]:
        return ys[-1]
    lo, hi = 0, len(ts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ts[mid] <= t:
            lo = mid
        else:
            hi = mid
    f = (t - ts[lo]) / (ts[hi] - ts[lo])
    return ys[lo] + f * (ys[hi] - ys[lo])


def dist(a, b, wrapped):
    d = a - b
    if wrapped:
        d = (d + 180.0) % 360.0 - 180.0
    return abs(d)


def compare(ours_path, ref_dir, tol_mult, only=None):
    ours = load(ours_path)
    refs = {}
    for p in sorted(glob.glob(os.path.join(ref_dir, "*_sim_*.csv"))):
        refs[os.path.basename(p)] = load(p)
    if not refs:
        raise SystemExit(f"no reference *_sim_*.csv under {ref_dir}")
    print(f"{ours_path} vs {len(refs)} reference sims in {ref_dir}")

    all_ok = True
    hdr = f"{'channel':>9s} {'refs':>4s} {'worst_dev':>10s} {'ref_scatter':>11s} " \
          f"{'tol':>8s}  verdict"
    print(hdr)
    for ours_col, ref_col, k, wrapped, floor in CHANNELS:
        if ours_col not in ours:
            continue
        if only and ours_col not in only:
            continue
        avail = {n: c for n, c in refs.items() if ref_col in c}
        if len(avail) < 2:
            print(f"{ours_col:>9s}    - {'':>10s} {'':>11s} {'':>8s}  SKIP "
                  f"(<2 refs carry {ref_col})")
            continue

        t_end = min(min(c["time"][-1] for c in avail.values()),
                    ours["time"][-1])
        ts = [t for t in ours["time"] if t <= t_end + 1e-9]
        ov = [v * k for v in ours[ours_col][:len(ts)]]

        worst = 0.0
        scatter = 0.0
        names = list(avail)
        for i, t in enumerate(ts):
            rv = [interp(avail[n]["time"], avail[n][ref_col], t)
                  for n in names]
            worst = max(worst, min(dist(ov[i], v, wrapped) for v in rv))
            for a in range(len(rv)):
                for b in range(a + 1, len(rv)):
                    scatter = max(scatter, dist(rv[a], rv[b], wrapped))

        tol = max(tol_mult * scatter, floor)
        ok = worst <= tol
        all_ok = all_ok and ok
        print(f"{ours_col:>9s} {len(avail):>4d} {worst:>10.4g} "
              f"{scatter:>11.4g} {tol:>8.4g}  {'PASS' if ok else 'FAIL'}")
    return all_ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("ours", help="our log CSV (produced by --scenario if given)")
    ap.add_argument("ref_dir")
    ap.add_argument("--tol-mult", type=float, default=1.5,
                    help="tolerance as a multiple of the reference scatter")
    ap.add_argument("--only", help="comma-separated channel subset, e.g. "
                    "p,q,r,phi,theta,psi (cases whose environment we cannot "
                    "fully represent gate on their rotational purpose)")
    ap.add_argument("--flightsim", help="flightsim binary: run --scenario first")
    ap.add_argument("--scenario", help="scenario JSON to fly before comparing")
    args = ap.parse_args()
    if args.flightsim and args.scenario:
        subprocess.run([args.flightsim, args.scenario], check=True,
                       stdout=subprocess.DEVNULL)
    only = set(args.only.split(",")) if args.only else None
    sys.exit(0 if compare(args.ours, args.ref_dir, args.tol_mult, only) else 1)


if __name__ == "__main__":
    main()
