#!/usr/bin/env python3
"""Golden regression gate for flightsim2.

Runs each scenario that has a committed baseline under golden/ and compares the
freshly written output/ CSVs (and the console summary) against it, cell by cell,
with a number-aware tolerance. This is the safety net for refactors: a flipped
sign, a reordered force sum, or a changed integration step moves a number and
fails here immediately.

Comparison is number-aware on purpose:
  - "-0" and "0" are equal (they parse to the same double);
  - "nan" equals "nan" (raw text/float equality would get both backwards);
  - non-numeric cells (none today) compare as trimmed text.

Usage (python3 on Linux, py on Windows):
  python3 tools/check_golden.py                     # check every golden scenario
  python3 tools/check_golden.py lqr_rocket_launch f16_cruise
  python3 tools/check_golden.py --atol 1e-9 --rtol 1e-9
  python3 tools/check_golden.py --update lqr_rocket_launch  # RE-BASELINE named scenarios
  python3 tools/check_golden.py --flightsim build/flightsim

Registered in ctest as `golden_gate`, so `ctest --test-dir build` runs it.
Exit code is nonzero if any scenario fails (so it can gate CI / a commit hook).
"""

import argparse
import csv
import math
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
GOLDEN = os.path.join(PROJ, "golden")
OUTPUT = os.path.join(PROJ, "output")


def parse_cell(s):
    """A cell as a float when it looks numeric, else its raw string."""
    s = s.strip()
    try:
        return float(s)
    except ValueError:
        return s


def cell_equal(a, b, atol, rtol):
    fa, fb = parse_cell(a), parse_cell(b)
    if isinstance(fa, float) and isinstance(fb, float):
        a_nan, b_nan = math.isnan(fa), math.isnan(fb)
        if a_nan or b_nan:
            return a_nan and b_nan          # nan matches only nan
        if math.isinf(fa) or math.isinf(fb):
            return fa == fb                  # inf must match exactly, sign included
        return abs(fa - fb) <= atol + rtol * abs(fb)
    return str(fa).strip() == str(fb).strip()


def compare_csv(golden_path, output_path, atol, rtol):
    """Return (ok, message). Reports the FIRST differing cell if any."""
    if not os.path.exists(output_path):
        return False, f"output missing: {os.path.relpath(output_path, PROJ)}"
    with open(golden_path, newline="") as f:
        grows = list(csv.reader(f))
    with open(output_path, newline="") as f:
        orows = list(csv.reader(f))

    if not grows:
        return False, "golden CSV is empty"
    if grows[0] != orows[0]:
        return False, (f"header differs\n    golden: {grows[0]}\n    output: {orows[0]}")
    if len(grows) != len(orows):
        return False, f"row count differs: golden {len(grows)} vs output {len(orows)}"

    header = grows[0]
    max_abs = 0.0
    for r in range(1, len(grows)):
        g, o = grows[r], orows[r]
        if len(g) != len(o):
            return False, f"row {r}: column count differs ({len(g)} vs {len(o)})"
        for c in range(len(g)):
            if not cell_equal(g[c], o[c], atol, rtol):
                col = header[c] if c < len(header) else f"col{c}"
                return False, (f"row {r} col '{col}' (line {r + 1}): "
                               f"golden={g[c]!r} output={o[c]!r}")
            fa, fb = parse_cell(g[c]), parse_cell(o[c])
            if (isinstance(fa, float) and isinstance(fb, float)
                    and not math.isnan(fa) and not math.isinf(fa)):
                max_abs = max(max_abs, abs(fa - fb))
    return True, f"max|delta|={max_abs:.2e}"


def compare_stdout(name, actual, atol, rtol):
    """Soft check: number-aware line compare of the console summary."""
    golden_file = os.path.join(GOLDEN, f"{name}.stdout.txt")
    if not os.path.exists(golden_file):
        return True, "no stdout golden"
    with open(golden_file) as f:
        gold = f.read().splitlines()
    got = actual.splitlines()
    if len(gold) != len(got):
        return False, f"line count differs ({len(gold)} vs {len(got)})"
    for i, (gl, ol) in enumerate(zip(gold, got)):
        gtok, otok = gl.split(), ol.split()
        if len(gtok) != len(otok):
            return False, f"line {i + 1} token count differs"
        for gt, ot in zip(gtok, otok):
            if not cell_equal(gt, ot, atol, rtol):
                return False, f"line {i + 1}: {gt!r} vs {ot!r}"
    return True, "ok"


def golden_scenarios():
    names = []
    for entry in sorted(os.listdir(GOLDEN)):
        d = os.path.join(GOLDEN, entry)
        if os.path.isdir(d):
            names.append(entry)
    return names


def run_scenario(name, flightsim):
    scenario = os.path.join(PROJ, "scenarios", f"{name}.json")
    if not os.path.exists(scenario):
        return None, f"scenario file missing: scenarios/{name}.json"
    # Run from the repo root: log paths and vehicle paths are relative to it.
    proc = subprocess.run([flightsim, os.path.join("scenarios", f"{name}.json")],
                          cwd=PROJ, capture_output=True, text=True)
    if proc.returncode != 0:
        return None, f"flightsim exited {proc.returncode}\n{proc.stderr.strip()}"
    return proc.stdout, None


def update_golden(name):
    """Copy fresh output/<name>/*.csv and the stdout over the committed golden."""
    src = os.path.join(OUTPUT, name)
    dst = os.path.join(GOLDEN, name)
    os.makedirs(dst, exist_ok=True)
    for f in os.listdir(dst):
        if f.endswith(".csv"):
            os.remove(os.path.join(dst, f))
    for f in os.listdir(src):
        if f.endswith(".csv"):
            shutil.copy2(os.path.join(src, f), os.path.join(dst, f))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenarios", nargs="*", help="scenario names (default: all golden)")
    ap.add_argument("--atol", type=float, default=1e-12, help="absolute tolerance")
    ap.add_argument("--rtol", type=float, default=1e-12, help="relative tolerance")
    ap.add_argument("--flightsim", default=None,
                    help="path to the flightsim executable "
                         "(default: build/flightsim[.exe], whichever exists)")
    ap.add_argument("--update", action="store_true",
                    help="RE-BASELINE: overwrite the named goldens with fresh output")
    args = ap.parse_args()

    flightsim = args.flightsim
    if flightsim is None:
        for candidate in ("flightsim", "flightsim.exe"):
            path = os.path.join(PROJ, "build", candidate)
            if os.path.exists(path):
                flightsim = path
                break
        else:
            flightsim = os.path.join(PROJ, "build", "flightsim")
    if not os.path.isabs(flightsim):
        flightsim = os.path.join(PROJ, flightsim)
    if not os.path.exists(flightsim):
        print(f"error: flightsim not found at {flightsim}\n"
              f"       build first: cmake --build build -j", file=sys.stderr)
        return 2

    names = args.scenarios or golden_scenarios()
    if args.update and not args.scenarios:
        print("refusing to --update ALL goldens at once; name the scenarios "
              "you mean to re-baseline.", file=sys.stderr)
        return 2

    print(f"golden gate: {len(names)} scenario(s), atol={args.atol:g} rtol={args.rtol:g}"
          + ("  [UPDATE MODE]" if args.update else ""))
    print("-" * 72)

    failures = 0
    for name in names:
        stdout, err = run_scenario(name, flightsim)
        if err:
            print(f"[ERROR ] {name}: {err}")
            failures += 1
            continue

        if args.update:
            update_golden(name)
            with open(os.path.join(GOLDEN, f"{name}.stdout.txt"), "w",
                      newline="") as f:
                f.write(stdout)
            print(f"[UPDATED] {name}: CSVs + stdout re-baselined from output/")
            continue

        gdir = os.path.join(GOLDEN, name)
        csvs = sorted(f for f in os.listdir(gdir) if f.endswith(".csv"))
        scen_ok = True
        details = []
        for fname in csvs:
            ok, msg = compare_csv(os.path.join(gdir, fname),
                                  os.path.join(OUTPUT, name, fname),
                                  args.atol, args.rtol)
            scen_ok = scen_ok and ok
            details.append(("  " + ("ok " if ok else "DIFF") + f" {fname}: {msg}"))

        sout_ok, sout_msg = compare_stdout(name, stdout, args.atol, args.rtol)
        scen_ok = scen_ok and sout_ok

        print(f"[{'PASS' if scen_ok else 'FAIL'}  ] {name}")
        if not scen_ok:
            for d in details:
                print(d)
            if not sout_ok:
                print(f"  DIFF stdout: {sout_msg}")
            failures += 1

    print("-" * 72)
    if args.update:
        print("done (update mode).")
        return 0
    print(f"{len(names) - failures}/{len(names)} scenarios match golden.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
