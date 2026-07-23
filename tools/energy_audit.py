#!/usr/bin/env python3
"""Independent force-bookkeeping audit of a flight log (stdlib only).

Reconstructs the velocity history by integrating the LOGGED per-component
body forces -- rotated to NED through the LOGGED Euler angles -- plus
gravity, divided by the LOGGED mass, and compares against the logged
velocity. The reconstruction shares no code with the sim (its own DCM, its
own integrator), so a frame error, an unlogged force, or a gravity mistake
shows up as an O(1) mismatch, while a healthy log agrees to the trapezoid
error of the sample spacing.

Limits (by design): translational only (the logs don't carry inertia), and
the tolerance must absorb first-order integration error over the log's
decimation interval -- this is an oracle for force bookkeeping and frames,
not a precision replay.

Usage:
  python3 tools/energy_audit.py <log.csv> [--gravity flat|spherical]
                                [--tol 0.02]
Exit 0 when the reconstruction agrees, 1 when it does not.
"""

import argparse
import csv
import math
import sys

G0 = 9.80665
RE = 6371000.0   # mean Earth radius [m], matches src/environment/GravityModel.h


def body_to_ned(phi, th, psi, v):
    """3-2-1 Euler body->NED rotation (independent of the sim's quaternion)."""
    cph, sph = math.cos(phi), math.sin(phi)
    cth, sth = math.cos(th), math.sin(th)
    cps, sps = math.cos(psi), math.sin(psi)
    x, y, z = v
    return (
        cth * cps * x + (sph * sth * cps - cph * sps) * y
        + (cph * sth * cps + sph * sps) * z,
        cth * sps * x + (sph * sth * sps + cph * cps) * y
        + (cph * sth * sps - sph * cps) * z,
        -sth * x + sph * cth * y + cph * cth * z,
    )


def audit(path, gravity="flat", tol=0.02):
    with open(path, newline="") as f:
        rows = [dict((k, float(v)) for k, v in r.items())
                for r in csv.DictReader(f)]
    if len(rows) < 3:
        raise SystemExit(f"{path}: too few rows to audit")

    # Discover the per-component force triads (<name>_fx/_fy/_fz).
    comps = sorted(k[:-3] for k in rows[0] if k.endswith("_fx")
                   and k[:-3] + "_fy" in rows[0] and k[:-3] + "_fz" in rows[0])
    if not comps:
        raise SystemExit(f"{path}: no component force columns to audit")

    def accel_ned(r):
        fb = [sum(r[c + "_f" + ax] for c in comps) for ax in "xyz"]
        an, ae, ad = body_to_ned(r["phi"], r["theta"], r["psi"], fb)
        m = r["mass"]
        g = G0 if gravity == "flat" else \
            G0 * (RE / (RE + r["altitude"])) ** 2
        return (an / m, ae / m, ad / m + g)   # NED: gravity is +z (down)

    # Trapezoidal reconstruction from the first logged state.
    v = (rows[0]["vx"], rows[0]["vy"], rows[0]["vz"])
    vmax = worst = 0.0
    worst_t = 0.0
    a_prev = accel_ned(rows[0])
    for prev, cur in zip(rows, rows[1:]):
        dt = cur["time"] - prev["time"]
        a_cur = accel_ned(cur)
        v = tuple(vi + 0.5 * dt * (ap + ac)
                  for vi, ap, ac in zip(v, a_prev, a_cur))
        a_prev = a_cur
        dev = math.dist(v, (cur["vx"], cur["vy"], cur["vz"]))
        speed = math.hypot(cur["vx"], cur["vy"], cur["vz"])
        vmax = max(vmax, speed)
        if dev > worst:
            worst, worst_t = dev, cur["time"]

    frac = worst / max(vmax, 1e-9)
    ok = frac <= tol
    print(f"{path}: {len(rows)} rows, components {'+'.join(comps)}; "
          f"max |v_reconstructed - v_logged| = {worst:.3g} m/s at "
          f"t={worst_t:.2f}s ({100 * frac:.2f}% of peak speed, "
          f"tol {100 * tol:.0f}%) -> {'OK' if ok else 'FAIL'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", nargs="+", help="flight log CSV(s)")
    ap.add_argument("--gravity", choices=["flat", "spherical"], default="flat")
    ap.add_argument("--tol", type=float, default=0.02,
                    help="max deviation as a fraction of peak speed")
    args = ap.parse_args()
    ok = all(audit(p, args.gravity, args.tol) for p in args.logs)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
