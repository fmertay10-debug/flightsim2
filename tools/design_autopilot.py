"""Semi-automated autopilot design: linearize a vehicle from its OWN aero +
mass data and synthesize a gain-scheduled pitch autopilot by LQR (or pole
placement). This is the "control block gets its parameters from the aero/mass
blocks" step -- you pick the method and weights, the tool reads the vehicle's
tables and writes the gains the C++ ScheduledController consumes.

Pipeline:
  vehicle.json (aero tables + refs + mass) --> short-period plant at each Mach
  --> LQR / pole placement --> gain_schedule.csv (mach, k_alpha, k_q, k_theta, k_i)

Plant (augmented short period, per Mach), state x = [alpha, q, e, z]:
  e = theta - theta_cmd,  z = integral(e)
  alpha_dot = Za*alpha + q + Zde*de
  q_dot     = Ma*alpha + Mq*q + Mde*de
  e_dot     = q
  z_dot     = e
Control law applied in the sim:  de = -(k_alpha*alpha + k_q*q + k_theta*e + k_i*z)

Usage:
  py tools/design_autopilot.py vehicles/generated/datcom_rocket/vehicle.json \
      --mass 72 --iyy 380 --altitude 5000 --method lqr
  py tools/design_autopilot.py <vehicle.json> --method place --wn 12 --zeta 0.7
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import control as ct

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, ".."))


# ---- ISA atmosphere (troposphere + lower stratosphere, enough for design) ----
def isa(alt):
    if alt < 11000.0:
        T = 288.15 - 0.0065 * alt
        p = 101325.0 * (T / 288.15) ** 5.2559
    else:
        T = 216.65
        p = 22632.06 * math.exp(-9.80665 * (alt - 11000.0) / (287.05287 * T))
    rho = p / (287.05287 * T)
    a = math.sqrt(1.4 * 287.05287 * T)
    return rho, a


def strip_comments(text):
    out = []
    for line in text.splitlines():
        q = False
        for i, ch in enumerate(line):
            if ch == '"':
                q = not q
            elif ch == '/' and i + 1 < len(line) and line[i + 1] == '/' and not q:
                line = line[:i]
                break
        out.append(line)
    return "\n".join(out)


def load_jsonc(path):
    with open(path) as f:
        return json.loads(strip_comments(f.read()))


class Grid2D:
    """Bilinear interpolation over a tidy (x, y, z) CSV column, matching the
    C++ LookupTable2D (edge-clamped)."""
    def __init__(self, rows, xcol, ycol, zcol, cols):
        xi, yi, zi = cols[xcol], cols[ycol], cols[zcol]
        xs = sorted(set(r[xi] for r in rows))
        ys = sorted(set(r[yi] for r in rows))
        z = np.full((len(xs), len(ys)), np.nan)
        ix = {v: i for i, v in enumerate(xs)}
        iy = {v: i for i, v in enumerate(ys)}
        for r in rows:
            z[ix[r[xi]], iy[r[yi]]] = r[zi]
        self.xs, self.ys, self.z = np.array(xs), np.array(ys), z

    def __call__(self, x, y):
        xi = np.clip(np.interp(x, self.xs, np.arange(len(self.xs))), 0, len(self.xs) - 1)
        yi = np.clip(np.interp(y, self.ys, np.arange(len(self.ys))), 0, len(self.ys) - 1)
        x0, y0 = int(np.floor(xi)), int(np.floor(yi))
        x1, y1 = min(x0 + 1, len(self.xs) - 1), min(y0 + 1, len(self.ys) - 1)
        fx, fy = xi - x0, yi - y0
        return ((1 - fx) * (1 - fy) * self.z[x0, y0] + fx * (1 - fy) * self.z[x1, y0]
                + (1 - fx) * fy * self.z[x0, y1] + fx * fy * self.z[x1, y1])


def read_csv(path):
    rows = []
    with open(path) as f:
        header = f.readline().strip().split(",")
        cols = {name: i for i, name in enumerate(header)}
        for line in f:
            if line.strip():
                rows.append([float(v) for v in line.split(",")])
    return rows, cols


def plant(mach, V, qbar, S, cbar, Iyy, mass, dxc,
          CNa, CMa_ref, CMq, CNde, CMde):
    """Build the augmented short-period (A, B) at one flight condition.
    dxc = (xcg - xref) / cbar : CG offset in chords for the moment transfer."""
    # Transfer static + control pitch moments from the aero reference to the CG.
    CMa = CMa_ref + CNa * dxc          # Cm_alpha about CG
    CMde_cg = CMde + CNde * dxc

    Za = -qbar * S / (mass * V) * CNa           # normal-force alpha damping (<0)
    Zde = -qbar * S / (mass * V) * CNde
    Ma = qbar * S * cbar / Iyy * CMa            # pitch stiffness (<0 stable)
    Mq = qbar * S * cbar / Iyy * CMq * cbar / (2.0 * V)
    Mde = qbar * S * cbar / Iyy * CMde_cg

    A = np.array([[Za, 1.0, 0.0, 0.0],
                  [Ma, Mq,  0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0]])
    B = np.array([[Zde], [Mde], [0.0], [0.0]])
    return A, B


def design_gains(A, B, method, Q, R, wn, zeta):
    if method == "lqr":
        K, _, _ = ct.lqr(A, B, Q, R)
        return np.asarray(K).flatten()
    # Pole placement: short-period pair at (wn, zeta), plus two slow real poles
    # for the attitude/integral modes.
    sp = np.roots([1.0, 2.0 * zeta * wn, wn * wn])
    poles = np.array([sp[0], sp[1], -0.6 * wn, -0.25 * wn])
    K = ct.place(A, B, poles)
    return np.asarray(K).flatten()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("vehicle", help="path to the vehicle.json (DATCOM table rocket)")
    ap.add_argument("--altitude", type=float, default=5000.0, help="design altitude [m]")
    ap.add_argument("--mass", type=float, help="design mass [kg] (else from json)")
    ap.add_argument("--iyy", type=float, help="design pitch inertia [kg m^2]")
    ap.add_argument("--xcg", type=float, help="design CG station [m] (else = aero xref)")
    ap.add_argument("--method", choices=["lqr", "place"], default="lqr")
    # Defaults tuned so the resulting fin commands stay within a typical +/-15
    # deg limit on step pitch commands (see the demo in tools/README).
    ap.add_argument("--q-theta", type=float, default=20.0, help="LQR weight on theta error")
    ap.add_argument("--q-int", type=float, default=2.0, help="LQR weight on integral")
    ap.add_argument("--r", type=float, default=80.0, help="LQR control-effort weight")
    ap.add_argument("--wn", type=float, default=12.0, help="pole-placement short-period wn")
    ap.add_argument("--zeta", type=float, default=0.7, help="pole-placement short-period zeta")
    ap.add_argument("--out", help="output gain_schedule.csv (default: next to vehicle)")
    args = ap.parse_args()

    vpath = os.path.abspath(args.vehicle)
    vdir = os.path.dirname(vpath)
    cfg = load_jsonc(vpath)
    aero = next((c for c in cfg.get("components", [])
                 if c.get("type") == "rocket_table_aero"), None)
    if aero is None or "tables_csv" not in aero:
        sys.exit("design_autopilot: vehicle has no rocket_table_aero component "
                 "with DATCOM tables_csv (this tool designs for the table-aero "
                 "rocket)")

    S = aero["sref_m2"]
    cbar = aero["cbar_m"]
    xref = aero.get("xref_m", aero["cbar_m"] * 0.0)

    mass = args.mass
    Iyy = args.iyy
    if mass is None or Iyy is None:
        if "inertia" in cfg and "mass_kg" in cfg:
            mass = mass or cfg["mass_kg"]
            Iyy = Iyy or cfg["inertia"]["iyy"]
        else:
            sys.exit("design_autopilot: pass --mass and --iyy (tabulated mass "
                     "vehicle has no scalar mass in the json)")
    xcg = args.xcg if args.xcg is not None else xref
    dxc = (xcg - xref) / cbar

    stat_rows, stat_cols = read_csv(os.path.join(vdir, aero["tables_csv"]))
    ctrl_rows, ctrl_cols = read_csv(os.path.join(vdir, aero["control_csv"]))
    CN = Grid2D(stat_rows, "alpha_rad", "mach", "CN", stat_cols)
    CM = Grid2D(stat_rows, "alpha_rad", "mach", "CM", stat_cols)
    CMQ = Grid2D(stat_rows, "alpha_rad", "mach", "CMQ", stat_cols)
    dCM = Grid2D(ctrl_rows, "delta_rad", "mach", "dCM_sym", ctrl_cols)
    dCL = Grid2D(ctrl_rows, "delta_rad", "mach", "dCL_sym", ctrl_cols)

    machs = sorted(set(r[stat_cols["mach"]] for r in stat_rows))
    # Skip the near-zero band (on the rail) and the transonic hole, where the
    # DATCOM data thins out and the linear derivatives degenerate (uncontrollable
    # plant -> zero gains). The C++ side interpolates the schedule across the gap.
    machs = [m for m in machs if 0.15 <= m <= 3.0 and not (0.9 < m < 1.5)]

    da = math.radians(2.0)    # central-difference step in alpha
    dd = math.radians(2.0)    # ... and in deflection
    Q = np.diag([1.0, 1.0, args.q_theta, args.q_int])
    R = np.array([[args.r]])

    schedule = []
    for m in machs:
        rho, a = isa(args.altitude)
        V = m * a
        qbar = 0.5 * rho * V * V

        CNa = (CN(da, m) - CN(-da, m)) / (2 * da)
        CMa = (CM(da, m) - CM(-da, m)) / (2 * da)
        CMq = CMQ(0.0, m)
        CNde = (dCL(dd, m) - dCL(-dd, m)) / (2 * dd)
        CMde = (dCM(dd, m) - dCM(-dd, m)) / (2 * dd)

        A, B = plant(m, V, qbar, S, cbar, Iyy, mass, dxc, CNa, CMa, CMq, CNde, CMde)
        K = design_gains(A, B, args.method, Q, R, args.wn, args.zeta)

        # Closed-loop poles for reporting.
        cl = np.linalg.eigvals(A - B @ K.reshape(1, -1))
        schedule.append((m, K, cl, A))

    out = args.out or os.path.join(vdir, "gain_schedule.csv")
    with open(out, "w") as f:
        f.write("mach,k_alpha,k_q,k_theta,k_i\n")
        for m, K, _, _ in schedule:
            f.write(f"{m:.3f},{K[0]:.6f},{K[1]:.6f},{K[2]:.6f},{K[3]:.6f}\n")

    print(f"designed {args.method.upper()} pitch autopilot for {os.path.basename(vpath)}")
    print(f"  altitude {args.altitude:.0f} m, mass {mass:.1f} kg, Iyy {Iyy:.0f}, "
          f"xcg-xref {xcg - xref:+.3f} m")
    print(f"  {len(schedule)} Mach points -> {out}")
    print(f"  {'mach':>5} {'k_alpha':>9} {'k_q':>8} {'k_theta':>9} {'k_i':>8}  "
          f"{'open-loop':>12} {'closed-loop wn,zeta':>20}")
    for m, K, cl, A in schedule:
        ol = np.linalg.eigvals(A[:2, :2])
        stab = "stable" if np.all(np.real(ol) < 0) else "UNSTABLE"
        # dominant closed-loop complex pair
        cpx = [p for p in cl if abs(p.imag) > 1e-6]
        if cpx:
            p = max(cpx, key=lambda z: z.real)
            wn = abs(p); zeta = -p.real / wn if wn > 0 else 0.0
            clstr = f"wn={wn:4.1f} z={zeta:4.2f}"
        else:
            clstr = "real poles"
        print(f"  {m:5.2f} {K[0]:9.3f} {K[1]:8.3f} {K[2]:9.3f} {K[3]:8.3f}  "
              f"{stab:>12} {clstr:>20}")


if __name__ == "__main__":
    main()
