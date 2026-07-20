"""Semi-automated autopilot design: linearize a vehicle from its OWN aero +
mass data and synthesize a gain-scheduled pitch autopilot by LQR (or pole
placement). This is the "control block gets its parameters from the aero/mass
blocks" step -- you pick the method and weights, the tool reads the vehicle's
tables and writes the gains the C++ ScheduledLaw consumes.

ACCELERATION DOMAIN (ADR-0004 Option C1): the design input is the demanded
pitch angular acceleration a_cmd, not a fin angle -- at runtime the law emits
WrenchCommand.moment = Iyy*a_cmd and the Allocator divides by the LIVE control
effectiveness, so the loop bandwidth self-adjusts as qbar leaves the design
condition (the old fin-domain schedule could not). Equivalent to the fin-domain
LQR by exact input scaling (B and R rescale by the control power Mde), so at
the design condition the closed loop is IDENTICAL to the historic design.

Pipeline:
  vehicle.json (aero tables + refs + mass) --> short-period plant at each Mach
  --> LQR / pole placement --> gain_schedule.csv
      (mach, k_alpha_acc, k_q_acc, k_theta_acc, k_i_acc)   [rad/s^2 per unit]

Plant (augmented short period, per Mach), state x = [alpha, q, e, z]:
  e = theta - theta_cmd,  z = integral(e)
  alpha_dot = Za*alpha + q + (Zde/Mde)*a
  q_dot     = Ma*alpha + Mq*q + a
  e_dot     = q
  z_dot     = e
Law applied in the sim:  a = -(kA_acc*alpha + kQ_acc*q + kT_acc*e + kI_acc*z)

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
    Mde = qbar * S * cbar / Iyy * CMde_cg       # control power [1/s^2 per rad fin]

    # Acceleration-domain input: a = Mde*de, so the accel column is [Zde/Mde, 1]
    # -- the fin-lift coupling per unit accel plus the accel itself.
    A = np.array([[Za, 1.0, 0.0, 0.0],
                  [Ma, Mq,  0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0]])
    B = np.array([[Zde / Mde], [1.0], [0.0], [0.0]])
    return A, B, Mde


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


def write_and_report(schedule, out, method, source):
    with open(out, "w") as f:
        f.write("mach,k_alpha_acc,k_q_acc,k_theta_acc,k_i_acc\n")
        for m, K, _, _ in schedule:
            f.write(f"{m:.3f},{K[0]:.6f},{K[1]:.6f},{K[2]:.6f},{K[3]:.6f}\n")
    print(f"designed {method.upper()} pitch autopilot from {source}")
    print(f"  {len(schedule)} Mach points -> {out}")
    print(f"  {'mach':>5} {'kA_acc':>9} {'kQ_acc':>8} {'kT_acc':>9} {'kI_acc':>8}  "
          f"{'open-loop':>12} {'closed-loop wn,zeta':>20}")
    for m, K, cl, A in schedule:
        ol = np.linalg.eigvals(A[:2, :2])
        stab = "stable" if np.all(np.real(ol) < 0) else "UNSTABLE"
        cpx = [pz for pz in cl if abs(pz.imag) > 1e-6]
        if cpx:
            pz = max(cpx, key=lambda z: z.real)
            wn = abs(pz); zeta = -pz.real / wn if wn > 0 else 0.0
            clstr = f"wn={wn:4.1f} z={zeta:4.2f}"
        else:
            clstr = "real poles"
        print(f"  {m:5.2f} {K[0]:9.3f} {K[1]:8.3f} {K[2]:9.3f} {K[3]:8.3f}  "
              f"{stab:>12} {clstr:>20}")


def design_from_plant(args):
    """Design from a sim-linearized plant.csv (flightsim --linearize)."""
    rows, cols = read_csv(args.plant)
    Q = np.diag([1.0, 1.0, args.q_theta, args.q_int])
    R = np.array([[args.r]])
    schedule = []
    for r in rows:
        m = r[cols["mach"]]
        Za, Zde = r[cols["Za"]], r[cols["Zde"]]
        Ma, Mq, Mde = r[cols["Ma"]], r[cols["Mq"]], r[cols["Mde"]]
        if abs(Mde) < 1e-9:
            continue
        A = np.array([[Za, 1.0, 0.0, 0.0],
                      [Ma, Mq,  0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0]])
        B = np.array([[Zde / Mde], [1.0], [0.0], [0.0]])
        K = design_gains(A, B, args.method, Q, R / (Mde * Mde), args.wn, args.zeta)
        cl = np.linalg.eigvals(A - B @ K.reshape(1, -1))
        schedule.append((m, K, cl, A))
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.plant)),
                                   "gain_schedule.csv")
    write_and_report(schedule, out, args.method, os.path.basename(args.plant))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("vehicle", nargs="?",
                    help="path to the vehicle.json (DATCOM table rocket); "
                         "not needed with --plant")
    ap.add_argument("--plant",
                    help="plant.csv from `flightsim --linearize` (the sim's "
                         "own f(x,u), ADR-0004 C2). Replaces the DATCOM-table "
                         "plant -- works for ANY vehicle with an elevator. "
                         "--altitude/--mass/--iyy/--xcg are ignored (already "
                         "baked into the plant).")
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

    if args.plant:
        return design_from_plant(args)
    if args.vehicle is None:
        sys.exit("design_autopilot: pass a vehicle.json or --plant plant.csv")
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
        mblock = cfg.get("mass", {})
        model = mblock.get("model")
        if model == "constant":
            mass = mass or mblock["mass_kg"]
            Iyy = Iyy or mblock["inertia"]["iyy"]
        elif model == "dry_plus_propellant":
            # Design at dry mass by default; pass --mass for a mid-burn point.
            mass = mass or mblock["dry_mass_kg"]
            Iyy = Iyy or mblock["inertia"]["iyy"]
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

        A, B, Mde = plant(m, V, qbar, S, cbar, Iyy, mass, dxc, CNa, CMa, CMq, CNde, CMde)
        if abs(Mde) < 1e-9:
            continue   # no control power here (degenerate data): skip the point
        # --r keeps its historic fin-effort meaning: R_acc = R_fin / Mde^2 makes
        # the accel-domain LQR EXACTLY the input-scaled fin-domain design.
        R_acc = R / (Mde * Mde)
        K = design_gains(A, B, args.method, Q, R_acc, args.wn, args.zeta)

        # Closed-loop poles for reporting.
        cl = np.linalg.eigvals(A - B @ K.reshape(1, -1))
        schedule.append((m, K, cl, A))

    out = args.out or os.path.join(vdir, "gain_schedule.csv")
    print(f"  altitude {args.altitude:.0f} m, mass {mass:.1f} kg, Iyy {Iyy:.0f}, "
          f"xcg-xref {xcg - xref:+.3f} m")
    write_and_report(schedule, out, args.method, os.path.basename(vpath))


if __name__ == "__main__":
    main()
