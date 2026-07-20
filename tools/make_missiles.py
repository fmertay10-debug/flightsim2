"""Build the missile example vehicles the WHOLE way: DATCOM aero run -> tables
-> vehicle.json -> auto-designed LQR autopilot. One command per class, or all.

For each missile it:
  1. defines the airframe geometry (pydatcom Vehicle, metres),
  2. runs DATCOM over a Mach sweep with a fin-deflection schedule (control),
  3. exports aero_tables.csv / control_tables.csv / mesh.json (datcom_export),
  4. writes vehicle.json: DATCOM table aero + dry mass + solid motor (propellant
     burns off) + an LQR controller,
  5. designs the gain schedule from that vehicle's own aero+mass
     (tools/design_autopilot.py).

Usage:
  py tools/make_missiles.py all
  py tools/make_missiles.py aam            # just one
"""
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(PROJ, "datcom"))
sys.path.insert(0, HERE)

from pydatcom import Vehicle, run_pipeline           # noqa: E402
import datcom_export as dx                            # noqa: E402

OUT = os.path.join(PROJ, "vehicles", "generated")

# Mach sweep + symmetric fin-deflection schedule (deg) for the control tables.
MACH = [0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.6, 2.0, 2.5, 3.0]
DELTA = [-15.0, -7.0, 0.0, 7.0, 15.0]

# name -> geometry (Vehicle, metres) + mass/motor/design/run altitude.
# ixx: roll inertia. The slender-body lower bound (0.5*m*r^2) makes the roll
# axis so fast that the fin loop limit-cycles at high qbar; fins + structure add
# substantially to it, so a realistic multiple is used -- and the ground-attack
# missiles (which dive to high qbar) get a lower design altitude so their gains
# stay valid there.
MISSILES = {
    "aam": dict(
        veh=Vehicle(unit="M", L=2.9, D=0.18, L_nc=0.55, L_af=2.35, nc="ogive",
                    xcg=1.55, fin_root=0.34, fin_tip=0.16, fin_sweep=35.0,
                    fin_height=0.20, fin_num=4),
        dry=58, prop=24, ixx=2.5, iyy=42,
        thrust=[[0.0, 20000], [0.25, 22000], [2.6, 19000], [3.0, 0]],
        design=dict(mass=70, iyy=42, alt=6000), max_fin=28, min_v=40),
    "sam": dict(
        veh=Vehicle(unit="M", L=3.6, D=0.25, L_nc=0.75, L_af=2.85, nc="ogive",
                    xcg=1.95, fin_root=0.45, fin_tip=0.20, fin_sweep=35.0,
                    fin_height=0.26, fin_num=4),
        dry=100, prop=55, ixx=6.0, iyy=150,
        thrust=[[0.0, 52000], [0.3, 58000], [4.5, 48000], [5.0, 0]],
        design=dict(mass=128, iyy=150, alt=4000, qt=80, qi=8, r=40), max_fin=24, min_v=30),
    "agm": dict(
        veh=Vehicle(unit="M", L=3.2, D=0.30, L_nc=0.70, L_af=2.50, nc="ogive",
                    xcg=1.60, fin_root=0.42, fin_tip=0.18, fin_sweep=30.0,
                    fin_height=0.24, fin_num=4),
        dry=96, prop=22, ixx=5.0, iyy=95,
        thrust=[[0.0, 15000], [0.3, 16000], [3.0, 13000], [3.5, 0]],
        design=dict(mass=107, iyy=95, alt=1200), max_fin=25, min_v=35),
    "ssm": dict(
        veh=Vehicle(unit="M", L=5.0, D=0.36, L_nc=1.10, L_af=3.90, nc="ogive",
                    xcg=2.80, fin_root=0.60, fin_tip=0.25, fin_sweep=30.0,
                    fin_height=0.34, fin_num=4),
        dry=185, prop=110, ixx=14.0, iyy=420,
        thrust=[[0.0, 90000], [0.4, 98000], [6.0, 82000], [6.5, 0]],
        design=dict(mass=240, iyy=420, alt=2000), max_fin=20, min_v=30),
}


def build(name, spec):
    outdir = os.path.join(OUT, name)
    os.makedirs(outdir, exist_ok=True)
    print(f"\n=== {name.upper()} ===")

    # 1-2. Run DATCOM (input deck -> solver -> parse) with fin control.
    print(f"  running DATCOM ({len(MACH)} Mach x {len(DELTA)} deflections)...")
    aero = run_pipeline(spec["veh"], M=MACH, alpha=None, delta=DELTA,
                        outdir=outdir, alt=1000.0, timeout=180.0)
    npz = os.path.join(outdir, "aero.npz")
    aero.save_npz(npz)
    d = np.load(npz, allow_pickle=True)
    scale = dx.unit_scale(d)

    # 3. Export tables + mesh (reuse the datcom_export writers).
    na, nm = dx.write_aero_tables(d, outdir)
    controlled = dx.write_control_tables(d, outdir)
    dx.write_mesh(spec["veh"], scale, outdir)
    if not controlled:
        print("  WARNING: no control tables from DATCOM (uncontrolled airframe)")

    # 4. Vehicle: DATCOM table aero + dry mass + solid motor + LQR controller.
    xref = round(float(d["xcg"]) * scale, 4)
    cfg = {
        "mass": {
            "model": "dry_plus_propellant",
            "dry_mass_kg": spec["dry"],
            "inertia": {"ixx": spec["ixx"], "iyy": spec["iyy"],
                        "izz": spec["iyy"]},
        },
        "components": [
            {
                "type": "rocket_table_aero",
                "sref_m2": round(float(d["sref"]) * scale ** 2, 6),
                "cbar_m": round(float(d["cbar"]) * scale, 4),
                "bref_m": round(float(d["blref"]) * scale, 4),
                "tables_csv": "aero_tables.csv",
                "control_csv": "control_tables.csv",
                "xref_m": xref,
            },
            {
                "type": "solid_motor", "propellant_kg": spec["prop"],
                "thrust_curve": spec["thrust"],
            },
        ],
        "gnc": {
            "control_law": {
                "type": "lqr",
                "schedule": "gain_schedule.csv",
                "limits": {"max_ang_accel_dps2": 40000,
                           "min_airspeed_ms": spec["min_v"]},
                "roll": {"kp": 40, "kd": 15},
            },
            "actuator": {"tau_s": 0.02, "rate_dps": 450, "limit_deg": spec["max_fin"]},
        },
        "geometry": {
            "length_m": round(float(spec["veh"].L) * scale, 4),
            "diameter_m": round(float(spec["veh"].D) * scale, 4),
            "xcg_m": xref, "mesh": "mesh.json",
        },
    }
    import json
    with open(os.path.join(outdir, "vehicle.json"), "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"  aero {na}x{nm}, sref {cfg['components'][0]['sref_m2']} m^2, "
          f"length {cfg['geometry']['length_m']} m, xref {xref} m")

    # 5. Design the LQR gain schedule from THIS vehicle's OWN dynamics
    # (ADR-0004 C2): linearize the real f(x,u) with flightsim --linearize,
    # then run the LQR on that plant. Design mach grid = the vehicle's table
    # machs with the historic filter (skip near-zero + the transonic hole).
    dz = spec["design"]
    flightsim = next((p for p in
                      (os.path.join(PROJ, "build", "flightsim"),
                       os.path.join(PROJ, "build", "flightsim.exe"))
                      if os.path.exists(p)), None)
    if flightsim is None:
        print("  design SKIPPED: build/flightsim not found (cmake --build "
              "build first), gain_schedule.csv NOT regenerated")
        return
    import csv as _csv
    with open(os.path.join(outdir, "aero_tables.csv")) as f:
        rows = list(_csv.reader(f))
    mi = rows[0].index("mach")
    machs = sorted(set(float(r[mi]) for r in rows[1:]))
    machs = [m for m in machs if 0.15 <= m <= 3.0 and not (0.9 < m < 1.5)]
    r = subprocess.run(
        [flightsim, "--linearize", os.path.join(outdir, "vehicle.json"),
         "--altitude", str(dz["alt"]), "--mass", str(dz["mass"]),
         "--iyy", str(dz["iyy"]),
         "--machs", ",".join(f"{m:g}" for m in machs),
         "--out", os.path.join(outdir, "plant.csv")],
        capture_output=True, text=True)
    if r.returncode != 0:
        print("  linearize FAILED:\n  " + r.stderr.strip().replace("\n", "\n  "))
        return
    r = subprocess.run(
        [sys.executable, os.path.join(HERE, "design_autopilot.py"),
         "--plant", os.path.join(outdir, "plant.csv"),
         "--out", os.path.join(outdir, "gain_schedule.csv"),
         "--method", "lqr"]
        + (["--q-theta", str(dz["qt"])] if "qt" in dz else [])
        + (["--q-int", str(dz["qi"])] if "qi" in dz else [])
        + (["--r", str(dz["r"])] if "r" in dz else []),
        capture_output=True, text=True)
    print("  " + r.stdout.strip().replace("\n", "\n  "))
    if r.returncode != 0:
        print("  design FAILED:\n  " + r.stderr.strip().replace("\n", "\n  "))


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    names = list(MISSILES) if which == "all" else [which]
    for n in names:
        if n not in MISSILES:
            sys.exit(f"unknown missile '{n}' (have: {', '.join(MISSILES)})")
        build(n, MISSILES[n])


if __name__ == "__main__":
    main()
