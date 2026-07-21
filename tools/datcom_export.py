"""Generate a flightsim2 vehicle from parsed DATCOM data.

Reads a pydatcom aero .npz (or a shipped example by name), converts to SI /
per-radian, and writes a ready-to-fly vehicle folder:

    vehicles/generated/<name>/
        aero_tables.csv     alpha_rad, mach, CN, CA, CM, CMQ, CNR, CLP, CNB, CYB
        control_tables.csv  delta_rad, mach, dCM_sym, dCL_sym, Cl_roll
        vehicle.json        full flightsim2 vehicle definition (type "rocket")
        mesh.json           3-D geometry (for the visualizer), SI meters

DATCOM gives aero + geometry only -- mass properties are estimated from a
slender-body model (Iyy = Izz = m L^2/12, Ixx = m r^2/2) unless overridden.

Usage:
    py tools/datcom_export.py 02_rocket_fin_control --name datcom_rocket
    py tools/datcom_export.py path/to/aero.npz --name my_rocket --mass-kg 85
    py tools/datcom_export.py --list
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(PROJ, "datcom"))

from pydatcom import Vehicle, define_example_rocket, define_example_canard_missile  # noqa: E402
from pydatcom.geometry import vehicle_mesh  # noqa: E402

FT2M = 0.3048
DEG2RAD = math.pi / 180.0

# Shipped examples: npz path + the Vehicle geometry that produced it
# (geometry is needed for the 3-D mesh and the body diameter).
EXAMPLES = {
    "01_rocket_baseline":    (define_example_rocket, None),
    "02_rocket_fin_control": (define_example_rocket, None),
    "03_rocket_boattail":    (lambda: Vehicle(L_af=21.0, L_bt=2.0, D_bt=0.8), None),
    "04_rocket_three_fin":   (lambda: Vehicle(fin_num=3), None),
    "05_canard_missile":     (define_example_canard_missile, None),
}


def unit_scale(d):
    """Length scale factor to meters, from the npz 'dim' field."""
    dim = str(d["dim"]).strip().lower()
    return 1.0 if dim.startswith("m") else FT2M


def write_aero_tables(d, outdir):
    alpha = np.asarray(d["alpha"], float) * DEG2RAD
    mach = np.asarray(d["mach"], float)
    cn, ca, cm = d["cn"], d["ca"], d["cm"]
    cmq, cnr, clp = d["cmq"], d["cnr"], d["clp"]     # rate derivatives: per-radian
    cnb = d["cnb"] / DEG2RAD                          # per-deg -> per-rad
    cyb = d["cyb"] / DEG2RAD

    path = os.path.join(outdir, "aero_tables.csv")
    with open(path, "w") as f:
        f.write("alpha_rad,mach,CN,CA,CM,CMQ,CNR,CLP,CNB,CYB\n")
        for i in range(len(alpha)):
            for j in range(len(mach)):
                f.write(f"{alpha[i]:.6f},{mach[j]:.4f},"
                        f"{cn[i, j]:.6f},{ca[i, j]:.6f},{cm[i, j]:.6f},"
                        f"{cmq[i, j]:.6f},{cnr[i, j]:.6f},{clp[i, j]:.6f},"
                        f"{cnb[i, j]:.6f},{cyb[i, j]:.6f}\n")
    return len(alpha), len(mach)


def write_control_tables(d, outdir):
    """Control increments vs (deflection, Mach); zero tables when the DATCOM
    run had no deflection schedule (uncontrolled airframe)."""
    mach = np.asarray(d["mach"], float)
    ndelta = int(d["ndelta"]) if "ndelta" in d else 0

    if ndelta > 0:
        delta = np.asarray(d["delta"], float) * DEG2RAD
        dcm = np.asarray(d["dcm_sym"], float)
        dcl = np.asarray(d["dcl_sym"], float)
        clr = np.asarray(d["clroll"], float) if "clroll" in d else np.zeros_like(dcm)
        if clr.shape != dcm.shape:
            clr = np.zeros_like(dcm)
    else:
        delta = np.array([-0.35, 0.0, 0.35])
        dcm = np.zeros((3, len(mach)))
        dcl = np.zeros((3, len(mach)))
        clr = np.zeros((3, len(mach)))

    path = os.path.join(outdir, "control_tables.csv")
    with open(path, "w") as f:
        f.write("delta_rad,mach,dCM_sym,dCL_sym,Cl_roll\n")
        for i in range(len(delta)):
            for j in range(len(mach)):
                f.write(f"{delta[i]:.6f},{mach[j]:.4f},"
                        f"{dcm[i, j]:.6f},{dcl[i, j]:.6f},{clr[i, j]:.6f}\n")
    return ndelta > 0


def write_mesh(veh, scale, outdir):
    """Vehicle 3-D mesh in SI meters, x measured from the nose tip."""
    parts = []
    for name, verts, faces in vehicle_mesh(veh):
        parts.append({
            "name": name,
            "vertices": (np.asarray(verts, float) * scale).round(4).tolist(),
            "faces": np.asarray(faces, int).tolist(),
        })
    with open(os.path.join(outdir, "mesh.json"), "w") as f:
        json.dump({"units": "m", "parts": parts}, f)


def estimate_mass(length_m, diameter_m):
    """Scale from the reference 85 kg / 8.23 m x 0.406 m example by volume."""
    ref = 8.2296 * 0.4064 ** 2
    return 85.0 * (length_m * diameter_m ** 2) / ref


def thrust_curve_points(mass_kg, burn=5.5):
    """A made-up but sensible boost profile sized for ~8 g liftoff [N]."""
    t0 = round(8.0 * 9.80665 * mass_kg, -1)
    return [(0.0, t0), (0.2, 1.15 * t0), (5.0, 1.05 * t0), (burn, 0.0)]


def write_simple_mass_props(path, thrust_curve, propellant, dry, ixx, iyy, izz):
    """Tabulated mass CSV without CG travel: impulse-proportional drain
    sampled at the thrust-curve breakpoints (exact -- the drain is piecewise
    linear between them), inertia held at the dry values."""
    t = [p[0] for p in thrust_curve]
    f = [p[1] for p in thrust_curve]
    imp = [0.0]
    for i in range(1, len(t)):
        imp.append(imp[-1] + 0.5 * (f[i] + f[i - 1]) * (t[i] - t[i - 1]))
    total = imp[-1]
    with open(path, "w") as out:
        out.write("time_s,mass_kg,ixx,iyy,izz\n")
        for i in range(len(t)):
            mass = dry + propellant * (1.0 - imp[i] / total)
            out.write(f"{t[i]:.10g},{mass:.10g},{ixx:.10g},{iyy:.10g},{izz:.10g}\n")


def write_variable_tables(outdir, mass_kg, length_m, diameter_m, xref_m, burn=5.5):
    """Emit thrust.csv (thrust vs time) and mass_props.csv (mass, inertia, CG
    vs time) as lookup tables -- the 'variable everything' rocket. Profiles are
    physically consistent but invented: propellant depletes in proportion to
    delivered impulse, dropping mass and inertia and moving the CG forward as
    the aft propellant burns off (raising static margin through the burn)."""
    dry = 0.7 * mass_kg
    prop = mass_kg - dry
    curve = thrust_curve_points(mass_kg, burn)

    # thrust.csv
    with open(os.path.join(outdir, "thrust.csv"), "w") as f:
        f.write("time_s,thrust_n\n")
        for t, thr in curve:
            f.write(f"{t:.3f},{thr:.1f}\n")

    # Cumulative delivered-impulse fraction on a fine grid -> mass(t).
    fine = np.linspace(0.0, burn, 400)
    ct, cf = zip(*curve)
    thr_fine = np.interp(fine, ct, cf)
    impulse = np.concatenate([[0.0], np.cumsum(
        0.5 * (thr_fine[1:] + thr_fine[:-1]) * np.diff(fine))])
    impulse /= impulse[-1]

    # CG travels from aft (loaded) to forward (empty) about the aero reference.
    xcg_wet = xref_m + 0.06 * length_m
    xcg_dry = xref_m - 0.05 * length_m

    grid = list(np.arange(0.0, burn + 1e-9, 0.25)) + [burn + 0.5]
    with open(os.path.join(outdir, "mass_props.csv"), "w") as f:
        f.write("time_s,mass_kg,ixx,iyy,izz,xcg_m\n")
        for t in grid:
            frac = float(np.interp(min(t, burn), fine, impulse))  # impulse frac
            pf = 1.0 - frac                                       # propellant left
            m = dry + prop * pf
            iyy = m * length_m ** 2 / 12.0
            ixx = m * (diameter_m / 2.0) ** 2 / 2.0
            xcg = xcg_dry + (xcg_wet - xcg_dry) * pf
            f.write(f"{t:.3f},{m:.3f},{ixx:.4f},{iyy:.3f},{iyy:.3f},{xcg:.4f}\n")


def build_vehicle_json(d, veh, scale, mass_kg, controlled, outdir, variable=False):
    length_m = float(veh.L) * scale
    diameter_m = float(veh.D) * scale
    xref_m = round(float(d["xcg"]) * scale, 4)   # DATCOM moment reference station
    dry = 0.7 * mass_kg
    prop = 0.3 * mass_kg

    # Slender-body inertia at total mass (constant through the burn).
    iyy = mass_kg * length_m ** 2 / 12.0
    ixx = mass_kg * (diameter_m / 2.0) ** 2 / 2.0

    # Solid motor sized for ~8 g liftoff acceleration over a 5.5 s burn.
    thrust = round(8.0 * 9.80665 * mass_kg, -1)

    aero = {
        "type": "rocket_table_aero",
        "sref_m2": round(float(d["sref"]) * scale ** 2, 6),
        "cbar_m": round(float(d["cbar"]) * scale, 4),
        "bref_m": round(float(d["blref"]) * scale, 4),
        "tables_csv": "aero_tables.csv",
        "control_csv": "control_tables.csv",
    }
    cfg = {}

    if variable:
        # Lego blocks: tabulated mass (mass/inertia/CG vs time) + tabulated
        # thrust, and the aero moment reference so CG travel changes the
        # static margin through the burn.
        write_variable_tables(outdir, mass_kg, length_m, diameter_m, xref_m)
        aero["xref_m"] = xref_m
        cfg["mass"] = {"model": "tabulated", "table": "mass_props.csv"}
        motor = {"type": "tabulated_thrust", "table": "thrust.csv"}
    else:
        # Tabulated mass without CG travel: rows at the thrust-curve
        # breakpoints, impulse-proportional propellant drain on top of the
        # dry mass, inertia held at the dry values.
        curve = [[0.0, thrust], [0.2, thrust * 1.15],
                 [5.0, thrust * 1.05], [5.5, 0.0]]
        write_simple_mass_props(os.path.join(outdir, "mass_props.csv"),
                                curve, round(prop, 2), round(dry, 2),
                                round(ixx, 3), round(iyy, 1), round(iyy, 1))
        cfg["mass"] = {"model": "tabulated", "table": "mass_props.csv"}
        motor = {
            "type": "solid_motor",
            "propellant_kg": round(prop, 2),   # generator metadata (sizes the drain)
            "thrust_curve": curve,
        }
    cfg["components"] = [aero, motor]

    if controlled:
        cfg["gnc"] = {
            "control_law": {
                # WrenchCommand + Allocator (ADR-0004); gains are the shared
                # rocket set proven on the datcom/sounding rockets.
                "type": "allocated_attitude",
                "gains": {
                    "pitch_kp": 150, "pitch_kd": 60, "pitch_ki": 30,
                    "yaw_kp": 150, "yaw_kd": 60, "yaw_ki": 30,
                    "roll_kp": 40, "roll_kd": 15,
                },
                "limits": {"max_ang_accel_dps2": 6000, "int_limit": 1.0,
                           "vertical_guard_deg": 84},
            },
            "actuator": {"tau_s": 0.03, "rate_dps": 200, "limit_deg": 15},
        }

    cfg["geometry"] = {   # informational + used by the visualizer
        "length_m": round(length_m, 4),
        "diameter_m": round(diameter_m, 4),
        "xcg_m": xref_m,
        "mesh": "mesh.json",
    }

    path = os.path.join(outdir, "vehicle.json")
    with open(path, "w") as f:
        json.dump(cfg, f, indent=2)
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", nargs="?",
                    help="example name (see --list) or path to an aero .npz")
    ap.add_argument("--name", help="output vehicle name (default: source name)")
    ap.add_argument("--mass-kg", type=float,
                    help="total liftoff mass (default: volume-scaled estimate)")
    ap.add_argument("--diameter-ft", type=float, default=16 / 12,
                    help="body diameter for non-example npz sources [ft]")
    ap.add_argument("--out", default=os.path.join(PROJ, "vehicles", "generated"),
                    help="output root directory")
    ap.add_argument("--variable", action="store_true",
                    help="emit variable mass/inertia/CG (mass_props.csv) and "
                         "variable thrust (thrust.csv) lookup tables, and wire "
                         "the vehicle to use them (CG travel -> static margin)")
    ap.add_argument("--list", action="store_true", help="list shipped examples")
    args = ap.parse_args()

    if args.list or not args.source:
        print("shipped DATCOM examples:")
        for name in EXAMPLES:
            print(f"  {name}")
        return

    if args.source in EXAMPLES:
        factory, _ = EXAMPLES[args.source]
        veh = factory()
        npz = os.path.join(PROJ, "datcom", "examples", args.source, "aero.npz")
        name = args.name or args.source
    else:
        npz = args.source
        veh = Vehicle(D=args.diameter_ft)   # generic geometry for mesh/diameter
        name = args.name or os.path.splitext(os.path.basename(npz))[0]

    d = np.load(npz, allow_pickle=True)
    scale = unit_scale(d)
    veh.L = float(d["cbar"])                 # cbar IS body length in this deck
    veh.xcg = float(d["xcg"])

    mass = args.mass_kg or estimate_mass(veh.L * scale, veh.D * scale)

    outdir = os.path.join(args.out, name)
    os.makedirs(outdir, exist_ok=True)

    na, nm = write_aero_tables(d, outdir)
    controlled = write_control_tables(d, outdir)
    write_mesh(veh, scale, outdir)
    cfg = build_vehicle_json(d, veh, scale, mass, controlled, outdir, args.variable)

    print(f"wrote {outdir}")
    print(f"  aero_tables.csv     {na} alpha x {nm} mach")
    print(f"  control_tables.csv  {'from DATCOM deltas' if controlled else 'ZERO (uncontrolled airframe)'}")
    if args.variable:
        print(f"  thrust.csv          variable thrust vs time")
        print(f"  mass_props.csv      variable mass/inertia/CG vs time (CG travel)")
        print(f"  vehicle.json        tabulated mass + tabulated thrust + DATCOM aero"
              f" (xref {cfg['components'][0]['xref_m']} m)")
    else:
        print(f"  vehicle.json        mass {mass:.1f} kg, "
              f"sref {cfg['components'][0]['sref_m2']} m^2, "
              f"length {cfg['geometry']['length_m']} m")
    print(f"  mesh.json           3-D geometry for the visualizer")


if __name__ == "__main__":
    main()
