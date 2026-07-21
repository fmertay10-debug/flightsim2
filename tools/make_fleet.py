"""Build the example FLEET: 13 ready-to-fly missile/rocket vehicles under
data/vehicles/fleet/<name>/, spanning airframes (tail-fin, canard+tail,
three-fin, boattail), effectors (fins, TVC gimbal, TVC+fin hybrid, none),
autopilots (auto-designed LQR, hand-tuned allocated_attitude, unguided), and
guidance assignments (pro_nav, pure_pursuit, flight-plan only -- recorded in
each vehicle.json's "role" block for the scenario builder).

Aerodynamics are genuine DATCOM output: each vehicle references one of the
parsed aero databases shipped in the repo (tools/datcom/examples/*/aero.npz
and data/vehicles/generated/*/aero.npz), geometrically scaled. The
coefficient tables are nondimensional, so scaling sref/cbar/bref rescales
the airframe exactly as DATCOM would at the same fidelity. Running the
DATCOM solver for NEW geometries needs tools/datcom/bin/DATCOM.exe (Windows;
wine on Linux) -- see make_missiles.py for that full pipeline.

Usage (venv python has numpy):
  .venv-tools/bin/python tools/make_fleet.py all
  .venv-tools/bin/python tools/make_fleet.py viper_aam dart_pd
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "datcom"))
sys.path.insert(0, HERE)

from pydatcom import Vehicle                          # noqa: E402
import datcom_export as dx                            # noqa: E402

OUT = os.path.join(PROJ, "data", "vehicles", "fleet")
EXAMPLES = os.path.join(HERE, "datcom", "examples")
GENERATED = os.path.join(PROJ, "data", "vehicles", "generated")
FT = 0.3048

# ---------------------------------------------------------------- databases
# Each parsed DATCOM database + the geometry of the airframe it was run on
# (native units; scaled per vehicle below). Meshes are rebuilt from this
# geometry at each vehicle's scale.

def _ex(name):
    return os.path.join(EXAMPLES, name, "aero.npz")

def _gen(name):
    return os.path.join(GENERATED, name, "aero.npz")

BASES = {
    # 27 ft example rocket family (ft)
    "ex_baseline": dict(npz=_ex("01_rocket_baseline"),
                        veh=dict(unit="FT")),                      # defaults
    "ex_fin":      dict(npz=_ex("02_rocket_fin_control"),
                        veh=dict(unit="FT")),
    "ex_boattail": dict(npz=_ex("03_rocket_boattail"),
                        veh=dict(unit="FT", L_af=21.0, L_bt=2.0, D_bt=0.8)),
    "ex_threefin": dict(npz=_ex("04_rocket_three_fin"),
                        veh=dict(unit="FT", fin_num=3)),
    "ex_canard":   dict(npz=_ex("05_canard_missile"),
                        veh=dict(unit="FT", canard_x=5.0, canard_root=1.2,
                                 canard_tip=0.6, canard_height=0.8,
                                 canard_sweep=30.0)),
    # the make_missiles airframes (m)
    "aam": dict(npz=_gen("aam"),
                veh=dict(unit="M", L=2.9, D=0.18, L_nc=0.55, L_af=2.35,
                         nc="ogive", xcg=1.55, fin_root=0.34, fin_tip=0.16,
                         fin_sweep=35.0, fin_height=0.20, fin_num=4)),
    "sam": dict(npz=_gen("sam"),
                veh=dict(unit="M", L=3.6, D=0.25, L_nc=0.75, L_af=2.85,
                         nc="ogive", xcg=1.95, fin_root=0.45, fin_tip=0.20,
                         fin_sweep=35.0, fin_height=0.26, fin_num=4)),
    "agm": dict(npz=_gen("agm"),
                veh=dict(unit="M", L=3.2, D=0.30, L_nc=0.70, L_af=2.50,
                         nc="ogive", xcg=1.60, fin_root=0.42, fin_tip=0.18,
                         fin_sweep=30.0, fin_height=0.24, fin_num=4)),
    "ssm": dict(npz=_gen("ssm"),
                veh=dict(unit="M", L=5.0, D=0.36, L_nc=1.10, L_af=3.90,
                         nc="ogive", xcg=2.80, fin_root=0.60, fin_tip=0.25,
                         fin_sweep=30.0, fin_height=0.34, fin_num=4)),
}

# ------------------------------------------------------------------- fleet
# gnc kinds:
#   lqr    -- auto-designed: --linearize -> design_autopilot --plant
#             (design: alt/mass/iyy + LQR weights qt/qi/r; min_v, max_fin)
#   alloc  -- hand-tuned allocated_attitude (gains/limits/actuator inline)
#   none   -- no control law at all (unguided ballistic)
FLEET = {
    "viper_aam": dict(
        base="aam", scale=1.0, blurb="agile short-range air-to-air missile",
        dry=52, prop=26, ixx=2.4, iyy=40,
        thrust=[[0.0, 21000], [0.25, 23000], [2.4, 20000], [2.8, 0]],
        gnc="lqr", design=dict(alt=7000, mass=65, iyy=40),
        max_fin=28, min_v=40, guidance="pro_nav"),

    "lance_lram": dict(
        base="aam", scale=1.25, blurb="long-range AAM, boost-sustain motor",
        dry=110, prop=58, ixx=6.0, iyy=130,
        thrust=[[0.0, 26000], [0.3, 28000], [2.0, 25000], [2.5, 9000],
                [8.0, 8000], [8.5, 0]],
        gnc="lqr", design=dict(alt=9000, mass=140, iyy=130),
        max_fin=24, min_v=35, guidance="pro_nav"),

    "aegis_sam": dict(
        base="sam", scale=1.0, blurb="area-defense surface-to-air missile",
        dry=102, prop=58, ixx=6.0, iyy=150,
        thrust=[[0.0, 52000], [0.3, 58000], [4.5, 48000], [5.0, 0]],
        gnc="lqr", design=dict(alt=4000, mass=130, iyy=150, qt=80, qi=8, r=40),
        max_fin=24, min_v=30, guidance="pro_nav"),

    "dart_pd": dict(
        base="sam", scale=0.62, blurb="point-defense interceptor",
        dry=34, prop=16, ixx=0.8, iyy=16,
        thrust=[[0.0, 14000], [0.2, 15000], [2.2, 12000], [2.6, 0]],
        gnc="lqr", design=dict(alt=1500, mass=42, iyy=16, r=120),
        max_fin=26, min_v=40, guidance="pure_pursuit"),

    "hammer_agm": dict(
        base="agm", scale=1.0, blurb="air-to-ground missile, top-attack dive",
        dry=118, prop=24, ixx=5.5, iyy=105,
        thrust=[[0.0, 16000], [0.3, 17000], [2.8, 14000], [3.2, 0]],
        gnc="lqr", design=dict(alt=800, mass=130, iyy=105),
        max_fin=25, min_v=35, guidance="pro_nav"),

    "harpy_ashm": dict(
        base="ssm", scale=0.9, blurb="anti-ship sea-skimmer, boost-sustain",
        dry=150, prop=90, ixx=11.0, iyy=300,
        thrust=[[0.0, 70000], [0.4, 76000], [3.5, 60000], [4.0, 20000],
                [12.0, 18000], [12.5, 0]],
        gnc="lqr", design=dict(alt=300, mass=195, iyy=300),
        max_fin=20, min_v=30, guidance="pro_nav"),

    "falx_canard": dict(
        base="ex_canard", scale=3.2 / (27 * FT),
        blurb="canard-destabilized interceptor (tail control)",
        dry=70, prop=30, ixx=2.2, iyy=60,
        thrust=[[0.0, 24000], [0.3, 26000], [3.2, 22000], [3.6, 0]],
        gnc="lqr", design=dict(alt=5000, mass=85, iyy=60),
        max_fin=25, min_v=35, guidance="pro_nav"),

    "corvus_trainer": dict(
        base="ex_canard", scale=2.4 / (27 * FT),
        blurb="training round, hand-tuned allocation autopilot",
        dry=38, prop=14, ixx=0.9, iyy=22,
        thrust=[[0.0, 9000], [0.2, 10000], [2.5, 8500], [3.0, 0]],
        gnc="alloc",
        gains=dict(pitch_kp=2200, pitch_kd=320, pitch_ki=180,
                   yaw_kp=2200, yaw_kd=320, yaw_ki=180,
                   roll_kp=40, roll_kd=18),
        limits=dict(max_ang_accel_dps2=12000, int_limit=1.0,
                    vertical_guard_deg=86),
        actuator=dict(tau_s=0.02, rate_dps=350, limit_deg=22),
        guidance="pure_pursuit"),

    "sentinel_sr": dict(
        base="ex_fin", scale=4.5 / (27 * FT),
        blurb="sounding rocket, pitch-program ballistic",
        dry=130, prop=90, ixx=4.0, iyy=260,
        thrust=[[0.0, 34000], [0.3, 37000], [6.5, 30000], [7.0, 0]],
        gnc="alloc",
        gains=dict(pitch_kp=1200, pitch_kd=250, pitch_ki=120,
                   yaw_kp=1200, yaw_kd=250, yaw_ki=120,
                   roll_kp=40, roll_kd=18),
        limits=dict(max_ang_accel_dps2=8000, int_limit=1.0,
                    vertical_guard_deg=86),
        actuator=dict(tau_s=0.02, rate_dps=300, limit_deg=20),
        guidance=None),

    "vulcan_hybrid": dict(
        base="ex_fin", scale=6.0 / (27 * FT),
        blurb="TVC + fin hybrid launcher (allocation blends the effectors)",
        dry=420, prop=260, ixx=30.0, iyy=1600,
        thrust=[[0.0, 95000], [0.3, 103000], [7.5, 88000], [8.0, 0]],
        gimbal=dict(nozzle_station_m=6.0, max_gimbal_deg=6),
        gnc="alloc",
        gains=dict(pitch_kp=150, pitch_kd=60, pitch_ki=30,
                   yaw_kp=150, yaw_kd=60, yaw_ki=30,
                   roll_kp=40, roll_kd=15),
        limits=dict(max_ang_accel_dps2=6000, int_limit=1.0,
                    vertical_guard_deg=84),
        actuator=dict(tau_s=0.03, rate_dps=200, limit_deg=20,
                      gimbal_limit_deg=6),
        guidance=None),

    "atlas_tvc": dict(
        base="ex_baseline", scale=7.0 / (27 * FT),
        blurb="pure-TVC booster (no aero control surfaces)",
        dry=520, prop=300, ixx=40.0, iyy=2600,
        thrust=[[0.0, 120000], [0.3, 130000], [8.5, 110000], [9.0, 0]],
        gimbal=dict(nozzle_station_m=7.0, max_gimbal_deg=8),
        gnc="alloc",
        gains=dict(pitch_kp=150, pitch_kd=60, pitch_ki=30,
                   yaw_kp=150, yaw_kd=60, yaw_ki=30,
                   roll_kp=40, roll_kd=15),
        limits=dict(max_ang_accel_dps2=6000, int_limit=1.0,
                    vertical_guard_deg=84),
        actuator=dict(tau_s=0.03, rate_dps=120, gimbal_limit_deg=8),
        guidance=None),

    "triax_probe": dict(
        base="ex_threefin", scale=3.5 / (27 * FT),
        blurb="three-fin research probe steered by TVC",
        dry=60, prop=35, ixx=1.5, iyy=55,
        thrust=[[0.0, 14000], [0.25, 15000], [4.5, 12000], [5.0, 0]],
        gimbal=dict(nozzle_station_m=3.5, max_gimbal_deg=7),
        gnc="alloc",
        gains=dict(pitch_kp=400, pitch_kd=120, pitch_ki=60,
                   yaw_kp=400, yaw_kd=120, yaw_ki=60,
                   roll_kp=40, roll_kd=15),
        limits=dict(max_ang_accel_dps2=9000, int_limit=1.0,
                    vertical_guard_deg=84),
        actuator=dict(tau_s=0.03, rate_dps=150, gimbal_limit_deg=7),
        guidance=None),

    "javelin_ballistic": dict(
        base="ex_boattail", scale=3.0 / (27 * FT),
        blurb="unguided boattail artillery rocket (aero stability only)",
        dry=45, prop=30, ixx=1.0, iyy=30,
        thrust=[[0.0, 30000], [0.2, 33000], [1.8, 28000], [2.0, 0]],
        gnc="none", guidance=None),
}


def scaled_vehicle(base_veh, s):
    """A pydatcom Vehicle in METERS: the base geometry x unit x scale."""
    v = dict(base_veh)
    unit = v.pop("unit")
    u = FT if unit == "FT" else 1.0
    geom = Vehicle(**{**v, "unit": unit})
    k = u * s
    fields = ["L", "D", "L_nc", "L_af", "L_bt", "D_bt", "r_n", "xcg",
              "fin_root", "fin_tip", "fin_height", "fin_disp"]
    canard = ["canard_x", "canard_root", "canard_tip", "canard_height"]
    out = Vehicle(unit="M", nc=geom.nc, fin_num=geom.fin_num,
                  fin_sweep=geom.fin_sweep, fin_shape=geom.fin_shape,
                  canard_sweep=geom.canard_sweep)
    for f in fields:
        setattr(out, f, getattr(geom, f) * k)
    if geom.has_canard:
        for f in canard:
            setattr(out, f, getattr(geom, f) * k)
    return out


def design_machs(d):
    machs = sorted(float(m) for m in np.asarray(d["mach"], float))
    return [m for m in machs if 0.15 <= m <= 3.0 and not (0.9 < m < 1.5)]


def build(name, spec):
    print(f"\n=== {name} ===  ({spec['blurb']})")
    base = BASES[spec["base"]]
    d = np.load(base["npz"], allow_pickle=True)
    u = dx.unit_scale(d)
    s = spec["scale"]
    k = u * s

    outdir = os.path.join(OUT, name)
    os.makedirs(outdir, exist_ok=True)

    # Aero tables straight from the parsed DATCOM database (nondimensional --
    # geometric scale enters only through the reference quantities below).
    na, nm = dx.write_aero_tables(d, outdir)
    controlled = dx.write_control_tables(d, outdir)

    # Mass: impulse-proportional drain at the thrust-curve breakpoints.
    dx.write_simple_mass_props(os.path.join(outdir, "mass_props.csv"),
                               spec["thrust"], spec["prop"], spec["dry"],
                               spec["ixx"], spec["iyy"], spec["iyy"])

    # Mesh from the scaled geometry.
    dx.write_mesh(scaled_vehicle(base["veh"], s), 1.0, outdir)

    sref = round(float(d["sref"]) * k * k, 6)
    cbar = round(float(d["cbar"]) * k, 4)
    bref = round(float(d["blref"]) * k, 4)
    xref = round(float(d["xcg"]) * k, 4)

    motor = {"type": "solid_motor", "propellant_kg": spec["prop"],
             "thrust_curve": spec["thrust"]}
    if "gimbal" in spec:
        motor["gimbal"] = spec["gimbal"]

    cfg = {
        "role": {
            "class": spec["blurb"],
            "guidance": spec["guidance"],
            "autopilot": spec["gnc"],
            "effectors": ("fins+tvc" if "gimbal" in spec and controlled else
                          "tvc" if "gimbal" in spec else
                          "fins" if controlled else "none"),
        },
        "mass": {"model": "tabulated", "table": "mass_props.csv"},
        "components": [
            {
                "type": "rocket_table_aero",
                "sref_m2": sref, "cbar_m": cbar, "bref_m": bref,
                "tables_csv": "aero_tables.csv",
                "control_csv": "control_tables.csv",
                "xref_m": xref,
            },
            motor,
        ],
        "geometry": {
            "length_m": round(cbar, 4), "diameter_m": round(bref, 4),
            "xcg_m": xref, "mesh": "mesh.json",
        },
    }

    if spec["gnc"] == "lqr":
        cfg["gnc"] = {
            "control_law": {
                "type": "lqr",
                "schedule": "gain_schedule.csv",
                "limits": {"max_ang_accel_dps2": 40000,
                           "min_airspeed_ms": spec["min_v"]},
                "roll": {"kp": 40, "kd": 15},
            },
            "actuator": {"tau_s": 0.02, "rate_dps": 450,
                         "limit_deg": spec["max_fin"]},
        }
    elif spec["gnc"] == "alloc":
        cfg["gnc"] = {
            "control_law": {
                "type": "allocated_attitude",
                "gains": spec["gains"],
                "limits": spec["limits"],
            },
            "actuator": spec["actuator"],
        }
    # gnc == "none": no block at all (unguided; zero control input).

    vpath = os.path.join(outdir, "vehicle.json")
    with open(vpath, "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"  aero {na}x{nm} ({'controlled' if controlled else 'uncontrolled'}), "
          f"sref {sref} m^2, length {cbar} m, xref {xref} m")

    flightsim = next((p for p in
                      (os.path.join(PROJ, "build", "flightsim"),
                       os.path.join(PROJ, "build", "flightsim.exe"))
                      if os.path.exists(p)), None)

    # Auto-design the LQR schedule from THIS vehicle's own dynamics.
    if spec["gnc"] == "lqr":
        if flightsim is None:
            print("  design SKIPPED: build/flightsim missing")
            return
        dz = spec["design"]
        machs = design_machs(d)
        plant = os.path.join(outdir, "plant.csv")
        r = subprocess.run(
            [flightsim, "--linearize", vpath,
             "--altitude", str(dz["alt"]), "--mass", str(dz["mass"]),
             "--iyy", str(dz["iyy"]),
             "--machs", ",".join(f"{m:g}" for m in machs),
             "--out", plant],
            capture_output=True, text=True)
        if r.returncode != 0:
            print("  linearize FAILED:\n  " +
                  r.stderr.strip().replace("\n", "\n  "))
            return
        cmd = [sys.executable, os.path.join(HERE, "design_autopilot.py"),
               "--plant", plant,
               "--out", os.path.join(outdir, "gain_schedule.csv")]
        for k2, flag in (("qt", "--q-theta"), ("qi", "--q-int"), ("r", "--r")):
            if k2 in dz:
                cmd += [flag, str(dz[k2])]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print("  design FAILED:\n  " + (r.stdout + r.stderr).strip())
            return
        tail = [ln for ln in r.stdout.strip().splitlines() if ln.strip()]
        print("  design: " + (tail[-1] if tail else "ok"))

    # Load-validate through the real loader (--describe on a probe scenario).
    if flightsim is not None:
        probe = {
            "name": f"_probe_{name}",
            "simulation": {"dt_s": 0.005, "duration_s": 0.01},
            "environment": {"gravity": "flat", "wind": {"model": "none"}},
            "vehicles": [{
                "name": name,
                "vehicle": os.path.relpath(vpath,
                                           os.path.join(PROJ, "data", "output")),
                "dynamics": "six_dof",
                "initial": {"position_m": [0, 0, -1000],
                            "velocity_ms": [200, 0, 0]},
            }],
        }
        ppath = os.path.join(PROJ, "data", "output", f"_probe_{name}.json")
        with open(ppath, "w") as f:
            json.dump(probe, f)
        r = subprocess.run([flightsim, "--describe", ppath],
                           capture_output=True, text=True)
        print("  load check: " + ("OK" if r.returncode == 0 else
                                  "FAILED\n  " + r.stderr.strip()))


def main():
    picks = sys.argv[1:] or ["all"]
    names = list(FLEET) if picks == ["all"] else picks
    for n in names:
        build(n, FLEET[n])
    print(f"\nfleet: {len(names)} vehicle(s) under {OUT}")


if __name__ == "__main__":
    main()
