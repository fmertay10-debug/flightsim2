# tools/

Python helpers (run with `py`, needs numpy; `design_autopilot.py` also needs
scipy + the `control` package). They sit *beside* the C++ sim — they generate
config/data the sim reads, and read the sim's CSV logs. None is required to
build or run the simulator.

## datcom_export.py — DATCOM aero → vehicle

Turns a parsed DATCOM `aero.npz` into a ready-to-fly vehicle folder (aero
tables, control tables, `vehicle.json`, 3-D `mesh.json`).

```
py tools/datcom_export.py --list                              # shipped examples
py tools/datcom_export.py 02_rocket_fin_control --name my_rocket
py tools/datcom_export.py 02_rocket_fin_control --name adv --variable --mass-kg 85
```

`--variable` additionally emits `mass_props.csv` (mass / inertia / **CG** vs
time) and `thrust.csv` (thrust vs time) and wires the vehicle to use them —
the "variable everything" rocket, whose CG migrates across the aero reference
so its static margin changes through the burn.

## design_autopilot.py — auto-design a gain-scheduled autopilot

Linearizes a DATCOM-table rocket from its own aero + mass at a Mach sweep and
synthesizes pitch gains by **LQR** (default) or **pole placement**, written as
`gain_schedule.csv` for the C++ `ScheduledController`.

```
py tools/design_autopilot.py vehicles/generated/datcom_rocket/vehicle.json \
    --mass 72 --iyy 480 --altitude 3000 --method lqr
py tools/design_autopilot.py <vehicle.json> --method place --wn 12 --zeta 0.7
```

Tuning knobs: `--q-theta`, `--q-int`, `--r` (LQR weights), `--wn`, `--zeta`
(pole placement). It prints each Mach point's open- and closed-loop modes so
you can see the design. See `docs/BUILDING_VEHICLES.md` for the full workflow.

## visualize.py (+ meshes.py) — 3-D HTML viewer

Turns a scenario's CSV logs into a self-contained HTML page (no libraries, no
server) with synced, orbit-able **trajectory** and **attitude** views, flying
each vehicle as its real 3-D model.

```
./build/flightsim scenarios/lqr_rocket_launch.json    # writes the logs
py tools/visualize.py scenarios/lqr_rocket_launch.json # -> output/.../view.html
```

## make_missiles.py — build the missile examples end-to-end

Builds the four missile classes (AAM / SAM / AGM / SSM) the WHOLE way: defines
each airframe geometry, runs DATCOM over a Mach sweep with fin control, exports
the tables, writes the vehicle, and auto-designs its LQR autopilot. Needs
`bin/DATCOM.exe`.

```
py tools/make_missiles.py all        # or: aam | sam | agm | ssm
```

## analyze.py — motion / control-tracking analysis

Reads a scenario CSV log and reports step-response metrics (rise, overshoot,
settling, steady-state error), oscillation/limit-cycle detection, and the
flight envelope. `--html` writes time-history plots; `--tmax`/`--tmin` restrict
to the controlled phase (e.g. before a rocket's ballistic descent).

```
py tools/analyze.py output/f16_cruise/viper.csv --html
py tools/analyze.py output/lqr_rocket_launch/lqr-rocket.csv --tmax 14
```

## monte_carlo.py — dispersion / hit-probability study

Runs an intercept scenario many times with randomized launch aim, target
location, wind, and motor thrust (via `flightsim --json`), then aggregates hit
probability (Pk), miss statistics, CEP/R90, and the 2-D miss dispersion into a
self-contained HTML report.

```
py tools/monte_carlo.py scenarios/aam_intercept.json --runs 300
py tools/monte_carlo.py scenarios/sam_intercept.json --runs 200 \
    --sigma-aim-deg 3 --sigma-target-m 100 --sigma-thrust 0.06
```
