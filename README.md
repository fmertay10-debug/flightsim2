# flightsim2

A modular, **scenario-driven** multi-vehicle flight dynamics simulator. C++17, stdlib only.

Successor to `flightsim`: same proven core (NED frames, quaternion attitude, ISA
atmosphere, Strategy/Factory/Observer architecture), but everything scenario-specific
now lives in **JSON config files** — no recompiling to change vehicles, initial
conditions, wind, or flight plans, and a scenario can fly any number of vehicles.

## Scope and data sources

This is a rigid-body flight dynamics simulator built from **published, publicly
available sources**. It models six-degree-of-freedom motion — forces, moments,
attitude, and the guidance and control loops that steer a vehicle along a
trajectory.

The data behind the shipped models:

- **Aerodynamics** — [USAF Digital DATCOM](https://en.wikipedia.org/wiki/DATCOM)
  (1976), the public-domain U.S. Government coefficient-prediction program.
  Vehicle aero comes from DATCOM runs on notional geometries, parsed into
  nondimensional coefficient tables.
- **F-16 model** — the wind-tunnel tables and turbofan model published in
  Stevens & Lewis, *Aircraft Control and Simulation* (Appendix A) and NASA
  TP-1538.
- **Validation cases** — the NASA Engineering and Safety Center 6-DOF check
  cases, NASA/TM-2015-218675.
- **Guidance** — proportional navigation and pure pursuit, as given in the
  standard textbook treatments (e.g. Zarchan, *Tactical and Strategic Missile
  Guidance*, AIAA).
- **Control** — LQR and pole placement via the Python `control` package.

**The vehicles are notional.** The airframes under `data/vehicles/fleet/` are
DATCOM example geometries, geometrically rescaled, with masses and inertias
estimated by slender-body approximation. Their names (`viper`, `javelin`,
`aegis`, `harpy`, `atlas`, …) are arbitrary labels chosen to make scenarios
readable — they are **not** models of the real-world systems whose names they
echo, and no performance claim about any real vehicle should be read into them.
There is no seeker model, warhead, fuzing, propellant chemistry, or hardware
interface anywhere in this repository; a "missile" here is a fin-stabilized
rigid body with a thrust curve and a control law.

## Build & run

Requires CMake + a C++17 compiler (MinGW/MSYS2 on Windows).

```
./build.ps1                                   # configure + build + test (Windows)
cmake -S . -B build && cmake --build build -j # Linux/macOS
./build/flightsim data/scenarios/aircraft_cruise.json
./build/flightsim data/scenarios/datcom_rocket_launch.json
./build/flightsim data/scenarios/mixed_traffic.json
```

Each vehicle with a `"log"` entry writes a CSV (state, controls, air data,
mass, thrust) you can plot with anything.

## Layout

```
data/
  scenarios/       scenario files: sim settings, environment, vehicle list, flight plans
  vehicles/        vehicle definitions: mass table + components[] + gnc block
    generated/     DATCOM-derived vehicles (built by tools/make_missiles.py etc.)
  output/          sim CSV logs + generated HTML viewers (gitignored)
tools/             Python: design/analysis/visualization tools (see tools/README.md)
  datcom/          vendored pydatcom: DATCOM output parser + example rockets (no ML)
src/
  math/            Vector3, Matrix3x3, Quaternion (scalar-first), lookup tables, units
  core/            shared currency structs: State, Channel(Table/Values), AirData, Telemetry
  io/              minimal JSON parser (// comments) + tidy-CSV / key-value reader
  environment/     ISA atmosphere; gravity + wind Strategies
  dynamics/        EOM Strategies: six_dof, kinematic (+ factory)
  component/       ForceComponent: the ONE force-producer contract
                   ((state, air, channels) -> body wrench + control
                   effectiveness dM/dchannel for allocation) + its registry
    aero/          AircraftAero (derivative), F16Aero (wind-tunnel tables),
                   RocketTableAero (DATCOM tables)
    propulsion/    Propulsor (motor + axial or gimbaled/TVC mount; owns the
                   throttle/tvc channels) over PropulsionModel motors:
                   solid_motor, tabulated_thrust, turbojet, f16_engine
  mass/            MassModel: tabulated (mass/inertia/CG vs time; constant = 2 rows)
  gnc/             the GNC stack. CommandSet vocabulary + per-channel ActuatorBank
                   at the root; registries keyed on explicit "type" strings
    control/       framework: ControlLaw iface + factory + building blocks
                   (Pid, Allocator: wrench -> channels by live effectiveness)
      laws/        the concrete laws: AllocatedAttitudeLaw, AircraftAllocatedLaw,
                   ScheduledLaw (designed gain schedules / LQR)
    guidance/      GuidanceLaw iface + ProNav3D, PurePursuit + FlightPlan
                   (scripted CommandSet segments)
  design/          Linearizer: trim + linearize the real f(x,u) (flightsim --linearize)
  vehicle/         Vehicle (mass + ForceComponent list + declared channels) + factory
  sim/             Simulation (two-phase multi-vehicle loop, intercept watch, CG
                   moment transfer), Entity, WorldView, observers, CsvLogger
  scenario/        ScenarioLoader: JSON -> registries -> ready-to-run Simulation
tests/             assert-based CTest suite (math, ISA, JSON, 6-DOF, channels,
                   actuators, validation, table aero, F-16 vs fixture, guidance,
                   end-to-end scenarios) + golden/ regression baselines
```

## Building vehicles (the Lego model) & control design

A vehicle is **mass + a components[] list + a gnc block**, snapped together in
one JSON file; every entry names its implementation explicitly via `"type"`,
each backed by a registry. You never touch `src/` to add a vehicle. The control
law is chosen independent of the airframe, and its gains can be
**auto-designed** from the vehicle's own aero + mass by LQR or pole placement
(`tools/design_autopilot.py`). Control commands flow as named channels that are
validated at load (a control law paired with an airframe that can't respond is
a load error, not a silent open loop). Full guide with a PID→LQR walkthrough:
**[docs/BUILDING_VEHICLES.md](docs/BUILDING_VEHICLES.md)**. Tool reference:
**[tools/README.md](tools/README.md)**.

Real-data vehicles included: the **F-16** (Stevens & Lewis / NASA TP-1538
wind-tunnel tables + turbofan model, aero validated against a reference fixture)
and DATCOM-derived rockets, including one with **variable thrust, mass, inertia,
and CG travel** (`data/scenarios/advanced_rocket_launch.json`).

**Control allocation** is how every law flies: the controller emits a desired
body wrench and the Allocator distributes it over whatever channels the
vehicle's components declare, weighted by each component's live control
effectiveness. Redundant effectors (e.g. a gimbaled nozzle plus fins) blend
automatically as flight condition changes — no mode switching, no per-phase
gains; the blend falls out of the effectiveness columns changing with qbar
and thrust.

## Interactive 3-D gallery

**[docs/gallery/index.html](docs/gallery/index.html)** — curated flights as
single offline HTML files (open in any browser, no install, no internet):
articulated 3-D animation (fins, control surfaces, and the TVC nozzle deflect
with the logged commands), toggleable overlays (body axes, velocity, α/β
arcs, line-of-sight with closing speed, thrust vector, exhaust plume, aero
force, setpoint ghost), orbit/follow/chase cameras, and preset telemetry
plots synced two-way with the animation — click a plot to jump the 3-D view
to that moment. Built by `python3 tools/make_gallery.py` (`py` on Windows);
any scenario gets the same viewer via
`python3 tools/visualize.py data/scenarios/<name>.json`.

**Missile examples** — AAM, SAM, AGM, and SSM, each built the whole way
(`tools/make_missiles.py`): a DATCOM aero run → tables → vehicle → auto-designed
LQR autopilot, flown to an intercept under proportional navigation
(`data/scenarios/{aam,sam,agm,ssm}_*.json`). All four hit.

**The fleet** (`data/vehicles/fleet/`, built by `tools/make_fleet.py`) —
13 more ready-to-fly missiles/rockets on the same DATCOM databases:
tail-fin, canard, three-fin, and boattail airframes; fins, TVC, TVC+fin
hybrid, and unguided effector suites; LQR-designed and hand-tuned
autopilots; each vehicle.json carries a `role` block naming its intended
guidance (`pro_nav` / `pure_pursuit` / flight-plan only).

**Analysis tools** (Python): `tools/analyze.py` reports control-tracking
metrics and oscillation detection from a run's log; `tools/monte_carlo.py` runs
a dispersion study over an intercept (randomized aim, target, wind, thrust) and
reports hit probability (Pk), CEP/R90, and a miss-scatter HTML. See
**[tools/README.md](tools/README.md)**.

## DATCOM vehicles & aerodynamics

Rockets fly with real USAF DATCOM aerodynamics. The `tools/datcom/` folder
vendors the `pydatcom` parser plus five example airframes. `RocketTableAero`
interpolates the coefficients bilinearly over the full (alpha, Mach) envelope
— coefficients genuinely vary with flight condition, valid to alpha =
+/-180 deg. Generate a ready-to-fly vehicle from DATCOM output:

```
python3 tools/datcom_export.py --list                     # shipped examples
python3 tools/datcom_export.py 02_rocket_fin_control --name datcom_rocket
# -> data/vehicles/generated/datcom_rocket/{aero_tables.csv,
#    control_tables.csv, mass_props.csv, vehicle.json, mesh.json}
```

## Guidance & intercepts

A vehicle can carry a `guidance` block that reads a named target from the
shared world snapshot and overlays the flight plan (its commands win on the
fields it sets). `pro_nav` (3D proportional navigation, skid-to-turn) and
`pure_pursuit` are built in. A scenario `end_conditions.intercept` block tracks
closest approach and ends the run on a hit. See
`data/scenarios/aam_intercept.json` (an air-to-air missile catching a
maneuvering bandit — reported HIT at ~7.5 m).

## Conventions (locked)

- Frames: inertial = NED (z down, altitude = `-z`); body = x fwd, y right, z down.
- Attitude: scalar-first quaternion, inertial → body, renormalized every step.
  Euler angles are a derived view (3-2-1 yaw–pitch–roll).
- Integration: forward Euler, small fixed dt (aero damping dominates).
- Units: SI + radians everywhere inside the sim. Config files use degrees
  (`*_deg`, `*_dps` keys) and are converted at the loading boundary.
- Two-phase stepping: all entities propagate from the same `WorldView` snapshot,
  then commit together — deterministic regardless of entity order.

## Adding a new vehicle CLASS (e.g. quadcopter)

The class-specific physics and control live in exactly two registrations:

1. **Force component(s)**: subclass `ForceComponent` (`src/component/`),
   declare the channels it consumes (e.g. four motor channels), report their
   control effectiveness, and register it:
   `component::Factory::registerComponent("quad_rotors", QuadRotors::fromJson);`
   (built-ins are registered in `ComponentFactory.cpp`).
2. **Control law** (often unnecessary — the allocation-based laws are
   airframe-agnostic): subclass `ControlLaw` (`src/gnc/control/`), bind the
   channels it writes, and register it:
   `gnc::Factory::registerControlLaw("quad_law", QuadLaw::fromJson);`
   (built-ins in `ControlLawFactory.cpp`).
3. Write a vehicle definition JSON listing your components and control law by
   those type names — the loader routes them to your classes and validates at
   load that every channel the control law writes has a consumer.

Nothing in `sim/`, `dynamics/`, or `scenario/` changes.

## Adding a new SCENARIO

Copy a file in `data/scenarios/`, edit. A scenario is: `simulation` (dt, duration),
`environment` (gravity: flat/spherical, wind: none/constant), and a `vehicles`
array. Each vehicle entry references a definition file (or embeds one under
`"definition"`), sets `dynamics`, `initial` state, an optional `flight_plan`
(time-ordered setpoint segments that merge cumulatively: pitch/roll/heading/
altitude/speed/throttle), and an optional CSV `log`. Paths in `"vehicle"` are
relative to the scenario file.

## Design patterns

Strategy (`EquationsOfMotion`, `ForceComponent`, `ControlLaw`,
`PropulsionModel`, `MassModel`, `GravityModel`, `WindModel`), registry-based
Factory (`component::Factory`, `gnc::Factory`, `guidance::Factory`,
`eom::create`), Observer (`SimObserver` → `CsvLogger`), Composition (`Entity` =
state + integrator + optional Vehicle + optional GNC stack; `Vehicle` = mass +
force components + declared channels).

## License

MIT — see [LICENSE](LICENSE). Third-party components (three.js, uPlot, and the
public-domain USAF Digital DATCOM binary) are listed in
[THIRD_PARTY.md](THIRD_PARTY.md).
