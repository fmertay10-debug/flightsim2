# flightsim2

A modular, **scenario-driven** multi-vehicle flight dynamics simulator. C++17, stdlib only.

Successor to `flightsim`: same proven core (NED frames, quaternion attitude, ISA
atmosphere, Strategy/Factory/Observer architecture), but everything scenario-specific
now lives in **JSON config files** — no recompiling to change vehicles, initial
conditions, wind, or flight plans, and a scenario can fly any number of vehicles.

## Build & run

Requires CMake + a C++17 compiler (MinGW/MSYS2 on Windows).

```
./build.ps1                                   # configure + build + test
./build/flightsim scenarios/aircraft_cruise.json
./build/flightsim scenarios/rocket_launch.json
./build/flightsim scenarios/mixed_traffic.json
```

Each vehicle with a `"log"` entry writes a CSV (state, controls, air data,
mass, thrust) you can plot with anything.

## Layout

```
scenarios/         scenario files: sim settings, environment, vehicle list, flight plans
vehicles/          vehicle definition files: mass, aero, propulsion, controller, actuator
  generated/       DATCOM-derived vehicles (built by tools/datcom_export.py)
datcom/            vendored pydatcom: DATCOM output parser + example rockets (no ML)
tools/             Python: datcom_export.py, visualize.py, meshes.py
output/            sim CSV logs + generated HTML viewers (gitignored)
src/
  math/            Vector3, Matrix3x3, Quaternion (scalar-first), lookup tables, units
  core/            shared currency structs: State, ControlInput, AirData, Telemetry
  io/              minimal JSON parser (// comments) + tidy-CSV / key-value reader
  environment/     ISA atmosphere; gravity + wind Strategies
  dynamics/        EOM Strategies: six_dof, point_mass, kinematic (+ factory)
  propulsion/      Strategies: turbojet, solid_motor, tabulated_thrust (thrust(t)),
                   f16_engine (idle/mil/max + power dynamics), none
  effector/        Effector Strategy (control -> body wrench): ThrustEffector
                   (axial), TvcEffector (gimbaled thrust); pluggable list on Entity
  mass/            MassModel Strategy: constant, tabulated (mass/inertia/CG vs time)
  aero/            AeroModel per vehicle type: AircraftAero, RocketAero (derivatives),
                   RocketTableAero (DATCOM tables), F16Aero (wind-tunnel tables),
                   registry factory; moment reference for CG travel
  control/         Controller per type: AircraftController, RocketController;
                   ScheduledController (LQR state feedback); TvcController (thrust
                   vectoring); PID, flight plans, actuators; factory dispatches on "method"
  guidance/        GuidanceLaw Strategy: ProNav3D, PurePursuit (+ factory) -- reads the
                   target from the WorldView, overlays the flight plan
  vehicle/         Vehicle (mass block + propulsion block) + factory
  sim/             Simulation (two-phase multi-vehicle loop, intercept watch, CG
                   moment transfer), Entity, WorldView, observers, CsvLogger
  scenario/        ScenarioLoader: JSON -> factories -> ready-to-run Simulation
tests/             assert-based CTest suite (math, ISA, JSON, 6-DOF, table aero,
                   F-16 vs fixture, guidance, end-to-end scenarios)
```

## Building vehicles (the Lego model) & control design

A vehicle is four independent blocks — `aero`, `mass`, `propulsion`,
`controller` — snapped together in one JSON file, each backed by a factory. You
never touch `src/` to add a vehicle. The control block is chosen by a `method`
field independent of the airframe, and its gains can be **auto-designed** from
the vehicle's own aero + mass by LQR or pole placement
(`tools/design_autopilot.py`). Full guide with a PID→LQR walkthrough:
**[docs/BUILDING_VEHICLES.md](docs/BUILDING_VEHICLES.md)**. Tool reference:
**[tools/README.md](tools/README.md)**.

Real-data vehicles included: the **F-16** (Stevens & Lewis / NASA TP-1538
wind-tunnel tables + turbofan model, aero validated against a reference fixture)
and DATCOM-derived rockets, including one with **variable thrust, mass, inertia,
and CG travel** (`scenarios/advanced_rocket_launch.json`).

**Missile examples** — AAM, SAM, AGM, and SSM, each built the whole way
(`tools/make_missiles.py`): a DATCOM aero run → tables → vehicle → auto-designed
LQR autopilot, flown to an intercept under proportional navigation
(`scenarios/{aam,sam,agm,ssm}_*.json`). All four hit.

**Analysis tools** (Python): `tools/analyze.py` reports control-tracking
metrics and oscillation detection from a run's log; `tools/monte_carlo.py` runs
a dispersion study over an intercept (randomized aim, target, wind, thrust) and
reports hit probability (Pk), CEP/R90, and a miss-scatter HTML. See
**[tools/README.md](tools/README.md)**.

## DATCOM vehicles & aerodynamics

Rockets can fly with real USAF DATCOM aerodynamics instead of point derivatives.
The `datcom/` folder vendors the `pydatcom` parser plus five example airframes.
`RocketTableAero` interpolates the coefficients bilinearly over the full
(alpha, Mach) envelope — coefficients genuinely vary with flight condition,
valid to alpha = +/-180 deg. Generate a ready-to-fly vehicle from DATCOM output:

```
py tools/datcom_export.py --list                          # shipped examples
py tools/datcom_export.py 02_rocket_fin_control --name datcom_rocket
# -> vehicles/generated/datcom_rocket/{aero_tables.csv, control_tables.csv,
#    vehicle.json, mesh.json}
```

A rocket vehicle uses table aero automatically when its `aero` block has a
`tables_csv` key; otherwise it uses the point-derivative `RocketAero`.

## Guidance & intercepts

A vehicle can carry a `guidance` block that reads a named target from the
shared world snapshot and overlays the flight plan (its commands win on the
fields it sets). `pro_nav` (3D proportional navigation, skid-to-turn) and
`pure_pursuit` are built in. A scenario `end_conditions.intercept` block tracks
closest approach and ends the run on a hit. See `scenarios/intercept.json`
(an agile missile catching a maneuvering drone — reported HIT at ~8 m).

## 3-D visualization

Turn any run's CSV logs into a self-contained HTML viewer (no libraries, no
server, opens in any browser) with two synced, orbit-able views: a **trajectory**
view flying every vehicle as its real 3-D model along its path, and an
**attitude** view spinning a chosen vehicle's model at its true attitude with a
reference triad and velocity vector.

```
./build/flightsim scenarios/intercept.json     # writes the CSV logs
py tools/visualize.py scenarios/intercept.json # -> output/intercept/view.html
```

## Conventions (locked)

- Frames: inertial = NED (z down, altitude = `-z`); body = x fwd, y right, z down.
- Attitude: scalar-first quaternion, inertial → body, renormalized every step.
  Euler angles are a derived view (3-2-1 yaw–pitch–roll).
- Integration: forward Euler, small fixed dt (aero damping dominates).
- Units: SI + radians everywhere inside the sim. Config files use degrees
  (`*_deg`, `*_dps` keys) and are converted at the loading boundary.
- Two-phase stepping: all entities propagate from the same `WorldView` snapshot,
  then commit together — deterministic regardless of entity order.

## Adding a new vehicle TYPE (e.g. quadcopter)

The per-type physics and control live in exactly two classes:

1. **Aero**: subclass `AeroModel` (`src/aero/`), give it a
   `static fromJson(const json::Value&)` builder, register it:
   `aero::Factory::registerModel("quadcopter", QuadAero::fromJson);`
   (built-ins are registered in `AeroFactory.cpp`).
2. **Controller**: subclass `Controller` (`src/control/`), same pattern:
   `control::Factory::registerController("quadcopter", QuadController::fromJson);`
   (built-ins in `ControllerFactory.cpp`).
3. Write a vehicle definition JSON with `"type": "quadcopter"` and your own
   `aero` / `controller` blocks — the loader routes them to your classes.
   New propulsion? Add a Strategy in `src/propulsion/` and a branch in
   `propulsion::create`.

Nothing in `sim/`, `dynamics/`, or `scenario/` changes.

## Adding a new SCENARIO

Copy a file in `scenarios/`, edit. A scenario is: `simulation` (dt, duration),
`environment` (gravity: flat/spherical, wind: none/constant), and a `vehicles`
array. Each vehicle entry references a definition file (or embeds one under
`"definition"`), sets `dynamics`, `initial` state, an optional `flight_plan`
(time-ordered setpoint segments that merge cumulatively: pitch/roll/heading/
altitude/speed/throttle), and an optional CSV `log`. Paths in `"vehicle"` are
relative to the scenario file.

## Design patterns

Strategy (`EquationsOfMotion`, `AeroModel`, `Controller`, `Actuator`,
`PropulsionModel`, `GravityModel`, `WindModel`), registry-based Factory
(`aero::Factory`, `control::Factory`, `eom::create`, `propulsion::create`),
Observer (`SimObserver` → `CsvLogger`), Composition (`Entity` bundles one
vehicle's full stack).
