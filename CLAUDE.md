# flightsim2 — agent notes

Scenario-driven multi-vehicle flight sim. C++17, stdlib only, no external deps.
See README.md for layout, config schemas, and how to add vehicle types.

## Build / test / run

```
cmake -S . -B build && cmake --build build -j       # Windows: add -G "MinGW Makefiles"
ctest --test-dir build --output-on-failure          # tests run with repo root as CWD
./build/flightsim scenarios/<name>.json
```

Primary dev machine is Linux (since 2026-07). ctest includes `golden_gate`
(tools/check_golden.py): byte-exact CSV comparison of all 15 golden scenarios.
Goldens are baselined on this Linux/GCC toolchain; MinGW builds differ in the
last printed digit (libm rounding), so re-baseline (`--update <names>`) only
deliberately, never to paper over a diff you don't understand.

On Windows, MinGW builds link `-static` on purpose (mixed libstdc++ DLLs on
PATH cause 0xc0000139 crashes otherwise). Keep it.

## Locked conventions — do not "fix"

- NED inertial frame, z down, `altitude = -position.z`. Body: x fwd, y right, z down.
- Quaternion is scalar-first (w,x,y,z), inertial → body, `normalize()` every EOM step.
- Forward Euler integration, fixed dt. Intentional (matches flightsim v1 behavior).
- Control sign conventions (see `src/core/Channel.h`): +elevator = nose DOWN,
  +rudder = nose LEFT, +aileron = right roll. Controllers flip signs accordingly
  (`out.set(elevator_, -pitchPid...)`).
- RocketAero axisymmetric mirror defaults: `cnb=-cma`, `cnr=cmq`, `cndr=cmde`,
  `cyb=-cna`, `cydr=-cnde` — deliberate, overridable per config.
- Config files: degrees/`_dps` keys, converted ONCE at the loading boundary
  (`math/Units.h`). Sim core is SI radians only.
- Two-phase step (snapshot → propagate → commit) in `Simulation::step` keeps
  multi-vehicle runs order-independent. `Entity::propagate` must not mutate state.

## Force components + channels (how loads and control enter the sim)

A Vehicle = MassModel + a flat list of **ForceComponents** (`src/component/`,
ADR-0001). Each component maps `(state, air, channels) → body Wrench` about its
own `momentReferenceStation()` (NaN = about CG); the Entity transfers each to
the current CG and sums them. Gravity is applied by the Entity (it SEEDS the
force accumulator — that exact FP summation order reproduces the pre-component
results bit-for-bit; don't reorder it casually). Aero models are wrapped by
`AeroComponent`; a motor + its mount is one `Propulsor` component (axial by
default, gimbaled TVC with a `"gimbal"` config block; My = arm*T*sin(tvc_pitch)
nose-up for +, Mz = -arm*T*sin(tvc_yaw) nose-right for +, arm = nozzleStation -
xcg so authority grows as CG moves forward and dies at burnout).

Control commands flow as NAMED CHANNELS (`src/core/Channel.h`, replaced the old
ControlInput union): components DECLARE the channels they consume
(`declareChannels`, e.g. `elevator`, `tvc_pitch`; motors own `throttle`) and the
controller BINDS the channels it writes (`bindChannels`) — both once, at load.
Required channels with no consumer throw there (listing what IS declared), so a
mismatched controller/airframe pairing fails at load instead of silently flying
open-loop. Derivative aero models declare only surfaces with nonzero control
derivatives — that's what makes the validation real. Runtime is index-based
(`ChannelValues`, fixed capacity, no string lookups in the loop). Per-channel
servo dynamics live in `ActuatorBank` (lag + slew rate + stop from the vehicle's
`gnc.actuator` block, parameter set picked by ChannelKind).

- Moment sign reminder: body My>0 = nose UP, Mz>0 = nose RIGHT (q_dot=My/Iyy).
  Fin controllers flip elevator sign (`-pitchPid`); the TVC controller does NOT
  (its gimbal sign is defined so +command = +attitude directly). The
  allocation path (`allocated_attitude` -> Allocator) carries NO signs at all:
  they live in the effectiveness columns (`controlEffectiveness`, dM/dchannel
  about the CG) that components report -- fins analytically, table aero by
  differencing its control tables, the gimbal as arm*lastThrust (one-step
  thrust lag, deliberate). Allocation gains are angular-accel scale and must
  dominate weathercock stiffness (~100x the direct-PID scale, plus ki for
  trim); see vehicles/hybrid_launcher.json + scenarios/hybrid_launch.json
  (the TVC+fin blend acceptance vehicle, asserted in test_scenario).
- A TVC launcher must be near-neutral/low-static-margin in pitch (small `cma`, no
  aero control derivatives) or the aero weathercock cancels the gimbal authority;
  roll is left to aero damping (single nozzle = pitch/yaw only). See
  vehicles/tvc_rocket.json.

## Config schema (components[] + gnc, since increment 2)

Vehicle JSON: `mass` block (or legacy flat `mass_kg`+`inertia`, which adds solid
propellant to dry mass) + `"components": [...]` (each entry names its
implementation via explicit `"type"` — no key-sniffing, no vehicle-type
dispatch) + `"gnc": {"control_law": {"type": ...}, "actuator": {...}}`. The old
schema (top-level aero/propulsion/thrust_vectoring/controller/actuator) is a
LOAD ERROR by design (clean break). Component order in the array is the compute
order — keep aero first, motor second for bit-identical results.

## Where things extend

Everything is a registry (see docs/BUILDING_VEHICLES.md):

- New force producer (aero, motor, RCS, rotor...) = ForceComponent subclass +
  `component::Factory::registerComponent`. Aero models can stay AeroModel
  subclasses wrapped in `AeroComponent`; component types: aircraft_aero /
  f16_aero / rocket_aero / rocket_table_aero / turbojet / solid_motor /
  tabulated_thrust / f16_engine.
- New mass model = MassModel subclass + branch in `vehicle::create` (`mass`
  block: "constant" | "dry_plus_propellant" | "tabulated").
- New control law = ControlLaw subclass + `gnc::Factory::registerControlLaw`
  (types: aircraft_pid / rocket_pid / tvc_pid / scheduled / lqr /
  allocated_attitude). Chosen independent of the airframe. Laws take a
  GncContext {state, air, mass, dt}.
- New guidance law = GuidanceLaw subclass + `guidance::Factory::registerLaw`
  (types: pro_nav / pure_pursuit). Guidance declares its command level
  (attitude vs acceleration); the control law declares what it accepts;
  mismatches throw when guidance attaches.
- Multi-vehicle interactions read others via the `WorldView` in
  `Entity::propagate`; the intercept watch lives in `Simulation::step`.

## CG travel / variable mass (the physics)

- `MassModel::at(t)` returns {mass, inertia (about CG), xcg}. `xcg` default NaN
  = "at the aero reference", no transfer. The Entity applies
  `M_cg = M_ref + (xcg - xref, 0, 0) x F` (body x fwd, station aft-positive):
  `My -= dx*Fz; Mz += dx*Fy`. Verified equivalent to the F-16's fractional
  transfer (test/notes). Both stations must share a datum + direction; only the
  difference is used.
- The F-16 declares `xref = xcgr * cbar`; a rocket declares `xref_m` (from nose)
  in its aero block; the mass model supplies `xcg` in the same frame.

## Control design (Python)

- `tools/design_autopilot.py` builds the augmented short-period plant
  [alpha, q, e, integral(e)] from the DATCOM derivatives (finite-differenced) +
  mass/CG at a Mach sweep, runs LQR (`control` pkg) or pole placement, and writes
  `gain_schedule.csv`. `ScheduledController` applies `de = -(k_alpha*alpha +
  k_q*q + k_theta*e + k_i*z)`, gains interpolated on Mach, yaw mirrored, roll PD.
  Default LQR weights (r=80, q_theta=20, q_int=2) keep fin commands within ~15 deg.
- Design signs are baked into K via the plant's B, so the controller applies
  `-(K.x)` with NO extra sign flip (unlike the PID's `-pitchPid`).

## DATCOM / Python side

- `datcom/` is a trimmed vendored copy of C:\dev\pyParserForDatcom: the
  `pydatcom` parser + `examples/` + `vehicles/` + `tests/`. The `ml/` tree,
  `.venv`, and caches were deliberately excluded. numpy-only.
- `tools/datcom_export.py` reads a parsed `aero.npz` and writes a vehicle
  folder (aero_tables.csv, control_tables.csv, vehicle.json, mesh.json).
  Conversions at the boundary: ft->m, per-deg->per-rad (cnb/cyb; rate
  derivatives cmq/cnr/clp are already per-rad), alpha/delta deg->rad. DATCOM
  gives aero+geometry only; mass/inertia are estimated (slender-body) or set
  via --mass-kg. `cbar` from the deck IS body length, not MAC.
- `RocketTableAero` mirrors flightsim v1's LinearAero conventions exactly,
  including the yaw cbar/bref conversion. One added twist: this project's
  +rudder = nose-LEFT (aircraft convention), but the DATCOM pitch tables are
  mirrored onto yaw with +delta = nose-RIGHT, so the rudder is looked up
  NEGATED (`dr = -u.rudder`). Do not remove.
- A sounding rocket is a poor interceptor (heavy, very stable, small fins):
  vehicles/interceptor_missile.json is the agile derivative-aero vehicle that
  actually hits in scenarios/intercept.json.
- The F-16 is now the REAL tabular model: Stevens & Lewis / NASA TP-1538
  wind-tunnel tables + turbofan, ported from the sibling Desktop/PROJECT model
  (vehicles/f16/*.csv). Aero validated cell-for-cell against
  tests/fixtures/f16_coeff_checks.csv (test_f16aero). The airframe is statically
  UNSTABLE at xcg=0.35 cbar and its aileron sign is INVERTED vs the missile
  convention (+aileron -> LEFT roll) -- vehicles/f16.json carries negative roll
  gains and the pitch loop acts as SAS. Credible subsonic only (no Mach dep).

## Python tools

- `tools/visualize.py` + `tools/meshes.py`: emit ONE self-contained offline
  HTML viewer per scenario -- three.js r147 UMD + uPlot, vendored in
  `tools/vendor/` and inlined at build (only place third-party code is
  allowed). Articulated body-frame meshes (+x fwd, +y right, +z down; parts
  carry `hinges` driven by channel columns -- fins, surfaces, TVC bell),
  toggleable overlays (triad, velocity, alpha/beta arcs, LOS + closing,
  thrust vector, plume, aero force w/ kN readout, setpoint ghost, labels),
  focus vehicle with orbit/follow/chase cameras, and preset-only uPlot
  plots (built per vehicle from available columns; click a plot to seek the
  animation). NED->scene mapping is x=east, y=up, z=north; VERIFY any
  renderer change with `node tools/check_viz.mjs <view.html>` -- it asserts
  the attitude math on known angles and runs the whole app under stubs.
- `tools/make_gallery.py`: runs the curated scenarios and writes the
  COMMITTED portfolio gallery `docs/gallery/*.html` + index. Regenerate it
  after any visualizer or vehicle change that alters those flights.
- Python is `python3` (3.12) on the Linux dev machine (`py` on Windows).
  numpy and node are NOT currently installed here: check_golden.py needs
  neither, but the DATCOM/design tools (numpy) and check_viz.mjs (node) do —
  install before using those.

## Known behaviors (not bugs)

- `rocket_launch.json`: after apogee the flight plan still commands 65° pitch,
  so fins saturate at -15° during ballistic descent. Expected; anti-windup holds.
- Kinematic entities need no vehicle definition; loader skips aero/controller.
- The LQR rocket has a small (~4 Hz, decaying) pitch-rate wiggle the PID lacks;
  `tools/analyze.py --tmax 14` shows it. Underdamped, stable; left as-is.

## Missile examples (AAM/SAM/AGM/SSM) — full pipeline

Built by `tools/make_missiles.py`: DATCOM aero run -> tables -> vehicle.json
(DATCOM table aero + solid motor + LQR autopilot) -> `design_autopilot.py`.
Scenarios use ProNav/pursuit guidance + the intercept watch. All four HIT.
Hard-won gotchas encoded there:
- ScheduledController YAW mirrors PITCH with the SAME sign (both +control ->
  attitude decreases); an earlier extra negation made yaw diverge.
- ROLL: DATCOM's CLP is ~0/slightly anti-damping and missile roll inertia is
  tiny, so the roll fin loop limit-cycles at high qbar and spins up p to
  thousands of deg/s, which then drives a gyroscopic q-r explosion (forward
  Euler can't integrate it). Fixes: realistic (larger) ixx in make_missiles,
  gentle roll gains, and `rollScale = min(1, qbarRef/qbar)` attenuation in
  ScheduledController (never amplify at low qbar).
- Ground-attack missiles dive to high qbar, so their LQR is designed at a LOW
  altitude (make_missiles `design.alt`) and the intercept watch runs BEFORE the
  ground-impact kill (Simulation::step) so a diving hit registers.
- Steep diving ProNav endgames rail the pitch command as range->0; a larger
  blind_range freezes the collision course first. Air-to-ground works best as a
  near-vertical top-attack (gravity-aligned, little horizontal reach needed).

## Analysis tools (Python)

- `analyze.py`: reads a CSV log (incl. the `*_sp` setpoint columns the
  CsvLogger now emits) -> step metrics + FFT oscillation detection + envelope.
- `monte_carlo.py`: perturbs a scenario (aim/target/wind/thrust), runs
  `flightsim --json` N times, aggregates Pk / CEP / R90 + dispersion HTML.
  `--json` main output and `InterceptResult::relPos` (miss vector) support it.
