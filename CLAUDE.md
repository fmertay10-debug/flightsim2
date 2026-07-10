# flightsim2 — agent notes

Scenario-driven multi-vehicle flight sim. C++17, stdlib only, no external deps.
See README.md for layout, config schemas, and how to add vehicle types.

## Build / test / run

```
cmake -S . -B build -G "MinGW Makefiles" && cmake --build build -j
ctest --test-dir build --output-on-failure          # tests run with repo root as CWD
./build/flightsim scenarios/<name>.json
```

MinGW builds link `-static` on purpose (mixed libstdc++ DLLs on PATH cause
0xc0000139 crashes otherwise). Keep it.

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

## Control effectors (how control enters the sim)

Control commands flow as NAMED CHANNELS (`src/core/Channel.h`, replaced the old
ControlInput union): force-producing components DECLARE the channels they consume
(`declareChannels`, e.g. `elevator`, `tvc_pitch`) and the controller BINDS the
channels it writes (`bindChannels`) — both once, at load, in the scenario loader.
Required channels with no consumer throw there (listing what IS declared), so a
mismatched controller/airframe pairing fails at load instead of silently flying
open-loop. Derivative aero models declare only surfaces with nonzero control
derivatives — that's what makes the validation real. Runtime is index-based
(`ChannelValues`, fixed capacity, no string lookups in the loop). Per-channel
servo dynamics live in `ActuatorBank` (lag + slew rate + stop from the vehicle's
`actuator` block, parameter set picked by ChannelKind).

Control reaches the vehicle two physical ways, kept deliberately separate:
- **Aerodynamic** control (fins) acts THROUGH the AeroModel — the surface
  channels change the airflow. Stays inside the aero models.
- **Propulsive/reaction** control is a pluggable **Effector list** on the Entity
  (`src/effector/`). Each Effector maps its channels + flight condition to a
  body-frame `Wrench` (about the CG). The Entity sums them (this replaced the old
  inline axial-thrust term, so `ThrustEffector` reproduces it exactly).
  - `ThrustEffector` (default): axial thrust, no moment, no channels.
  - `TvcEffector`: gimbaled thrust; My = arm*T*sin(tvc_pitch) (nose-up for +),
    Mz = -arm*T*sin(tvc_yaw)... (nose-right for +). arm = nozzleStation - xcg, so
    it grows as the CG moves forward and goes to zero at burnout (thrust=0).
- New control method = Effector subclass declaring its own channels
  (+ `effector::build` branch) + Controller binding them (+ `control::Factory`
  "method" branch). No shared struct to edit — channel names are the whole
  contract; sim/dynamics/mass/aero are untouched.
- Moment sign reminder: body My>0 = nose UP, Mz>0 = nose RIGHT (q_dot=My/Iyy).
  Fin controllers flip elevator sign (`-pitchPid`); the TVC controller does NOT
  (its gimbal sign is defined so +command = +attitude directly).
- A TVC launcher must be near-neutral/low-static-margin in pitch (small `cma`, no
  aero control derivatives) or the aero weathercock cancels the gimbal authority;
  roll is left to aero damping (single nozzle = pitch/yaw only). See
  vehicles/tvc_rocket.json.

## Where things extend

The vehicle is Lego blocks (aero / mass / propulsion / effectors / controller),
each a Strategy + factory. See docs/BUILDING_VEHICLES.md.

- New aero (per vehicle type) = AeroModel subclass registered in `aero::Factory`
  (registry in AeroFactory.cpp). Builder takes `baseDir` for data paths. Report
  moments about `momentReferenceStation()` (NaN = about CG, no transfer); the
  Entity transfers to the current CG using `MassState::xcg`.
- New mass model = MassModel subclass + branch in `vehicle::create` (`mass`
  block: "constant" | "tabulated").
- New propulsion = PropulsionModel subclass + branch in `propulsion::create`.
  `thrust(PropulsionContext&)` is non-const (engines advance spool state).
- New controller / control law = Controller subclass. Register a per-type
  default in `control::Factory`, OR add a `method` branch (like "lqr" ->
  ScheduledController) so it's selectable independent of the airframe.
- New guidance law = GuidanceLaw subclass + branch in `guidance::create`.
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

- `tools/visualize.py` + `tools/meshes.py`: emit a self-contained HTML 3-D
  viewer from a scenario's CSV logs. Hand-rolled canvas renderer (no deps).
  Body-frame procedural meshes (+x fwd, +y right, +z down) so attitude is
  exact. Verify changes headlessly with node: extract the `<script>`, stub
  document/canvas, eval, and call drawTrajectory/drawAttitude across frames.
- Python is invoked as `py` (Windows launcher, 3.13). numpy 2.x is available.

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
