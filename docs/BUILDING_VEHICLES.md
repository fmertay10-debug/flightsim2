# Building a vehicle — the Lego model

A vehicle in flightsim2 is **mass properties + a list of force components +
a GNC stack**, snapped together in one JSON file. You never touch the
simulation core to add a vehicle; every block names its implementation
explicitly via `"type"`, and the loader wires them through registries.

```
                 ┌──────────────────────────────────────────────────┐
   vehicle.json  │  mass          ──►  mass / inertia / CG            │  MassModel
                 │  components: [                                     │
                 │    aero        ──►  aero forces/moments             │  ForceComponent
                 │    motor       ──►  thrust (axial or gimbaled)      │  ForceComponent
                 │    ...         ──►  any wrench producer             │
                 │  ]                                                  │
                 │  gnc:                                               │
                 │    control_law ──►  control algorithm               │  ControlLaw
                 │    actuator    ──►  per-channel servo dynamics (opt)│  ActuatorBank
                 └──────────────────────────────────────────────────┘
```

Every component maps `(state, environment, channels) → body wrench` about its
own reference station; the Entity transfers each to the current CG and sums
them (gravity is applied by the Entity itself). The blocks are interchangeable
and independent — a rocket can use a solid motor or a tabulated thrust curve;
an aircraft can use derivative aero or DATCOM/wind-tunnel tables; any vehicle
can be flown by a hand-tuned PID or an auto-designed LQR.

## The blocks

### `components` — force producers (registry: `component::Factory`)

Aerodynamics:
| `type` | config | what |
|---|---|---|
| `aircraft_aero` | `cla`,`cma`,… | linear stability derivatives |
| `f16_aero` | `dir` | wind-tunnel table set (the F-16) |
| `rocket_table_aero` | `tables_csv`,`control_csv` | DATCOM (alpha, Mach) tables |

Optional `xref_m` names the moment reference station so CG travel changes the
static margin (see the mass block).

Motors (each takes an optional `gimbal` block — see below):
`turbojet` (throttleable, density-lapsed) · `solid_motor` (thrust-time curve;
`propellant_kg` is generator metadata that sizes the mass CSV's drain) ·
`tabulated_thrust` (raw thrust(t) table) · `f16_engine` (idle/mil/max tables
with power-lever dynamics).

### `mass` — mass / inertia / CG

One model covers every case:

```json
"mass": { "model": "tabulated", "table": "mass_props.csv" }
```

CSV columns: `time_s, mass_kg, ixx, iyy, izz` plus optional `ixy, ixz, iyz`
(products of inertia, entered as negative off-diagonals) and optional `xcg_m`
(CG station vs time; absent = CG at the aero reference, no moment transfer).
Exception: a **gimbaled** vehicle REQUIRES `xcg_m` — the TVC moment arm is
`nozzle_station_m - xcg`, which cannot be formed without a CG, so the loader
rejects a gimbal whose mass table lacks the column (a constant CG is one value
repeated per row). Values interpolate linearly and hold at the endpoints, so:

- a **constant-mass** vehicle is a two-row table (see `data/vehicles/f16/mass_props.csv`);
- a **burning motor** is rows over the burn — the generators sample the
  impulse-proportional propellant drain at the thrust-curve breakpoints;
- **CG travel** (changing static margin) is an `xcg_m` column
  (see `data/vehicles/generated/advanced_rocket/mass_props.csv`).

(The retired `constant` / `dry_plus_propellant` models were special cases of
this table; the loader rejects them, pointing here.)

### `gnc.control_law` — the control algorithm (registry: `gnc::Factory`)
| `type` | config | what |
|---|---|---|
| `allocated_attitude` | `gains`, `limits` | attitude PID → desired body moments → **Allocator** (rockets/missiles) |
| `aircraft_allocated` | `gains`, `limits` | fixed-wing cascades (altitude→climb→pitch, heading→bank, speed→throttle) over the same allocated inner loop |
| `scheduled` / `lqr` | `schedule: gains.csv` | gain-scheduled state feedback, gains **auto-designed**; output converted to a WrenchCommand at the boundary |

The law is chosen independently of the airframe — fly the same vehicle with
hand gains or an LQR schedule by editing one line.

One output contract (ADR-0004; the direct-write PIDs were retired after every
vehicle converted): a law emits a desired body wrench (WrenchCommand) and the
**Allocator** distributes it over whatever channels the components declare,
weighted by each component's queried effectiveness at the current flight
condition. Throttle passes through as a direct channel write. That is
what would fly a hybrid (gimbaled motor AND fins) with no mode switching:
the gimbal's authority columns dominate at low qbar, the fins' as speed
builds, and after burnout the fins track alone. Note allocation gains are in
angular-acceleration units (rad/s² per rad of error) and must dominate the
airframe's weathercock stiffness — expect values ~100×, not ~1×, with an
integral term to hold trim.

### Channels — how control reaches the components

Control commands flow as **named channels** (`src/core/Channel.h`): each
component *declares* the channels it consumes (`elevator`, `tvc_pitch`, ...)
and the control law *binds* the channels it writes, once, at load. The loader
validates the pairing — a control law whose required channel nothing on the
vehicle declares fails with an error that lists what IS declared. A derivative
aero model only declares surfaces with nonzero control derivatives, so pairing
a fin-requiring law with a control-derivative-free TVC airframe is caught too.
An allocating law is also PROBED at load: every declared Surface channel must
have an effectiveness column from some component, or the load fails naming the
channel (no silent open-loop flight).

Two physical routes into the airframe:

- **Aerodynamic** (fins): the surface channels feed the aero component, which
  changes the airflow.
- **Propulsive** (the motor's mount): an axial motor is pure thrust through
  the CG; adding a `gimbal` block makes the nozzle steerable — the motor then
  declares `tvc_pitch`/`tvc_yaw` and produces pitch/yaw control moments:

```json
{ "type": "solid_motor", "propellant_kg": 210, "thrust_curve": [...],
  "gimbal": { "nozzle_station_m": 6.0, "max_gimbal_deg": 6 } }
```

TVC authority is proportional to thrust and to the CG→nozzle moment arm, so it
strengthens as the CG migrates forward through the burn and goes to zero at
burnout — the physics falls straight out of the mass/motor blocks. A single
nozzle gives pitch + yaw only; roll needs fins or (future) an RCS component,
so a TVC vehicle should be aerodynamically roll-damped. TVC launchers are
near-neutral or unstable in pitch (that's *why* they need TVC), so give the
aero component a small `cma` and no control derivatives.

**To add a new force/control component** (e.g. RCS): implement a
`ForceComponent` that *declares* its own channels (e.g. `rcs_roll`) +
`component::Factory::registerComponent("rcs", ...)`, and a control law that
*binds* and writes those channels + `gnc::Factory::registerControlLaw`.
No shared struct to edit — channel names are the whole contract. Nothing in
`sim/`, `dynamics/`, or the other blocks changes.

## Worked example: a fin-controlled rocket, PID → LQR

1. **Generate the airframe from DATCOM** (aero tables + mesh + a starter
   vehicle.json), optionally with variable mass/CG/thrust:

   ```
   py tools/datcom_export.py 02_rocket_fin_control --name my_rocket
   # add --variable for tabulated mass_props.csv + thrust.csv (CG travel)
   ```

2. **Fly it with the hand-tuned allocation law** it shipped with:

   ```
   ./build/flightsim data/scenarios/datcom_rocket_launch.json
   ```

3. **Design an autopilot from the vehicle's OWN data** — the control block gets
   its parameters from the aero + mass blocks:

   ```
   ./build/flightsim --linearize data/vehicles/generated/my_rocket/vehicle.json \
       --altitude 3000 --machs 0.3,0.5,0.7 --out plant.csv
   py tools/design_autopilot.py --plant plant.csv --method lqr
   #   -> data/vehicles/generated/my_rocket/gain_schedule.csv (LQR gains vs Mach)
   ```

   `--linearize` trims and linearizes the REAL component stack (the same
   f(x,u) the sim flies) into a short-period plant per Mach; design_autopilot
   runs LQR on it and prints the open- and closed-loop modes.
   `--method place --wn 12 --zeta 0.7` uses pole placement instead.

4. **Swap the control law to LQR** — one section of the vehicle file:

   ```json
   "gnc": {
     "control_law": {
       "type": "lqr",
       "schedule": "gain_schedule.csv",
       "limits": { "max_fin_deg": 15, "min_airspeed_ms": 20 }
     }
   }
   ```

   (`data/vehicles/generated/datcom_rocket/vehicle_lqr.json` is exactly this: the
   same components as `vehicle.json`, only the control law changed.)

5. **Fly the LQR version** and compare — same airframe, same pitch program:

   ```
   ./build/flightsim data/scenarios/lqr_rocket_launch.json
   py tools/visualize.py data/scenarios/lqr_rocket_launch.json   # 3-D viewer
   ```

Nothing in `src/` changed across any of this. New airframe = data + JSON; new
control law or component = a subclass + one registry call.
