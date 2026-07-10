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
                 │    control_law ──►  control algorithm               │  Controller
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
| `rocket_aero` | `cna`,`cma`,… | linear missile derivatives |
| `rocket_table_aero` | `tables_csv`,`control_csv` | DATCOM (alpha, Mach) tables |

Optional `xref_m` names the moment reference station so CG travel changes the
static margin (see the mass block).

Motors (each takes an optional `gimbal` block — see below):
`turbojet` (throttleable, density-lapsed) · `solid_motor` (thrust-time curve +
impulse-consistent propellant) · `tabulated_thrust` (raw thrust(t) table) ·
`f16_engine` (idle/mil/max tables with power-lever dynamics).

### `mass` — mass / inertia / CG
| option | config | what |
|---|---|---|
| constant | `{"model":"constant","mass_kg":…,"inertia":{…}}` | fixed tensor (+ optional `ixz`) |
| tabulated | `{"model":"tabulated","table":"mass_props.csv"}` | mass, Ixx/Iyy/Izz, **xcg** vs time |

(Legacy flat `mass_kg` + `inertia` still works and adds a solid motor's
propellant to the dry mass automatically.)

### `gnc.control_law` — the control algorithm (registry: `gnc::Factory`)
| `type` | config | what |
|---|---|---|
| `aircraft_pid` | `gains`, `limits` | cascaded fixed-wing PID (direct-write) |
| `rocket_pid` | `gains`, `limits` | finned-rocket attitude PID (direct-write) |
| `tvc_pid` | `gains`, `limits` | thrust-vector-control PID (direct-write) |
| `scheduled` / `lqr` | `schedule: gains.csv` | gain-scheduled state feedback, gains **auto-designed** |
| `allocated_attitude` | `gains`, `limits` | attitude PID → desired body moments → **Allocator** |

The law is chosen independently of the airframe — fly the same vehicle with
PID or LQR by editing one line.

Two output contracts (ADR-0002): the direct-write laws put plant knowledge in
their gains and write specific channels; `allocated_attitude` emits desired
body moments (scaled by the live inertia) and the **Allocator** distributes
them over whatever channels the components declare, weighted by each
component's queried effectiveness at the current flight condition. That is
what flies a hybrid: `vehicles/hybrid_launcher.json` has a gimbaled motor AND
fins — the gimbal steers the low-qbar pad phase, the fins take over as speed
builds, and after burnout the fins track alone, all under one law with no
mode switching (`scenarios/hybrid_launch.json`). Note its gains are in
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
`rocket_pid` with a control-derivative-free TVC airframe is caught too.

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
aero component a small `cma` and no control derivatives — see
`vehicles/tvc_rocket.json` and `scenarios/tvc_launch.json`.

**To add a new force/control component** (e.g. RCS): implement a
`ForceComponent` that *declares* its own channels (e.g. `rcs_roll`) +
`component::Factory::registerComponent("rcs", ...)`, and a control law that
*binds* and writes those channels + `control::Factory::registerControlLaw`.
No shared struct to edit — channel names are the whole contract. Nothing in
`sim/`, `dynamics/`, or the other blocks changes.

## Worked example: a fin-controlled rocket, PID → LQR

1. **Generate the airframe from DATCOM** (aero tables + mesh + a starter
   vehicle.json), optionally with variable mass/CG/thrust:

   ```
   py tools/datcom_export.py 02_rocket_fin_control --name my_rocket
   # add --variable for tabulated mass_props.csv + thrust.csv (CG travel)
   ```

2. **Fly it with the hand-tuned PID** it shipped with:

   ```
   ./build/flightsim scenarios/datcom_rocket_launch.json
   ```

3. **Design an autopilot from the vehicle's OWN data** — the control block gets
   its parameters from the aero + mass blocks:

   ```
   py tools/design_autopilot.py vehicles/generated/my_rocket/vehicle.json \
       --mass 72 --iyy 480 --altitude 3000 --method lqr
   #   -> vehicles/generated/my_rocket/gain_schedule.csv  (LQR gains vs Mach)
   ```

   The tool linearizes the short-period plant at each Mach from the DATCOM
   derivatives + the mass/CG, runs LQR, and prints the open- and closed-loop
   modes. `--method place --wn 12 --zeta 0.7` uses pole placement instead.

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

   (`vehicles/generated/datcom_rocket/vehicle_lqr.json` is exactly this: the
   same components as `vehicle.json`, only the control law changed.)

5. **Fly the LQR version** and compare — same airframe, same pitch program:

   ```
   ./build/flightsim scenarios/lqr_rocket_launch.json
   py tools/visualize.py scenarios/lqr_rocket_launch.json   # 3-D viewer
   ```

Nothing in `src/` changed across any of this. New airframe = data + JSON; new
control law or component = a subclass + one registry call.
