# Building a vehicle — the Lego model

A vehicle in flightsim2 is **four independent blocks** you snap together in one
JSON file. You never touch the simulation core to add a vehicle; you pick a
block for each slot, and the loader wires them through their factories.

```
                 ┌──────────────────────────────────────────────┐
   vehicle.json  │  type:  "aircraft" | "rocket" | ...            │
                 │                                                │
                 │  aero             ──►  aero forces/moments      │  AeroModel
                 │  mass             ──►  mass / inertia / CG      │  MassModel
                 │  propulsion       ──►  thrust magnitude         │  PropulsionModel
                 │  thrust_vectoring ──►  how thrust is applied     │  Effector (opt)
                 │  controller       ──►  control algorithm        │  Controller
                 │  actuator         ──►  servo dynamics (opt)     │  Actuator
                 └──────────────────────────────────────────────┘
```

Each slot is a Strategy with a factory, so the options below are interchangeable
and independent — a rocket can use a solid motor or a tabulated thrust curve; an
aircraft can use derivative aero or DATCOM/wind-tunnel tables; any vehicle can be
flown by a hand-tuned PID or an auto-designed LQR.

## The blocks

### `aero` — aerodynamics
| option | trigger | what |
|---|---|---|
| derivative aircraft | `aircraft` + `cla`,`cma`,… | linear stability derivatives |
| DATCOM/wind-tunnel aircraft | `aircraft` + `dir` | table set (the F-16) |
| derivative rocket | `rocket` + `cna`,`cma`,… | linear missile derivatives |
| DATCOM table rocket | `rocket` + `tables_csv` | (alpha, Mach) coefficient tables |

Optional `xref_m` names the moment reference station so CG travel changes the
static margin (see the mass block).

### `mass` — mass / inertia / CG
| option | config | what |
|---|---|---|
| constant | `{"model":"constant","mass_kg":…,"inertia":{…}}` | fixed tensor (+ optional `ixz`) |
| tabulated | `{"model":"tabulated","table":"mass_props.csv"}` | mass, Ixx/Iyy/Izz, **xcg** vs time |

(Legacy flat `mass_kg` + `inertia` still works and adds a solid motor's
propellant to the dry mass automatically.)

### `propulsion` — thrust
`none` · `turbojet` (throttleable, density-lapsed) · `solid_motor` (thrust-time
curve + impulse-consistent propellant) · `tabulated_thrust` (raw thrust(t)
table) · `f16_engine` (idle/mil/max tables with power-lever dynamics).

### `controller` — the control algorithm (independent of the airframe)
| method | config | what |
|---|---|---|
| `pid` (default) | `gains`, `limits` | per-type cascaded PID |
| `lqr` / `scheduled` | `schedule: gains.csv` | gain-scheduled state feedback, gains **auto-designed** |
| `tvc` | `gains`, `limits` | thrust-vector-control PID (drives the nozzle gimbal) |

The control method is chosen by the `method` field, *not* by the vehicle type —
so you can fly the same airframe with PID or LQR by editing one line.

### Control effectors — HOW control moments are produced

Control reaches the airframe through **effectors**. Two physical families:

- **Aerodynamic** (fins): the surface deflection channels feed the `AeroModel`,
  which changes the airflow. This is implicit — any aero model that reads the
  surfaces is a fin effector.
- **Propulsive / reaction** (a pluggable effector list on the Entity): the
  default is an axial `ThrustEffector`; adding a `thrust_vectoring` block swaps
  in a `TvcEffector` that gimbals the thrust for pitch/yaw moments.

Control commands flow as **named channels** (`src/core/Channel.h`): each
component *declares* the channels it consumes (`elevator`, `tvc_pitch`, ...)
and the controller *binds* the channels it writes, once, at load. The loader
validates the pairing — a controller whose required channel nothing on the
vehicle declares fails with an error that lists what IS declared. A derivative
aero model only declares surfaces with nonzero control derivatives, so pairing
`method: "pid"` with a control-derivative-free TVC airframe is caught too.

```json
"thrust_vectoring": { "nozzle_station_m": 6.0, "max_gimbal_deg": 6 }
```

TVC authority is proportional to thrust and to the CG→nozzle moment arm, so it
strengthens as the CG migrates forward through the burn and goes to zero at
burnout — the physics falls straight out of the mass/propulsion blocks. A
single nozzle gives pitch + yaw only; roll needs fins or (future) an RCS
effector, so a TVC vehicle should be aerodynamically roll-damped. TVC launchers
are near-neutral or unstable in pitch (that's *why* they need TVC), so give the
`aero` block a small `cma` and no control derivatives — see `vehicles/tvc_rocket.json`
and `scenarios/tvc_launch.json`.

**To add a new control METHOD** (e.g. RCS): implement an `Effector` subclass
that *declares* its own channels (e.g. `rcs_roll`) + register it in
`effector::build`, and add a `Controller` that *binds* and writes those
channels + register the method in `control::Factory`. No shared struct to
edit — channel names are the whole contract. Nothing in `sim/`, `dynamics/`,
or the other blocks changes.

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

4. **Swap the control block to LQR** — one section of the vehicle file:

   ```json
   "controller": {
     "method": "lqr",
     "schedule": "gain_schedule.csv",
     "limits": { "max_fin_deg": 15, "min_airspeed_ms": 20 }
   }
   ```

   (`vehicles/generated/datcom_rocket/vehicle_lqr.json` is exactly this: the same
   aero/mass/propulsion as `vehicle.json`, only the controller changed.)

5. **Fly the LQR version** and compare — same airframe, same pitch program:

   ```
   ./build/flightsim scenarios/lqr_rocket_launch.json
   py tools/visualize.py scenarios/lqr_rocket_launch.json   # 3-D viewer
   ```

Nothing in `src/` changed across any of this. New airframe = data + JSON; new
control law = a factory branch (`method`) + a design pass.
