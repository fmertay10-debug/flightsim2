# Increment 1 — Named channels + load-time validation

Part of the architecture refactor decided in `docs/adr/0001` / `docs/adr/0002` and
`CONTEXT.md`. Increment sequence: **(1) channels + validation** → (2) ForceComponent
unification + explicit `components[]` schema → (3) GNC restack (ControlLaw rename,
Allocator, dual command vocabulary). Source-tree moves land with the increment that
touches the files.

## Context

`ControlInput` (`src/core/ControlInput.h`) is a fixed union of six channels
(`elevator, aileron, rudder, throttle, tvcPitch, tvcYaw`). Every controller,
aero model, and effector shares it; which fields are *live* for a vehicle is pure
convention, and a controller that writes channels nothing reads fails silently
(vehicle flies open-loop, no error). New vehicle classes (helicopter collective,
per-motor throttles) cannot be expressed without editing this core header.

This increment replaces the union with **named, declared channels** validated at
load, and turns the monolithic `FirstOrderActuator` into **per-channel actuator
dynamics**. Behavior of every existing vehicle must be numerically unchanged.

**Scope guard:** no JSON schema changes (that's increment 2 — the loader still
dispatches on `type`/`method`/key-presence), no allocator, no renames of
Controller/Effector (increment 3). Channel names are hardcoded in the component
classes for now.

## Design

### Core types — new file `src/core/Channel.h` (+ `.cpp`)

```cpp
enum class ChannelKind { Surface, Gimbal, Throttle };  // selects actuator params

struct ChannelDef {
    std::string name;        // snake_case: "elevator", "tvc_pitch", ...
    ChannelKind kind;
    double      minValue, maxValue;   // hard position limits [rad] or [0..1]
};

class ChannelTable {         // built once at load; immutable afterwards
    // add(def) -> ChannelHandle; duplicate name -> throw
    // find(name) -> optional<ChannelHandle>; require(name) -> handle or throw
    //   (error message lists all declared channels)
    // size(), def(i)
};

struct ChannelHandle { int index = -1; bool valid() const; };

class ChannelValues {        // the runtime command/position vector
    // fixed inline capacity (std::array<double,16> + count) -- no heap in the
    // sim loop; sized/zeroed from a ChannelTable
    // operator[](ChannelHandle) get/set; get returns 0.0 for invalid handle
};
```

Semantics, signs, and units of the six existing channels are IDENTICAL to the old
`ControlInput` fields (document the sign table from `ControlInput.h` in each
declaring class). New canonical names: `elevator`, `aileron`, `rudder`,
`throttle`, `tvc_pitch`, `tvc_yaw`.

### Declaration & binding (two load-time phases, wired in `scenario::buildEntity`)

**Phase A — components declare the channels they consume** (they own them):

- `AeroModel`, `Effector`, and `Vehicle` (for propulsion) get:
  `virtual void declareChannels(ChannelTable& t)` — default no-op. Each stores the
  returned handles as members for use in `compute()`.
  - `AircraftAero`, `RocketAero`, `RocketTableAero`, `F16Aero`: declare
    `elevator/aileron/rudder` (Surface, ±limit from their existing config where
    present, else a generous default).
  - `TvcEffector`: declares `tvc_pitch`/`tvc_yaw` (Gimbal, ±`maxGimbal_` — the
    clamp currently inside `TvcEffector::compute` moves to the channel limit).
  - `ThrustEffector`: declares nothing.
  - `Vehicle::declareChannels`: declares `throttle` (Throttle, [0..1]) **only if**
    the propulsion model uses it — add
    `virtual bool PropulsionModel::usesThrottle() const` (true for
    `F16Engine`/`Turbojet`, false for `SolidMotor`/`TabulatedThrust`/`NoPropulsion`).

**Phase B — the controller binds the channels it writes:**

- `Controller` gets `virtual void bindChannels(const ChannelTable& t)`.
  Controllers resolve handles with `t.require(name)` for channels they *must*
  drive and `t.find(name)` for optional ones, and keep them as members.
  - `RocketController` / `ScheduledController`: require `elevator/aileron/rudder`;
    `throttle` optional (solid-motor vehicles don't declare it).
  - `AircraftController`: require `elevator/aileron/rudder/throttle`.
  - `TvcController`: require `tvc_pitch/tvc_yaw`; `throttle` optional.

**Validation outcomes** (this is the whole point):
- Controller requires a channel no component declared → **load error** naming the
  channel and listing what IS declared (e.g. TVC controller on a vehicle without a
  `thrust_vectoring` block now fails loudly instead of flying open-loop).
- Duplicate declaration of the same name → **load error**.
- Declared channel that the controller never binds → **stderr warning** (held at 0),
  not an error — a ballistic vehicle with no controller is legitimate.
- No controller (kinematic or ballistic entity) → validation skipped.

### Interface signature changes

```cpp
// Controller: writes into the vector via its stored handles
virtual void update(const State&, const AirData&, const CommandSet&,
                    double dt, ChannelValues& out) = 0;

// AeroModel / Effector: read via stored handles
virtual AeroForces compute(const State&, const AirData&, const ChannelValues&) const = 0;
virtual Wrench     compute(const EffectorContext&, const ChannelValues&) const = 0;
```

`Entity` gains `channels_` (`ChannelTable`, shared with Telemetry) and threads
`ChannelValues commanded / actual` through the existing step sequence in
`Entity::propagate` (`src/sim/Entity.cpp:54-70`) — flow is unchanged:
flight plan → guidance overlay → `controller_->update(...)` → actuator bank →
aero + effectors. `PropulsionContext.throttle` is filled from the `throttle`
handle (0 if absent).

### Per-channel actuators — `ActuatorBank` replaces `Actuator`/`FirstOrderActuator`

`src/control/ActuatorBank.{h,cpp}`. One first-order servo state per channel
(lag τ + slew-rate limit + position clamp = exactly `FirstOrderActuator::stepChannel`).
Built by the loader from the **unchanged** `"actuator"` JSON block, mapped by
`ChannelKind`:

| kind     | τ               | rate limit | position limit                          |
|----------|-----------------|------------|-----------------------------------------|
| Surface  | `tau_s`         | `rate_dps` | min(`limit_deg`, channel limit)         |
| Gimbal   | `tau_s`         | `rate_dps` | min(`gimbal_limit_deg`, channel limit)  |
| Throttle | `throttle_tau_s`| none       | clamp [0..1]                            |

No `"actuator"` block → ideal (actual = commanded, position clamp only).
This reproduces `FirstOrderActuator` exactly — locked by a regression test.

### Telemetry & CSV logging

`Telemetry.control/controlCmd` become `ChannelValues` + a pointer to the entity's
`ChannelTable`. `CsvLogger` writes dynamic headers from the table: one `<name>` and
one `<name>_cmd` column per channel, in declaration order, in place of today's fixed
`elevator..throttle_cmd` columns (`src/sim/CsvLogger.h`).

- Fin vehicles declare elevator/aileron/rudder/throttle in that order → **headers
  byte-identical to today** → `tools/analyze.py`, `visualize.py`, `monte_carlo.py`
  untouched for them.
- TVC vehicles gain real `tvc_pitch/tvc_yaw` columns (today they log meaningless
  zero elevator columns). Spot-check `analyze.py` against a TVC log once.
- Check `main.cpp --json` output for control fields and port the same way.

## Work plan (each step compiles; ctest green at every ✂ marker)

0. **Safety net first — the repo is not under git.** `git init`, commit baseline,
   then produce golden runs: build current master, run every `scenarios/*.json`,
   save all CSV logs to `golden/` (outside the repo or gitignored).
1. Add `src/core/Channel.{h,cpp}` + unit tests (`tests/test_channels.cpp`):
   add/require/find, duplicate → throw, unknown require → throw with listing,
   capacity guard. ✂
2. Add `ActuatorBank` + regression test proving it matches `FirstOrderActuator`
   step-for-step for a surface/gimbal/throttle triple. ✂
3. Break the interfaces (one commit, the tree won't compile mid-step):
   change `Controller`/`AeroModel`/`Effector` signatures, add `declareChannels`/
   `bindChannels`, delete `ControlInput.h`, `Actuator.h`, `FirstOrderActuator.*`.
   Port all implementers in the same commit:
   - aero: `AircraftAero`, `RocketAero`, `RocketTableAero`, `F16Aero`
   - effectors: `ThrustEffector`, `TvcEffector`
   - controllers: `RocketController`, `AircraftController`, `TvcController`,
     `ScheduledController` (mechanical: field access → handle access)
   - `Entity.{h,cpp}`, `Telemetry.h`, `CsvLogger.{h,cpp}`, `main.cpp`
4. Wire the loader (`src/scenario/ScenarioLoader.cpp` `buildEntity`): build
   `ChannelTable`, run Phase A over aero/effectors/vehicle, Phase B on the
   controller, construct `ActuatorBank` from the `actuator` block. Emit the
   declared-but-unwritten warning. ✂
5. Port existing tests that construct `ControlInput` directly:
   `test_f16aero.cpp` (fixture-driven coefficient checks), `test_tableaero.cpp`,
   `test_tvc.cpp`, `test_scenario.cpp`. Add validation tests: TVC controller on a
   fin-only vehicle → load error; duplicate declaration → load error. ✂
6. Docs: update `CLAUDE.md` (ControlInput references in "Locked conventions" and
   "Control effectors" sections) and `docs/BUILDING_VEHICLES.md`.

## Verification

1. `cmake --build build -j` && `ctest --test-dir build --output-on-failure` — all
   pass, including the new channel/actuator/validation tests.
2. **Golden-run comparison**: run every `scenarios/*.json`, numerically diff each
   CSV against `golden/` (small py script, exact match expected — same math, same
   order of operations; tolerance 1e-12 only to absorb printf round-trip).
   Fin-vehicle headers must be byte-identical; TVC logs gain the two new columns.
3. Negative test by hand: add `"method": "tvc"` to a fin vehicle's controller
   block → loader must reject it with the channel listing (this exact mistake is
   silent today).
4. `py tools/analyze.py` on a rocket log and `py tools/monte_carlo.py` smoke run
   on `scenarios/intercept.json` — confirm the Python toolchain still parses logs.

## Out of scope (later increments)

- Increment 2: `components[]` explicit schema, ForceComponent unification
  (AeroModel/Effector/propulsion behind one wrench interface), registries for
  propulsion/guidance/effectors, port `datcom_export.py` / `make_missiles.py`,
  clean-break schema error for old configs.
- Increment 3: Controller→ControlLaw rename, Allocator + effectiveness queries,
  two-level Command vocabulary, `src/models/<family>/` + `gnc/` layout moves,
  hybrid TVC+fin acceptance vehicle.

## Risks

- **Widest mechanical step is 3** (one big-bang commit across ~28 files). Mitigate:
  it's mechanical field→handle translation; the golden runs catch any slip.
- **CSV column contract with Python tools** — mitigated by preserving declaration
  order for fin vehicles; verify step 4 of Verification.
- **Hidden ControlInput uses**: grep confirms 28 files; `Telemetry`/`CsvLogger`/
  `main.cpp` are the easy-to-forget ones (all listed in step 3).
