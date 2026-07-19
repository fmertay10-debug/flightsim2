# Converting the aircraft to allocation (ADR-0004, last 4 configs) — options

Status: DECISION PENDING (written 2026-07-19, after the rocket/missile fleet
— 9/13 vehicles — converted).

## What's different about aircraft

The remaining direct-write configs are `vehicles/trainer.json` (aircraft_pid),
`vehicles/f16.json` (aircraft_pid as SAS), and their scenarios
(`aircraft_cruise`, `mixed_traffic`, `f16_cruise`, `f16_turn`). Three stacked
problems the rockets didn't have:

1. **Outer loops.** Aircraft scenarios command *flight conditions* (altitude,
   speed, heading), not attitude. `AircraftPidLaw` converts them in cascades
   (altitude → climb-rate → pitch → elevator; heading → bank → aileron;
   speed → throttle). `allocated_attitude` only has the innermost stage, so
   the outer loops need a home.
2. **Missing effectiveness columns.** `AircraftAero` and `F16Aero` still have
   the return-0 `controlEffectiveness` default — the allocator would be blind
   on these airframes. (Dropping `allocated_attitude` into f16.json today
   loads fine and then departs: channels are declared, columns are not — a
   runtime hole, not a load error.)
3. **The F-16 pitch loop is a SAS on a statically UNSTABLE airframe** — the
   riskiest re-tune of the migration — and its inverted aileron convention
   currently lives as NEGATIVE roll gains in f16.json, exactly the plant
   knowledge ADR-0004 evicts from controllers.

## Decision A — where do the outer loops live?

**A1 (recommended): a new cascaded allocation law, `aircraft_allocated`.**
Keep AircraftPidLaw's outer loops verbatim (same gains/limits keys:
`alt_to_vs`, `vs_kp/vs_ki`, `heading_kp`, `speed_kp/ki`, max bank/pitch/climb
clamps), but replace the inner stage: attitude errors → desired angular
accelerations → `WrenchCommand` (moment = I·α) → Allocator, like
AllocatedAttitudeLaw. Throttle stays a direct pass-through channel (precedent:
every converted law writes throttle directly). One new law file, tuning
transfers from the working aircraft, and the sign quirk moves from gains to
effectiveness columns where it belongs.
- Cost: the outer-loop code is duplicated from AircraftPidLaw for one
  release cycle (until aircraft_pid is deleted with the other retired laws).

**A2: outer loops as a command-translator layer.** A separate stage rewrites
CommandSet (altitude/speed → pitch/throttle) upstream of ANY attitude law; the
inner law is just allocated_attitude. Architecturally cleaner (outer loops
become airframe- and law-agnostic) but needs new plumbing (where does the
translator attach — Entity? a wrapping law?) and changes two things at once on
the unstable F-16. Better done later as part of the ControllerSpec/topology
work (TARGET_ARCHITECTURE's CascadedAttitude/TECS library).

**A3: TECS for the longitudinal axis** (throttle ↔ total energy, pitch ↔
energy distribution). The right long-term fixed-wing topology, but a full
redesign with new tuning on both aircraft — Option-C-era work, not a
migration step.

## Decision B — effectiveness columns (required for any option)

- `AircraftAero`: analytic, mirroring `RocketAero`: dMy/dde = qbar·S·cbar·cmde
  (+ pitch-force term via clde and the CG transfer, following the
  RocketTableAero pattern of folding dx·dF into dM), dMx/dda = qbar·S·bref·clda,
  dMz/ddr = qbar·S·bref·cndr, plus cross terms clda↔cnda, cldr↔cndr as extra
  vector components in the same columns.
- `F16Aero`: central-difference the deflection dimension of the wind-tunnel
  tables about zero at the current (alpha, beta), exactly like
  `RocketTableAero::controlEffectiveness` (src/models/rocket/RocketTableAero.cpp:104)
  differences its control tables. The inverted aileron sign then comes out of
  the DATA — f16.json's negative roll gains become positive and ordinary.
- Both referenced to the CG (F16Aero already knows its xcgr reference).

## Decision C — close the silent-failure hole

Add a load-time probe: when a law that allocates binds components, query
`controlEffectiveness` once at a synthetic healthy condition (qbar > 0, thrust
> 0 unavailable at load — so probe aero-only columns) and throw if a declared
Surface control channel reports no column from any component. Cheap, and turns
"loads fine then departs" into a load error naming the model. (Gimbal columns
are legitimately zero at load — exempt Gimbal-kind channels.)

## Execution order (any option)

1. Effectiveness columns + unit tests (extend test_allocator's effectiveness
   section with AircraftAero/F16Aero cases checked against the derivative
   inputs / finite differences of the fixture tables). Behavior-neutral:
   golden gate must stay 15/15 byte-identical.
2. The load-time probe (C), also behavior-neutral for correct configs.
3. Trainer conversion (stable, forgiving): swap to the new law, tune the inner
   accel gains (outer gains carry over), prove `aircraft_cruise` +
   `mixed_traffic` assertions and flight quality, re-baseline those goldens.
4. F-16 conversion (the SAS): same law, careful inner-loop tuning at high
   bandwidth; prove `f16_cruise` (150±15 m/s, 3500±60 m) and `f16_turn`;
   re-baseline + regenerate the f16_turn gallery page.
5. Afterward (cleanup sweep): aircraft_pid, rocket_pid, tvc_pid all have zero
   users → delete the laws, their registry entries, and ControlLaw.h's
   direct-write wording; finish remaining mass_kg migrations.

## Recommendation

A1 + B + C, in the order above. A1 is the smallest step that completes
ADR-0004's contract on all 13 vehicles while preserving the two aircraft's
working outer-loop behavior; A2/A3 stay open as the natural shape of the
later ControllerSpec/topology work, and nothing in A1 blocks them.
