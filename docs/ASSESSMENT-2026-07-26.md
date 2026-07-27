# src/ architecture assessment — 2026-07-26

Point-in-time review of `src/` (all ~6.2k lines read) against the project's
stated goal: **a tool for designing various kinds of vehicles**. Decisions
recorded here were made by Mert in the review session; statuses reflect them.

## Verdict

An excellent small **simulator** with professional-grade validation, not yet a
**design tool**. "Various kinds of vehicles" today means slender things with
fins or a gimbal (missiles, rockets, fixed-wing). The component/channel
architecture is the right skeleton for the ambition; the caps are (a) a thin
design-product surface on top of NESC-grade dynamics, and (b) three structural
decisions (integrator seam, scalar-x geometry, flat-Earth environment) that
quietly limit which vehicle classes can ever exist here.

## What is solid (don't churn this)

- `ForceComponent` contract + declare/bind channel graph + loud load-time
  failures + the authority probe (`ScenarioLoader.cpp`). A mismatched
  airframe/controller cannot fly silently open-loop. Keep this philosophy.
- ADR-0003 externalized state (pure f(x,u)) — exactly what a design tool
  needs underneath. Two-phase snapshot/commit multi-vehicle stepping.
- The test culture (external oracles, Stevens/Lewis, NESC) is the crown jewel
  and the only reason aggressive refactors are safe.

## Findings, ranked (status in brackets)

1. **Design surface is one narrow slice** [DEFERRED — Mert extends Linearizer
   later]. `Linearizer` is pitch-only and throws without an `elevator` channel
   (`Linearizer.cpp:27`) — the 5 hand-tuned fleet vehicles are exactly the
   TVC ones the pipeline can't touch. Missing products, none needing
   architecture changes: lateral-directional plant, trim envelopes across
   Mach/alt, static-margin-vs-Mach report, performance numbers (range,
   ceiling, turn rate, SEP), parameter sweeps.

2. **Integrator seam is at the wrong level; forward Euler is load-bearing**
   [ACCEPTED for now — revisit deliberately]. Two distinct facts:
   - Accuracy: Euler's O(dt) error biases oscillation/damping — the exact
     numbers a designer reads. Accepted at current dt for current vehicles.
   - Structure (the real concern): "the integrator" lives in THREE places —
     `SixDofEom::solve` (rigid body), the component-state Euler loop inside
     `Entity::propagate` (~line 137), and `ActuatorBank::step`. `SixDofEom`
     receives one pre-computed force/moment, but multi-stage integrators must
     re-evaluate forces at trial states, so upgrading later is a
     restructuring of `Entity::propagate`, not a new class behind the
     existing seam. Every golden baseline is byte-exact Euler output; a
     future switch invalidates all of them at once. Each new golden adds a
     weld to that door.

3. **Scalar-x geometry throughout** [DEFERRED — expand to vector CG later].
   `MassState::xcg` is a scalar station; CG transfer is only
   `(dx,0,0) x F` (`Entity.cpp:146`); `ComponentContext` exposes scalar
   `xcg`. Full inertia tensor IS supported (ixy/ixz/iyz) — inconsistent with
   scalar CG. Blocks: off-axis CG (capsule steering, asymmetric stores),
   force application points off the x-axis (landing gear, side boosters,
   rotors). Deepest cap on "various KINDS of vehicles".

4. **Flat-Earth, hard-coded down, ISA-only** [DEFERRED — more atmosphere and
   gravity models later]. `GravityModel` returns a magnitude; direction
   `(0,0,+g)` is baked into `Entity.cpp:122`. Atmosphere is ISA with no
   scenario selection (no hot/cold day, no density dispersions for Monte
   Carlo). Walls off long-range ballistic / orbital / rotating-Earth work
   (NESC cases 1/5–10).

5. **Truth-fed GNC, closed command vocabulary** [DEFERRED — sensor/estimator
   seam later; keep in mind when adding laws]. Laws and seekers read perfect
   state: infinite FOV, zero latency — fleet Pk is optimistic by
   construction. Every new law deepens the retrofit cost (`GncContext` /
   `GuidanceLaw` signatures). Related: `CommandSet` is hand-merged
   field-by-field in `Entity.cpp:93` (add-a-field-and-forget-the-overlay
   trap); `accepts()` is per-level not per-field; Euler-angle vocabulary +
   vertical guard means near-vertical flight is damp-only (cannot command a
   launch-vehicle pitch-over through 90°).

## Actionable now (small, high value — not yet scheduled)

- **Intercept sampling** (`Simulation.cpp:94`): hit test on discrete
  committed states; at 1000+ m/s closing and dt=5 ms the pursuer moves 5+ m
  per step vs a 5 m hit sphere — Pk/CEP carry sampling noise. Fix: segment
  closest-approach interpolation between steps. Do before trusting Monte
  Carlo comparisons between candidate designs.
- **ScheduledLaw silently maps an unsupported `speed` command to
  throttle=1.0** (`ScheduledLaw.cpp:74`) — betrays the fail-loud philosophy
  within an accepted command level.
- **`Propulsor::computeWrench` mutates `lastThrust_`** (`Propulsor.cpp:31`),
  violating the PURE contract its base class declares (documented one-step
  lag). Makes `controlEffectiveness` evaluation-order-dependent — a trap for
  the Linearizer's central differences when it grows to cover TVC.
- **Allocator**: clamp-after-solve = no redistribution when a channel
  saturates (hurts at high-alpha endgame); 6x6 path mixes N and N·m in one
  Gram with a shared ridge — works tuned, footgun for arbitrary vehicles.
- Smaller: one actuator parameter set per ChannelKind (canard+tail can't
  differ); `kMaxChannels=16` caps effector-rich vehicles; one intercept pair
  per scenario; end conditions are duration/ground/one pair only.

## Notes for the DATCOM aero work (next up)

- **Beta convention**: the sim computes `beta = atan2(v,u)` (flank angle),
  not `asin(v/V)`. Self-consistent internally, but verify which definition
  each DATCOM table generator assumed before feeding high-sideslip tables —
  they diverge at combined high alpha/beta.
- New airframes' tables are only as useful as the conclusions drawn from
  them; until the Linearizer grows (finding 1), each new DATCOM airframe
  gets a pitch plant and a flight log, nothing more.
- Existing pipeline pieces that already work: `tools/datcom/` (pydatcom),
  `datcom_export.py`, `make_fleet.py` geometric rescaling of nondimensional
  tables, `RocketTableAero` (with its deliberate negated-rudder lookup).
  Running the DATCOM solver itself needs DATCOM.exe (Windows/wine — not
  installed here).
