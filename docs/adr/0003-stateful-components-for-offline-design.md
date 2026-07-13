# Components expose internal state (derivatives form) to enable offline autopilot design

A Vehicle is mass properties + a list of ForceComponents (ADR-0001). Some
components carry internal state — engine spool, per-channel actuator lag, the
TVC one-step lag. Today each such component *integrates its own state inside
`compute()`*, so `compute()` has side effects and the true vehicle dynamics
`xdot = f(x, u)` cannot be evaluated as a pure function.

**Decision.** ForceComponents (and the per-channel `ActuatorBank`, made
stateless) expose `numStates()`, `initializeState()`, a side-effect-free
`derivatives(...) -> xdot`, and a pure `computeWrench(...)`. The Entity holds
every internal state — component spools and actuator positions — in one
augmented vector integrated alongside the vehicle's 6-DOF state. Stateless
components (all aero) report `numStates() == 0` and are unaffected.

The Entity integrates **advance-then-evaluate**: it steps each state forward,
then evaluates the wrench from the advanced state. This reproduces the legacy
self-integrating "advance internally, then report" ordering **bit-for-bit**, so
the whole refactor is byte-identical. Crucially, the sim's stepping order is
independent of the offline linearizer, which evaluates the pure `derivatives`/
`computeWrench` at a *frozen* state to build A, B — so byte-identity costs the
linearizer nothing.

**Why.** Making `f(x, u)` a pure function lets an offline design tool trim the
vehicle and numerically linearize it (build A, B by perturbation), so LQR / gain
schedules can be auto-designed for *any* vehicle on *any* axis — including
actuator dynamics — instead of today's approximate, pitch-only, actuator-blind
plant re-derived in Python (`tools/design_autopilot.py`). It also makes internal
state loggable and opens the door to a non-Euler integrator later.

**Cost / consequences.** Every stateful component is refactored in the existing
repo (not a rewrite — the mass+effectors shape already matches the goal), guarded
by a committed golden gate (`tools/check_golden.py`) over all 15 scenarios. The
advance-then-evaluate ordering keeps every scenario byte-identical, so a flipped
sign or reordered sum is caught immediately and no re-baseline is needed.

The actuator's home was a real decision: the target doc's "actuator inside each
effector component" shape does not fit this codebase's monolithic multi-channel
aero, so actuators stay a per-channel servo model (`ActuatorBank`) with their
state externalized — no vehicle-config schema change. The 1st-order servo is
externalized as-is; a 2nd-order upgrade is a localized future change.

An earlier attempt used "clean" semantics (components read the pre-step state),
which shifted every controlled scenario by one 5 ms step. That is physically
negligible, but it flipped two chaotically-sensitive intercepts (`intercept`,
`sam_intercept`) from hit to a 600–2200 m miss — those ProNav endgames sit on a
knife-edge (documented forward-Euler roll/gyroscopic marginal stability). It was
abandoned for advance-then-evaluate. Flagged separately: those two scenarios are
poor regression baselines precisely because a 5 ms perturbation changes the
outcome by kilometres.

**Considered and rejected.** Keeping self-integrating components (simpler,
already flies — four missiles auto-designed by the current Python plant all hit)
was rejected because it permanently caps auto-design at the approximate plant.
Note: the `TARGET_ARCHITECTURE.md` that proposed this shape was drafted without
access to the codebase; this decision adopts the component-state idea on its
own merits, not the whole target doc wholesale — the control-layer pieces
(WrenchCommand, ControllerSpec, TrimSolver) remain to be grilled separately.
