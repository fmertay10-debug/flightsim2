# Components expose internal state (derivatives form) to enable offline autopilot design

A Vehicle is mass properties + a list of ForceComponents (ADR-0001). Some
components carry internal state — engine spool, per-channel actuator lag, the
TVC one-step lag. Today each such component *integrates its own state inside
`compute()`*, so `compute()` has side effects and the true vehicle dynamics
`xdot = f(x, u)` cannot be evaluated as a pure function.

**Decision.** ForceComponents will additionally expose `numStates()` and a
side-effect-free `derivatives(vehicleState, env, x_comp, u_comp) -> xdot_comp`,
with a pure `computeWrench(...)`. The simulation integrates every component's
state together with the vehicle's 6-DOF state in one augmented vector. Stateless
components (all aero) report `numStates() == 0` and are unaffected.

**Why.** Making `f(x, u)` a pure function lets an offline design tool trim the
vehicle and numerically linearize it (build A, B by perturbation), so LQR / gain
schedules can be auto-designed for *any* vehicle on *any* axis — including
actuator dynamics — instead of today's approximate, pitch-only, actuator-blind
plant re-derived in Python (`tools/design_autopilot.py`). It also makes internal
state loggable and opens the door to a non-Euler integrator later.

**Cost / consequences.** Every stateful component is refactored. This is done
in the existing repo (not a rewrite — the mass+effectors shape already matches
the goal) under a two-tier golden gate: the physics tier (given recorded control
inputs, the body wrench + state trajectory) is held byte-identical, so a flipped
sign or reordered sum is caught immediately; the closed-loop control tier is
deliberately re-baselined as the GNC layer is redesigned. The internal updates
being replaced are already forward-Euler steps, so externalizing them under the
same integrator and ordering reproduces results exactly.

**Considered and rejected.** Keeping self-integrating components (simpler,
already flies — four missiles auto-designed by the current Python plant all hit)
was rejected because it permanently caps auto-design at the approximate plant.
Note: the `TARGET_ARCHITECTURE.md` that proposed this shape was drafted without
access to the codebase; this decision adopts the component-state idea on its
own merits, not the whole target doc wholesale — the control-layer pieces
(WrenchCommand, ControllerSpec, TrimSolver) remain to be grilled separately.
