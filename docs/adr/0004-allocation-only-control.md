# Allocation-only control: every control law emits a WrenchCommand

ADR-0002 established a *dual contract*: a control law could emit pseudo-controls
(body moments) for the Allocator, **or** write actuator channels directly, and
the direct-write laws (the PIDs, `ScheduledLaw`) were to stay legal indefinitely.
`TARGET_ARCHITECTURE.md` instead wants a single control output — a virtual
`WrenchCommand {Fx,Fy,Fz,Mx,My,Mz}` — that a vehicle-specific `ControlAllocator`
mixes onto effectors.

**Decision.** Converge to **allocation-only**, superseding ADR-0002's "keep
direct-write forever." Every control law emits a `WrenchCommand`; the Allocator
(extended from moment-only to force+moment) maps it to channels via each
component's queried effectiveness. Direct-write laws are retired.

**Execution — vehicle-by-vehicle, never big-bang.** Extend the Allocator to
force+moment first, then convert ONE vehicle's control to allocation, re-tune and
re-prove its scenario (the four missile hits, the F-16 climb, the aircraft
cruise), re-baseline its golden, and only then move to the next. A regression is
isolated to the vehicle just touched, not smeared across all 15 scenarios.

**Why.** A single controller-output contract makes controllers airframe-agnostic
(they speak in forces/moments, not "elevator = −0.07"), which is what lets the
fixed-topology library (ThreeLoop / CascadedAttitude / TECS) and the offline
design pipeline (ADR-0003's pure `f(x,u)` → trim → linearize → gains) apply
uniformly: the pipeline designs the topology's gains against the plant, and
allocation handles the effector mapping for whatever effectors the vehicle has.

**Consequences.** The validated direct-write tuning (the missile LQR schedules,
the F-16 SAS) must be re-expressed as moment/force demands and re-proven per
vehicle. Allocation produces different channel numbers than direct-write, so the
control-tier goldens re-baseline — byte-identity is given up for the control
layer (the physics tier from ADR-0003 stays exact). The Allocator gains force
allocation and, eventually, saturation redistribution. This decision is coupled
to the still-to-be-designed `ControllerSpec` (topology id + scheduled gain
tables) that the offline pipeline emits and a generic runtime executor consumes.
