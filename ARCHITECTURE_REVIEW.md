# ARCHITECTURE_REVIEW — flightsim2 vs TARGET_ARCHITECTURE.md

Analysis only; no code changed. Written 2026-07-12 against the saved
TARGET_ARCHITECTURE.md (vision: vehicle-agnostic core dynamics +
composition-based vehicles via `ForceMomentComponent`; layered
Mission→Guidance→Controller→ControlAllocator→Effectors with a virtual
`WrenchCommand`; small fixed controller-topology library; offline
TrimSolver→Linearizer→gain-synthesis emitting a `ControllerSpec`; Phase 1 =
rockets/missiles + fixed wing, DATCOM aero; rotorcraft deferred but stateful
components from day one).

**Executive summary.** The current codebase is roughly 70% of the target
already — its own ADRs 0001/0002 describe component composition and the
layered GNC stack in nearly the target's words, and both are implemented,
tested, and golden-baselined. The two genuine architectural divergences are
(1) the **component contract**: the target's `derivatives()`-form stateful
components integrated in an augmented state vector vs today's self-integrating
`compute()` — this is the deepest gap, and it is also the enabler for
everything in the target's offline pipeline; and (2) the **offline split**:
TrimSolver and a numeric Linearizer against the true `f(x,u)` do not exist —
today's `design_autopilot.py` linearizes an analytic short-period
approximation instead. Recommendation: **evolve in place**; a fresh project
would re-do finished work, discard the golden safety net, and re-derive
validated sign conventions, while the target's two real gaps are additive
refactors, not reasons to rewrite. Details and effort figures in Part 3.

---

# Part 1 — Inventory of the current project

## 1.1 Shape and dependency graph

C++17, stdlib only. One static lib (`flightsim_core`, recursive glob of
`src/`), one executable, one CTest exe per `tests/test_*.cpp`. Include
direction verified by grep — strictly one-way, no cycles (single blemish:
`core/Telemetry.h` includes `gnc/CommandSet.h`):

```
scenario/ScenarioLoader ── composition root; nothing includes it
   │
   ├─► sim/  (Simulation, Entity, WorldView, SimObserver, CsvLogger)
   │     └─► gnc/ (guidance, control laws, Allocator, ActuatorBank, CommandSet)
   │           └─► component/ ── models/, propulsion/   (via ForceComponent,
   │           └─► mass/                                 AeroModel, PropulsionModel)
   ├─► dynamics/ (EOM strategies — takes only state+wrench+mass, never Vehicle)
   └─► environment/ (ISA + gravity/wind strategies)
                └─── core/ (State, AirData, Wrench, Channel, Telemetry)
                       └─── math/, io/
```

Python sidecar (file contracts only, no build coupling):
`pydatcom` (deck gen → runs bundled DATCOM.exe → parses for006 → npz) →
`tools/datcom_export.py` (vehicle folder: tables + vehicle.json + mesh) →
`tools/design_autopilot.py` (LQR/pole placement → gain_schedule.csv) ←
consumed by `ScheduledLaw`. Downstream of the sim: `visualize.py`/`meshes.py`
(+`check_viz.mjs`), `analyze.py`, `monte_carlo.py`, `make_gallery.py`,
`make_missiles.py` (end-to-end missile builder).

## 1.2 Module inventory with port classification

Classifications are against the saved target doc.

### PORT AS-IS

**`src/math`** — Vector3/Matrix3x3/Quaternion (scalar-first), LookupTable1D/2D,
Units. Pure, dependency-free, unit-tested. Every target concept needs exactly
these types; the locked conventions (NED, scalar-first, 3-2-1) are project
conventions, not couplings.

**`src/io`** — JSONC parser + tidy-CSV reader, stdlib-only, fail-loud,
unit-tested. No opinions about anything above it.

**`src/core`** — State, AirData, Wrench, and the Channel system
(load-time named declaration + `require()` validation, index-based
`ChannelValues` in the loop, `kMaxChannels=16`). The channel table *is* the
target's `u_comp` routing made concrete — components declare the channels they
consume ("controlled vs uncontrolled = has u_comp or not" is literally
`declareChannels()` being non-empty), and the loader validates the write/read
graph so a mismatched controller/airframe pairing fails at load, not silently
open-loop. The target doc doesn't mention this validation; it must survive the
migration (defended in Part 2). Two nits to fix in passing: the Telemetry→gnc
include, and the 16-channel cap (fine for Phase-1 vehicles).

**`src/environment`** — gravity and wind already injected strategies; ISA
atmosphere is hard-coded in `Environment.cpp` (constexpr layer table, no
`AtmosphereModel` seam). Matches the target's "core dynamics owns earth model,
atmosphere, gravity, wind." Extracting an atmosphere strategy is a ~1-hour
mechanical change whenever a non-ISA model is actually wanted.

**`src/models`** (aircraft derivative aero, rocket derivative + DATCOM table
aero, F-16 wind-tunnel tables) — these are the target's `AirframeAero`
instances: stateless table/derivative lookups, exactly the "stateless
computeWrench" case, so they slot into the new contract without surgery
(shell change only when `AeroModel` is flattened — below). They are also the
least rewrite-safe asset in the repo: F16Aero validated cell-for-cell against
a fixture; RocketTableAero's rudder negation and cbar/bref scaling
sign-convention-tested; RocketAero's axisymmetric mirrors documented. The
hacks are validated physics, not liabilities. One fair criticism: the
`AeroModel` contract doesn't pin a force-sign convention, so two opposite
conventions coexist (F16 direct body-axis vs rocket leading-minus) and only
tests keep them honest.

**`src/dynamics` — the EOM cores.** `solve(state, F_body, M_body, mass,
inertia, dt)`: pure, stateless, consumes exactly what the target says core
dynamics may consume (total mass properties + total wrench + state; zero
vehicle knowledge). The rigid-body math ports untouched. What changes around
them is integration plumbing (see REFACTOR: state assembly).

**`datcom/` (pydatcom)** — self-contained toolchain with its own pytest suite
and a real-DATCOM.exe golden e2e: deck generation from a `Vehicle` geometry
dataclass, runner (timeout, CONERR detection), a 617-line for006 parser with
MATLAB-fidelity semantics and fill-mask bookkeeping, batch/ML dataset
campaigns, mesh/HTML viewers. Zero dependency on the C++ project. The target
says "DATCOM aero (rocket geometries now, fixed-wing generalization later)" —
which is precisely pydatcom's current shape (deck gen is rocket/missile-
parameterized; fixed-wing decks are the later work). Be precise about scope:
pydatcom **wraps** the USAF DATCOM executable; it is not itself an aero
solver.

**`tools/` analysis & viz** — `visualize.py`+`meshes.py`+`check_viz.mjs`
(single-file offline three.js/uPlot viewers, headlessly verified),
`analyze.py`, `monte_carlo.py`, `make_gallery.py`. They couple only to the
CsvLogger column contract and the `--json` result contract — both worth
keeping stable regardless of architecture (goldens and the committed gallery
depend on them). `check_viz.mjs` is tightly bound to `visualize.py`'s internal
JS structure; they move as a pair.

**`gnc/guidance`** — GuidanceLaw + ProNav3D/PurePursuit + FlightPlan +
factory. Already vehicle-agnostic (reads own state + WorldView, emits
Commands), already declares its emitted command level. Matches the target's
"Guidance (vehicle-agnostic)" box as-is. One extension flagged for Phase 1:
ProNav currently emits *attitude-level* commands; the target's
ThreeLoopAutopilot tracks *acceleration*, so the known follow-up
("accel-native ProNav") gets promoted from nice-to-have to required — the
vocabulary seam already exists (`CommandSet.accelUp/accelRight`,
`CommandLevel::Acceleration`, pairing validated at attach).

### PORT WITH REFACTOR

**`src/component` — the contract itself.** Current:
`compute(ComponentContext{state, air, alt, dt, xcg}, ChannelValues) → Wrench`
about `momentReferenceStation()`, plus `declareChannels()` and
`controlEffectiveness()` (∂M/∂channel columns). Target:
`numStates() / derivatives(vehicleState, env, x_comp, u_comp) → xdot_comp /
computeWrench(...) → {F,M}` + optional mass-property contribution.

The conceptual mapping is one-to-one — same inputs, same wrench output, same
own-reference-station idea — **except state handling**, and that difference is
load-bearing. Today stateful components self-integrate inside `compute()`
(F16Engine advances its spool `power_` by `ctx.dt` internally; Propulsor's
effectiveness uses last-step thrust; per-channel actuator states live in a
separate ActuatorBank), which is why `compute()` is non-const and must be
called exactly once per step. That makes the true `xdot = f(x,u)` impossible
to evaluate without side effects — which forecloses exactly what the target's
offline pipeline needs (trim, numeric linearization) and any non-Euler
integrator. The refactor: add `numStates()`/`derivatives()` to the interface,
externalize component state into an Entity-assembled augmented state vector,
make `computeWrench` const. Migration is mechanical and provably behavior-
preserving for goldens: the internal updates are already forward-Euler steps
(`power_ += rate*dt` **is** Euler on `xdot = rate`), so externalizing them
under the same Euler integrator with the same ordering reproduces results.
Stateless components (all aero) need `numStates()==0` and nothing else.
Also fold the `AeroModel`/`AeroComponent` adapter flat while touching this —
the adapter is a pure 1:1 repackaging (`AeroForces`↔`Wrench`) around a
duplicate of the same interface; keep `Propulsor` (motor + axial/gimbaled
mount) as the good composition example it is.

**`src/gnc/ActuatorBank`** — per-channel first-order lag + slew + stop,
regression-tested. In the target, actuator dynamics are component states
(`Fin (2nd-order actuator)`), not a separate layer between controller and
components. The lag/slew/stop math and the per-`ChannelKind` parameterization
port; the home changes (dissolve into effector components, or equivalently:
effector components own actuator states, the bank's tested behavior becomes
their default dynamics). This is the piece whose *placement* the target
changes most.

**`src/mass`** — `MassModel::at(t) → {mass, inertia about CG, xcg}` with the
datum-cancelling `xcg − xref` transfer, tabulated variant flying real CG
travel. The target instead offers "optional mass-property contribution
(dm/dt, cg shift, inertia)" per component (RocketMotor owns propellant state).
Reshaping: let components contribute mass properties, keep a central
aggregator. I argue in Part 2 for keeping the centralized tabulated model as a
first-class option rather than forcing all mass through components. LEAVE
BEHIND (delete, don't port): the third, legacy path — flat `mass_kg`+`inertia`
schema with `addMotorPropellant`, threaded through
`PropulsionModel::propellantMass` → `Vehicle::addMotorPropellant_` →
`VehicleFactory::buildMass`. TabulatedMassModel (or a motor mass
contribution) subsumes it; two vehicles still use it.

**`src/propulsion`** — clean strategies (`thrust(PropulsionContext)`);
SolidMotor's impulse-fraction depletion is genuinely good and becomes the
RocketMotor component's propellant-state derivative almost verbatim. Refactor
= the state externalization above (F16Engine spool is the only real stateful
case) + delete `propellantMass()` from the interface with the legacy mass
path.

**`src/sim`** — the two-phase snapshot→propagate→commit loop, WorldView,
observers, CsvLogger: keep all of it (the target doc is silent on simulation
orchestration; this is finished infrastructure the target must not erase —
Part 2). The refactor is `Entity::propagate` (`Entity.cpp:36-122`): a
five-job monolith (inline AirData computation; mass lookup; the whole GNC
sequence — flight plan → field-by-field guidance overlay → law → actuators;
gravity-seeded force summation with per-component CG transfer; telemetry;
integrate). The component-state refactor forces this method open anyway
(state assembly, derivative evaluation), so split it then: AirData helper,
a small GncStack, a force/mass aggregator. Preserve knowingly: gravity seeds
the FP summation order (that exact order reproduces goldens bit-for-bit).

**`src/gnc/control`** — the layering already matches the target
(Guidance → ControlLaw → Allocator → Channels; ADR-0002), but three gaps are
real:
1. **The virtual wrench is not first-class.** `AllocatedAttitudeLaw` computes
   desired body moments internally and owns its Allocator privately;
   `ControlLaw::update` is still (commands → channel values). Target:
   Controller *outputs* `WrenchCommand {Fx,Fy,Fz,Mx,My,Mz}`, allocation is a
   separate, per-vehicle stage. Signature-level refactor, not conceptual.
2. **Allocator scope**: moments only (`allocate(desiredMoment, …)`); no force
   allocation (throttle handled separately), no saturation redistribution
   (documented future work). Target's WrenchCommand includes forces.
3. **Topology library**: current laws ≠ target's Phase-1 set. ScheduledLaw is
   already "fixed topology + Mach-scheduled gains" in miniature (state
   feedback, gains from the offline tool) and AllocatedAttitudeLaw is a
   CascadedAttitudeAutopilot-with-allocation in miniature; but
   ThreeLoopAutopilot (missile accel tracking) and FixedWingAutopilot (TECS)
   don't exist — that's new control engineering, not plumbing.
   The direct-write PID laws are covered in Part 2 (ADR-0002 conflict).

**`src/vehicle` + `src/scenario`** — Vehicle = mass + components[] +
channels; ScenarioLoader is the sole composition root with the two-phase
channel wiring and clean-break schema check. Logic is right; shape needs work
regardless of target: one large procedural parse+wire function, path
resolution duplicated in four files, and `ScenarioLoader.h`'s doc comment
still describes the *old rejected* schema. Target additions land here too:
`VehicleDefinition` gains trim-recipe and topology-selection fields;
ControllerSpec loading.

**`tools/design_autopilot.py`** — the existing offline pipeline: builds an
augmented short-period plant [α, q, e, ∫e] per Mach from the vehicle's own
DATCOM tables + mass (central-differenced derivatives, CG transfer), runs LQR
(`control` pkg) or pole placement, writes `gain_schedule.csv`; the Python
bilinear interpolator deliberately mirrors the C++ `LookupTable2D` so design
matches runtime. This *is* the target's "gain synthesis per envelope grid
point → serialize" stage, working and validated (four missiles designed by it
hit). What it is not: a TrimSolver (none exists anywhere) or a numeric
Linearizer of the true `f(x,u)` — it linearizes an *analytic approximation*
(pitch axis only, no actuator states in the plant, schedule on Mach only,
roll/yaw hand-set). Refactor = grow it into the target pipeline once the C++
side can expose `f(x,u)` (see Part 3 order).

**`tools/datcom_export.py` + `make_missiles.py`** — the DATCOM→vehicle bridge
and the end-to-end missile builder. Keep; two debts: the C++↔Python contract
is stringly-typed on both sides (exact CSV headers, exact JSON keys, no shared
schema definition — the single most brittle coupling in the repo), and
`strip_comments`/JSONC loading is copy-pasted in ≥3 tools. Fix with one shared
`tools/common.py` + one cited schema-constants source when ControllerSpec
lands (it will add more shared keys).

### LEAVE BEHIND

The complete list — its brevity is itself a finding:
1. The legacy flat-mass / `propellantMass()` path (three files).
2. The `AeroModel`/`AeroComponent` adapter indirection.
3. `ActuatorBank` *as a separate layer* (its math survives inside effector
   components).
4. The stale schema comment in `ScenarioLoader.h`; old-schema error
   scaffolding once no old configs remain.

Nothing else qualifies. There is no vehicle-type hierarchy to leave behind —
grep confirms zero vehicle-type branching below `models/`; "rocket" and
"aircraft" survive only as registry strings. The old architecture the target
defines itself against was already migrated out (increments 1–3, ADRs
0001/0002, goldens byte-identical throughout).

## 1.3 Verification assets (what protects any migration)

- 12 CTest suites: math, atmosphere, json, sixdof, channels, actuatorbank,
  allocator (incl. effectiveness correctness), tvc, tableaero, f16aero
  (fixture-golden, cell-for-cell), channel validation (load-failure paths),
  and 9 end-to-end scenario acceptance runs (intercepts HIT, apogees, hybrid
  TVC→fin handover asserted).
- `golden/`: committed full CSV+stdout baselines for all 15 scenarios — but
  the diff harness the increment-1 plan describes is **not in the tree**;
  today goldens are a manually-used reference, not an automated gate.
- Coverage gaps: no isolated tests for any control law, any propulsion model,
  mass models, PointMass/Kinematic EOMs, FlightPlan, or
  Simulation/Entity/ScenarioLoader internals — E2E scenarios are their
  de-facto coverage. pydatcom has its own pytest suite incl. a real-solver
  golden.

---

# Part 2 — Architecture comparison

## 2.1 What the current architecture got right

1. **Component composition is done, and done well.** ForceComponent ≈
   ForceMomentComponent minus the state contract: same inputs, wrench output
   about an own reference station with uniform CG transfer, registry
   construction from explicit `"type"` strings, channels declared by
   consumers. ADR-0001's rationale (a rotor is aero *and* effector; a
   quadrotor needs four motor channels without editing a core header) is the
   target's own argument.
2. **Load-time validation of the control graph** — channels *and* command
   levels. A law paired with an airframe that can't respond throws at load
   with a listing of what IS declared. The target doc never mentions this;
   it is strictly better than the target's silence and must carry forward.
3. **Deterministic multi-vehicle simulation** — two-phase
   snapshot/propagate/commit, pure propagate, seeded FP order, stable CSV
   contract, 15 golden baselines, intercept watch ordered before ground-kill
   so diving hits register. The target doc says nothing about simulation
   orchestration, scenarios, multi-vehicle interaction, guidance targets, or
   visualization — roughly half the working system's value is outside the
   target's frame and must not be regressed in its name.
4. **The offline pipeline exists in miniature and closes the loop.**
   DATCOM run → tables → vehicle.json → LQR on the vehicle's own aero+mass →
   gain_schedule.csv → ScheduledLaw, design interpolator matching runtime
   lookup. Four missiles built this way hit. The target generalizes this; it
   does not introduce it.
5. **Validated physics with encoded gotchas** — rudder negation, cbar/bref
   scale, F16 aileron inversion, TVC arm growth, missile roll limit-cycle
   mitigations, gravity-seeded summation order. Asserted in tests; the least
   rewrite-safe asset in the repo.

## 2.2 Feature-by-feature against the target

| Target element | Current state | Verdict |
|---|---|---|
| Core dynamics: vehicle-agnostic, consumes mass+wrench+state | `EquationsOfMotion::solve(state,F,M,mass,inertia,dt)`, zero vehicle knowledge | **Already compliant** |
| Vehicle = component container, no type hierarchy | Done (ADR-0001); registry keys, explicit configs | **Already compliant** |
| ForceMomentComponent: computeWrench | `ForceComponent::compute → Wrench`, own ref station | Matches |
| ForceMomentComponent: `numStates()/derivatives()` | **Missing** — stateful components self-integrate inside `compute()` (non-const); actuator states live in a separate bank | **Deepest divergence** |
| Component mass contributions (dm/dt, cg, inertia) | Central MassModel (constant/tabulated w/ CG travel) + one legacy propellant hack | Divergent — but see defense below |
| Guidance vehicle-agnostic | ProNav3D/PurePursuit/FlightPlan, WorldView-fed, level-declared | Compliant; needs accel-native ProNav for three-loop |
| Controller outputs virtual WrenchCommand | Pseudo-controls exist *inside* AllocatedAttitudeLaw (moments only), not as a public type; ControlLaw still emits channel values | Partial — signature-level gap |
| ControlAllocator as the vehicle-specific stage | `Allocator` (damped min-norm LS over live ∂M/∂channel from components) — the hybrid TVC+fin launcher flies on it | Compliant for moments; **no force allocation, no saturation redistribution** |
| Fixed topology library: ThreeLoop / CascadedAttitude / FixedWing(TECS) | aircraft/rocket/tvc PIDs, ScheduledLaw (LQR state feedback), AllocatedAttitudeLaw | Philosophy matches (small library, genericity in parameters — CONTEXT.md agrees); **the specific Phase-1 topologies don't exist** (ThreeLoop, TECS = new control engineering) |
| ControllerSpec: topology id + param schema + gain tables over schedule vars | `gain_schedule.csv` (4 gains × Mach) hard-wired to ScheduledLaw | Prototype exists; needs the general contract (multi-var schedule e.g. Mach×alt, topology id, serialization) |
| Offline TrimSolver | **Does not exist** | New work |
| Numeric Linearizer of true `f(x,u)` | `design_autopilot.py` linearizes an *analytic* short-period approx from DATCOM tables (no actuator states, pitch only) | Divergent — target's way is strictly better (catches actuator dynamics, CG/thrust effects, any axis) **but requires the derivatives-form contract first**; current `compute()` side effects make true `f(x,u)` unevaluable |
| DATCOM aero, rocket now / fixed-wing later | pydatcom: rocket/missile deck gen + runner + parser, golden-tested | Compliant; fixed-wing deck gen is the "later" |
| Stateful components from day one (rotorcraft-ready) | Not in the required form | Same as derivatives gap |

## 2.3 Direct answers to the questions asked

**Does the current control structure separate the virtual wrench command from
effector mixing?** In concept, yes — CONTEXT.md defines "pseudo-control"
verbatim and the Allocator maps moments→channels via live effectiveness; the
hybrid launcher proves it in flight. In interface, no — the pseudo-control
never crosses a public boundary (it's a local inside AllocatedAttitudeLaw),
it's moments-only rather than a full 6-component wrench, and the dual contract
lets other laws bypass allocation entirely. Consequences of the current shape:
you can't swap allocators per vehicle, can't log the commanded wrench as
telemetry, can't unit-test a law's wrench demand without its mixer. The fix is
a signature-level restack, not a redesign.

**Does it support gain scheduling?** Yes — ScheduledLaw interpolates
LQR-designed state-feedback gains on Mach, produced by a real offline tool
whose plant deliberately matches the runtime table lookup. But scheduling is
*per-law*, hand-rolled (four 1-D tables, one topology, one schedule variable),
not a mechanism. There is no generic "any parameter of any topology, scheduled
on any envelope variable."

**How much flexibility does ControllerSpec + fixed topology add?** Genuine,
bounded value: (a) controllers become *data* — designed, serialized, and
versioned by the offline tool, with one generic runtime interpolator instead
of a `fromJson` per law; (b) multi-variable schedules (Mach×alt) fall out of
the format instead of requiring code; (c) numeric linearization against the
true dynamics catches what the analytic plant misses today (actuator lag in
the loop, real CG/thrust coupling — the current pitch-only design is exactly
where the missile roll/yaw hand-tuning pain came from); (d) the topology-id
+ parameter-schema contract is what lets trim recipes and topology selection
live in VehicleDefinition config, per the target. What it does *not* add:
expressive power beyond the topology library — and the target is explicit
and right that genericity should live in parameters, not free block
composition (the current registry laws already follow that philosophy; the
existing laws' protective idioms — vertical guard, roll-gain qbar
attenuation, anti-windup — must become topology parameters or they'll be
painfully rediscovered).

## 2.4 Where the current design should push back on the target

Requested: a real comparison, not agreement. Four pushbacks, with reasons.

1. **Keep the named-channel table + load-time validation as the `u_comp`
   fabric.** The target says only "has u_comp or not." Anonymous per-component
   input vectors would *lose* the declared/validated write-read graph — the
   feature that turns controller/airframe mismatches into load errors instead
   of silent open loops. Channels are the better-engineered version of
   `u_comp`; adopt the target's contract *on top of* channels, not instead.

2. **Don't force all mass through components.** The target's per-component
   mass contribution is right for RocketMotor propellant (and is nearly
   implemented — SolidMotor already tracks propellant by delivered impulse).
   But the centralized `TabulatedMassModel` (mass/Ixx/Iyy/Izz/xcg vs time) is
   simpler, already validated against the F-16's fractional-cbar formulation,
   and matches how real vehicle data arrives (a mass-properties table, not a
   per-part decomposition). Summing per-component inertias about a moving
   composite CG every step is fiddly bookkeeping with new failure modes. Keep
   both: components MAY contribute; a central table remains a first-class
   source. The target's "optional" wording permits this — hold it to that.

3. **Preserve ADR-0002's dual contract through the transition.** The target
   implies WrenchCommand is *the* controller output; ADR-0002 says direct-write
   laws (the PIDs, ScheduledLaw — plant signs baked into gains) stay legal, and
   carries a standing "do not clean this up." Both can be honored in time
   order: build the WrenchCommand path as the standard one (it already is, per
   ADR-0002's own framing), keep direct-write laws as re-validated regression
   vehicles until each Phase-1 topology demonstrably replaces one on its
   scenarios, then retire per-law. Forcing everything through allocation on
   day one would trade working, golden-baselined controllers for unproven ones
   — maximum risk for zero Phase-1 capability.

4. **The target underspecifies half the working system.** Scenarios,
   multi-vehicle determinism (WorldView), guidance targeting, intercept
   scoring, CSV/telemetry contracts, the visualizer/gallery, Monte-Carlo
   dispersion — none appear in the target doc, and all are load-bearing,
   finished, and tested. The migration must treat their contracts (CSV
   columns, `--json` result, scenario schema) as frozen interfaces unless a
   change is deliberate.

Concessions, equally honest: the derivatives-form component contract is
simply better than the current self-integrating one — it unlocks trim,
linearization, non-Euler integrators, and clean state bookkeeping, and the
current design cannot get there by configuration. The numeric-linearizer
pipeline is better than the analytic short-period tool. Actuators-as-
component-states is more honest than a side-band ActuatorBank (actuator
states become visible to the linearizer — today's designs ignore them).
These three are the real content of the target, and they're worth doing.

---

# Part 3 — Recommendation

## 3.1 Evolve in place vs fresh project + selective migration

**Evolve in place. It is less total work and much less risk.**

- **The inventory says so.** PORT AS-IS covers math, core, io, environment,
  dynamics cores, all aero models, guidance, all of datcom/, and most of
  tools/ — the large majority of the tree. PORT WITH REFACTOR is a handful of
  bounded, interface-level changes. LEAVE BEHIND is four small items. A fresh
  project means copying ~80% verbatim, then performing the *same* refactors —
  in a repo with no golden baselines wired, no CI habits, no history.
- **The risk lives exactly where rewrites bleed**: sign conventions and FP
  determinism. The current tree has them asserted (f16aero fixture, tableaero
  sign tests, hybrid handover assertions, 15 golden baselines). A fresh tree
  re-derives them the hard way; the goldens die with a rewrite.
- **The two real target gaps are additive.** The derivatives-form contract is
  an interface migration with a provable Euler-equivalence path (the internal
  updates already ARE Euler steps — externalizing them under the same
  integrator and ordering reproduces goldens). The offline pipeline is new
  code either way; it lands faster next to a working sim it can call.
- Rough effort, evolve-in-place, to full Phase-1 target compliance:
  - Golden-diff harness committed + wired to CTest (prerequisite): **~1 day**.
  - Derivatives-form contract + state assembly + Entity split + ActuatorBank
    dissolution, golden-guarded: **~1–2 weeks**.
  - WrenchCommand as first-class + Allocator force extension + restack:
    **~1 week**.
  - ControllerSpec format + runtime executor subsuming ScheduledLaw: **~1 week**.
  - TrimSolver + numeric Linearizer (C++ exposes `f(x,u)`; Python drives) +
    design_autopilot generalization: **~2 weeks**.
  - ThreeLoopAutopilot + accel-native ProNav; FixedWingAutopilot/TECS:
    **~2–3 weeks** (control engineering + validation scenarios, not plumbing).

  Total ≈ **7–10 weeks**. A fresh project needs the same 7–10 weeks *plus*
  re-porting and re-validating everything PORT AS-IS and rebuilding the
  scenario/multi-vehicle/viz/test infrastructure the target doc doesn't
  cover: realistically **+6–10 weeks** and a long tail of re-discovered sign
  bugs. There is no version of the arithmetic where fresh wins.
- What would flip this: only a spine change the target doesn't actually ask
  for — multibody dynamics, an ECS runtime, variable-step integration as a
  hard requirement. The saved target asks for none of these.

## 3.2 Proposed order of work (in place; each step golden-guarded)

0. **Phase 0 (do first regardless):** commit the golden-diff runner
   (increment-1 plan describes it; it's absent), wire into CTest. Fix the
   stale ScenarioLoader.h comment. Shared `tools/common.py` for JSONC/paths.
1. **Component-state contract:** add `numStates()`/`derivatives()`/const
   `computeWrench` to ForceComponent; Entity assembles the augmented state
   vector and integrates it (Euler, same ordering — byte-identical goldens as
   the acceptance test). Migrate F16Engine, then actuator channels
   (ActuatorBank dissolves into effector components), then Propulsor's thrust
   lag. Flatten AeroModel→ForceComponent. Delete the legacy mass path;
   optional component mass contributions with the central model retained.
2. **Control restack:** `WrenchCommand` type; ControlLaw variants emit it;
   Allocator handles force+moment; direct-write laws untouched (dual
   contract).
3. **Offline pipeline:** expose `f(x,u)` evaluation from the C++ core
   (CLI `flightsim --linearize` or a thin binding); TrimSolver; numeric
   Linearizer; ControllerSpec schema (topology id + param schema + gain
   tables over schedule vars); runtime executor; ScheduledLaw becomes the
   first ControllerSpec topology.
4. **Topology library:** ThreeLoopAutopilot + accel-native ProNav (validate
   on the four missile scenarios); FixedWingAutopilot/TECS (validate on
   f16/trainer scenarios).
5. **Later, per target scope:** fixed-wing DATCOM deck generalization in
   pydatcom; rotorcraft components ride on the state contract from step 1.

If you overrule the recommendation and want a fresh repo anyway: build the
skeleton first — per-module CMake targets (`fs_math`, `fs_core`, …) so include
direction is build-enforced, CTest + golden runner from day one, the shared
schema-constants module — then move modules in dependency order (math/core/io
→ environment/dynamics → component/mass/propulsion/models with their fixture
tests first → sim/vehicle/scenario → gnc last, where the restack happens),
re-establishing goldens at each step. datcom/ and tools/ are folder copies at
any point.

## 3.3 Open questions / decisions needed before Phase 3

1. **Golden policy through the component-state migration:** hold byte-identical
   (achievable via Euler-equivalence, constrains implementation order and FP
   summation), or accept documented one-time re-baselines at named steps?
   I recommend byte-identical through step 1, re-baseline allowed at step 2+
   only where a fix is deliberate.
2. **Mass properties:** confirm the hybrid (components MAY contribute;
   central TabulatedMassModel stays first-class) vs strict per-component
   decomposition. Affects step 1 scope.
3. **Fate of direct-write laws:** keep indefinitely per ADR-0002, or retire
   each once a ControllerSpec topology beats it on its own scenarios? (My
   recommendation: the latter, but the decision contradicts a recorded ADR
   and should be yours, recorded as ADR-0003.)
4. **ControllerSpec serialization:** JSON (fits the vehicle-file ecosystem and
   `//`-comment tooling) vs CSV-per-table like today? And the Phase-1 schedule
   variables — Mach only, or Mach×alt from the start (the AGM low-altitude
   design pain suggests alt matters)?
5. **`f(x,u)` exposure for trim/linearization:** C++ CLI (`--trim`,
   `--linearize`, JSON in/out — keeps Python thin, one source of truth) vs a
   pybind-style binding (faster iteration, adds a build dependency — breaks
   the "stdlib only" rule)? I recommend the CLI.
6. **Actuator dynamics home:** confirm dissolving ActuatorBank into effector
   components (target's shape) vs keeping the bank and only *modeling* it in
   the linearizer. Dissolution is more honest but touches every vehicle
   config's `gnc.actuator` block — schema migration needed.
7. **ThreeLoopAutopilot commands:** accel-native ProNav becomes required for
   it — confirm promoting that known follow-up into Phase 1.
8. **Integrator ambitions:** the derivatives contract *enables* RK4; the
   locked convention is forward Euler and all goldens depend on it. Is a
   selectable integrator in scope at all, or Euler-only until further notice?
   (I recommend: build the seam in step 1 since it's nearly free, ship
   Euler-only.)
9. **NavState/estimation:** ADR-0002 deferred it; the target doc doesn't
   mention sensors. Confirm it stays deferred through Phase 1.
10. **Fixed-wing DATCOM generalization:** Phase-1-adjacent or strictly later?
    It's the only pydatcom work implied by the target.
