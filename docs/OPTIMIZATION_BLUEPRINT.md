# Design-optimization blueprint — what to optimize, and how

Written 2026-07-26. A reference report + blueprint for the LATER project phase
that turns flightsim2 into a preliminary design tool with an optimization loop.
Nothing here is implemented yet; this document exists so that when the loop is
built, the formulation is taken from the literature instead of improvised.

Companion: `docs/ASSESSMENT-2026-07-26.md` (the structural caps of the sim that
bound what can honestly be optimized).

---

## 1. Purpose and scope

The goal is **preliminary design optimization**: given a parameterized vehicle
(mass, inertia, CG travel, motor, fins, ...), fly it in the sim, extract
performance numbers from the flight, and let an optimizer adjust the vehicle
parameters iteratively. In the literature this is **simulation-based design
optimization**, and when it spans several disciplines at once (mass +
propulsion + aero + control) it is **MDO — Multidisciplinary Design
Optimization**. It is a mature field: NASA has run "fly the sim inside an
optimizer" since the 1970s (POST — Program to Optimize Simulated Trajectories),
and the modern open-source reference is NASA's OpenMDAO.

Explicitly out of scope: modeling any real vehicle at certification fidelity.
The sim's known biases (forward Euler, ISA-only atmosphere, truth-fed guidance)
are acceptable for *ranking candidate designs* as long as every candidate is
scored the same way — that consistency argument is used throughout this report
and its limits are flagged where they matter.

A note on difficulty: this problem is **not NP-hard** in the formal sense.
NP-hardness belongs to combinatorial problems (discrete choices exploding).
Continuous design optimization is *nonconvex* (several local optima, no
certificate of a global best) and *expensive* (one function evaluation = one
sim run). The field's response is pragmatic: nobody certifies global optima in
preliminary design; they find designs better than the baseline and map the
trade space. With 5–15 design variables and a sim run measured in seconds,
hundreds to thousands of runs is routine on one machine. The single place a
genuinely combinatorial problem appears is discrete choices (section 4.4).

---

## 2. The standard formulation

Every design optimization in the literature has one shape:

```
minimize    f(x)        the OBJECTIVE        one number: "how good is this design"
subject to  g(x) <= 0   the CONSTRAINTS      "is this even a valid design"
over        x in X      the DESIGN VARIABLES the knobs the optimizer may turn
```

Glossary (plain language, used throughout):

- **Design variable (x)** — a number the optimizer is allowed to change:
  propellant mass, fin span, thrust scale. The vector of all of them is the
  *design vector*; its dimension is what sets the difficulty.
- **Objective (f)** — the single number being minimized (or maximized —
  minimize the negative). If you care about two things at once, see *Pareto
  front* below.
- **Constraint (g)** — a condition a design must satisfy to count at all:
  "static margin stays above 1 caliber for the whole burn". A design that
  violates a constraint is **infeasible** — it does not matter how good its
  objective is.
- **Active constraint** — a constraint the optimum pushes right up against.
  Preliminary design optima are almost always ON several constraints (the
  best rocket is exactly as unstable as you allow) — which is why the
  constraint set, not the objective, encodes the actual engineering.
- **Pareto front** — with two competing objectives (mass vs apogee), the curve
  of designs where improving one must worsen the other. The standard
  deliverable of a trade study.
- **Surrogate model** — a cheap statistical stand-in for f(x) fitted to the sim
  runs done so far, optimized instead of the sim when runs are expensive.
- **Derivative-free optimization** — methods that use only f(x) values, no
  gradients. The default when the evaluator is a black-box simulator like
  flightsim (table lookups make finite-difference gradients noisy anyway).

**The key lesson from the field: the objective is the boring part; the
constraints are where the engineering lives.** "Maximize apogee" alone is a
broken problem — the optimizer answers with a pencil carrying an infinite
motor. Every formulation in section 5 is therefore written constraints-first.

---

## 3. The loop architecture, mapped onto flightsim2

The canonical MDO loop, and what already exists in this repo for each box:

```
        +-------------------------------------------------------------+
        |                                                             |
        v                                                             |
  design vector x                                                     |
        |                                                             |
        |  (a) build the vehicle            make_missiles.py /        |
        v                                   make_fleet.py pattern     |
  vehicle.json + mass_props.csv + tables                              |
        |                                                             |
        |  (b) design the controller        flightsim --linearize     |
        v                                   + design_autopilot.py     |
  gain_schedule.csv                                                   |
        |                                                             |
        |  (c) fly                          flightsim --json          |
        v                                                             |
  flight CSV + intercept result                                       |
        |                                                             |
        |  (d) extract metrics              analyze.py /              |
        v                                   monte_carlo.py            |
  f(x), g(x)                                                          |
        |                                                             |
        |  (e) propose the next x           THE ONLY MISSING BOX      |
        +------------------------------->   (~100 lines of Python)  --+
```

Two literature points about this diagram:

- **Architecture.** Running the full coupled chain (a)–(d) for every candidate
  is the **MDF architecture** (Multidisciplinary Feasible — Martins & Lambe
  2013): simplest to build, every evaluated design is a complete consistent
  vehicle, and it is the right choice here. The alternatives (IDF, collaborative
  optimization, BLISS) exist to decompose problems too large for one loop —
  irrelevant at this scale.
- **Control co-design.** Step (b) — redesigning the controller for every plant
  variant instead of freezing one controller — is called **control co-design**
  in the literature, specifically the **nested** strategy (Herber & Allison
  2019): an inner loop (LQR) fully solves the control problem for each
  candidate the outer loop proposes. This is the piece most hobby-scale
  projects cannot do because their controllers are hand-tuned; flightsim2's
  `--linearize` + `design_autopilot.py` pipeline already automates it for any
  airframe with an elevator channel. It also cleanly answers "should the
  optimizer tune the gains?" — no; gains are the inner loop's job (section
  4.5).

Consequence: the missing work is not simulation or control machinery — it is
(e) plus a hardened version of (a), i.e. a `design_vector -> vehicle folder`
generator with a stable interface (today make_missiles.py bakes its parameters
in code).

---

## 4. Which system needs which kind of optimization

Different design variables produce differently-shaped problems, and the
literature assigns different methods to each. Summary table, then notes.

| # | Variable type | Examples | Problem character | Standard method |
|---|--------------|----------|-------------------|-----------------|
| 4.1 | Sizing (mass/propulsion) | propellant mass, thrust scale, burn time, mass layout / CG travel | continuous, smooth, cheap to evaluate | Nelder–Mead / pattern search; finite-difference gradient if clean |
| 4.2 | Trajectory / mission | launch elevation, pitch-over schedule, guidance constant N', blind range, LQR design altitude | continuous, smooth-ish | same local derivative-free methods |
| 4.3 | Aero shape | fin area/span/sweep, body fineness | continuous but multimodal; needs aero table regeneration | CMA-ES or a GA; **gated on an aero generator** (section 7) |
| 4.4 | Discrete choices | fin count, motor picked from a catalog | combinatorial (the one honestly NP-hard corner) | enumeration when the set is small (it is); GA otherwise |
| 4.5 | Control gains | LQR weights, K matrices | — | **not the outer optimizer's job** — nested co-design, section 3 |
| 4.6 | Stochastic metrics | Pk, CEP, R90 under dispersions | noisy objective | Monte Carlo + common random numbers; prefer continuous metrics |

Notes:

- **4.1 / 4.2** are where the loop should start: every variable already flows
  through existing generators and scenario JSON, and the objectives (apogee,
  range, time) are smooth functions of them. Nelder–Mead (1965) or
  coordinate/pattern search (Hooke & Jeeves 1961) will just work.
- **4.3** is the classic missile-design optimization problem (fin planform
  selection — Fleeman devotes chapters to it) and the literature default is an
  evolutionary method because the landscape is multimodal. But it is *blocked*
  in this repo until a table generator exists that responds to shape changes:
  rescaling sref/cbar/bref of an existing DATCOM table set is exact only for
  uniform geometric scaling of a FIXED shape; growing the fins on the same
  body changes the coefficients themselves. Options: DATCOM.exe under wine
  (not installed), or a Barrowman-class estimator (section 7).
- **4.4**: fin count ∈ {3, 4}, motor ∈ catalog of ~10 — enumerate. Running the
  whole continuous optimization once per discrete combination is standard
  practice and removes the combinatorial problem by brute force at this size.
- **4.5**: if the outer optimizer were allowed to touch gains, every plant
  change would be confounded with a detuned controller. Nested co-design keeps
  the comparison honest: every candidate flies with *its own best* (per the
  fixed LQR weights) controller. The LQR *weights* themselves stay fixed
  across the study — they are part of the problem definition, not a variable.
- **4.6**: optimizers chase noise. Two standard defenses. (1) Use continuous
  metrics — miss distance, not binary hit/no-hit; a step function has no slope
  to follow. (2) **Common random numbers (CRN)**: score every candidate with
  the SAME set of dispersion draws (same seeds), so the difference between two
  designs is a design difference, not a re-rolled-dice difference. This is a
  textbook variance-reduction technique and `monte_carlo.py` needs only a
  fixed-seed option to support it.

---

## 5. Per-vehicle-class formulations

The core of this report. One subsection per class that exists in the repo
today, each as: objective / design variables / constraints / what the sim
already measures. Everything metric-shaped below is already in the flight CSV,
`--json` output, `analyze.py`, `monte_carlo.py`, or the Linearizer's plant —
noted per item. Constraints marked **(gap)** need work listed in section 7.

### 5.1 Sounding rocket (max apogee class)

The recommended FIRST problem: no intercept involved, so none of the
miss-distance noise issues apply, and every needed variable works today.

- **Objective:** maximize apogee for a fixed gross liftoff mass (GLOW) — or the
  dual: minimize GLOW subject to apogee ≥ requirement. (Apogee = max
  `-position.z` in the log; GLOW = row 0 of mass_props.csv.)
- **Design variables:** propellant mass, thrust-curve scale, burn time (the
  three trade against each other through the mass table + thrust CSV),
  overall geometric scale, fin scale (uniform — exact under table rescaling),
  launch elevation angle.
- **Constraints:**
  - static margin within a band (e.g. 1–3 calibers) **across the whole burn**
    — CG travel is already in the mass tables; margin vs time is computable
    from xcg(t) and the aero tables' center of pressure. Too little margin =
    unstable; too much = the vehicle weathercocks into every gust and wastes
    energy. This is THE sounding-rocket constraint.
  - max dynamic pressure `qbar` ≤ structural limit (logged every step).
  - rail-exit velocity ≥ threshold (speed when leaving the launch rail — below
    it the fins have no authority and the trajectory is wind-dominated).
    Computable from the log given a rail length.
  - fin-loading / flutter proxy: qbar at max-q below a fin structural limit.
- **Notes:** this is essentially what OpenRocket users optimize by hand;
  automating it with a pattern search is Phase 1 of section 8.

### 5.2 Interceptor missile (AAM / SAM — the fleet's guided members)

- **Objective:** minimize miss distance (continuous, preferred), or maximize
  Pk over dispersions (Monte Carlo, needs CRN), or minimize time-to-intercept
  for a fixed engagement. All three are already emitted (`InterceptResult`,
  `--json`, `monte_carlo.py` Pk/CEP/R90).
- **Design variables:** fin geometry (blocked until section 7's aero
  generator; fin *scale* works today), boost/sustain split of the thrust
  curve, ProNav navigation constant N', blind range, LQR design-point
  altitude/Mach (make_missiles `design.alt` — a real variable: the AGM lesson
  showed design point placement decides endgame behavior).
- **Constraints:**
  - max alpha seen in flight ≤ table validity / structural limit (logged).
  - actuator saturation margin: max |fin deflection| and rate observed vs the
    actuator block's limits — an optimizer will happily use 100% of the
    actuator on the nominal run, leaving nothing for dispersions; require
    e.g. ≤ 80% (channels + limits are all logged).
  - closed-loop short-period damping ratio ≥ floor, from the designed poles
    (`design_autopilot.py` computes them per candidate — a free constraint).
  - structural load proxy qbar·alpha ≤ limit (both logged; the product is the
    standard bending-load surrogate).
  - roll-rate ceiling (the known limit-cycle → gyroscopic-coupling failure
    mode; `analyze.py`'s FFT detector can flag it).
- **Honest caveat (assessment finding 5):** guidance is truth-fed — perfect
  seeker, infinite field of view, zero latency — so absolute Pk is optimistic
  by construction. *Relative* comparisons between airframes remain meaningful;
  absolute Pk numbers should never be quoted as predictions.

### 5.3 Unguided ballistic rocket (javelin_ballistic class)

- **Objective:** minimize dispersion — CEP (circular error probable: the
  radius containing 50% of impacts) under launch/wind/thrust dispersions.
  `monte_carlo.py` already computes CEP and R90.
- **Design variables:** fin scale, static margin via mass layout (move the CG
  through ballast placement in the mass table), GLOW vs impulse trade.
- **Constraints:** static margin band over the burn (as 5.1); minimum range /
  apogee so the "most accurate rocket" is not simply the shortest-flying one;
  max qbar.
- **Notes:** spin-stabilization is the classic dispersion fix but is NOT
  currently expressible (no canted-fin roll moment or spin model at launch) —
  if wanted, that is a small aero-component feature, listed in section 7.

### 5.4 Ground-attack missile (AGM / SSM)

- **Objective:** maximize impact velocity at a required dive angle (penetration
  proxy), or minimize CEP for the fixed target. Terminal state comes from the
  last log rows / intercept relPos.
- **Design variables:** thrust profile (loft vs direct), LQR design altitude
  (the low-altitude design point — the hard-won make_missiles lesson, now a
  variable), blind range, top-attack pitch-over geometry in the flight plan.
- **Constraints:** same actuator/alpha/load set as 5.2, plus dive-angle ≥
  requirement at impact; the intercept watch must keep running before the
  ground-impact kill (already the case in `Simulation::step`).

### 5.5 TVC launch vehicle (atlas_tvc / triax_probe / vulcan_hybrid class)

- **Objective:** maximize delivered velocity at a target altitude (the
  gravity-loss trade: burn hard and fight drag, or burn long and pay gravity
  losses — the classic launch-vehicle problem POST was built for), or maximize
  apogee per unit GLOW.
- **Design variables:** thrust-curve shape, propellant fraction, pitch-over
  timing and rate in the flight plan schedule, nozzle station (moves the TVC
  arm).
- **Constraints:**
  - max-q ≤ structural limit (the constraint every real launcher throttles
    for).
  - gimbal authority margin across the burn: the arm is (nozzle − xcg) and
    authority is proportional to live thrust — both ends of the burn need
    checking, since the arm grows as CG moves forward but thrust dies at
    burnout. Effectiveness is exactly what `Propulsor::controlEffectiveness`
    reports; a margin check is a log/derived metric.
  - max gimbal deflection observed ≤ e.g. 80% of the stop (logged).
  - near-neutral static margin requirement (the hard-won TVC wisdom: a stiff
    weathercock cancels gimbal authority).
- **Honest caveat (assessment finding 5):** the attitude vocabulary rate-damps
  near vertical (the Euler guard), so a true continuous pitch-over through 90°
  cannot be *commanded* today — pitch-over profiles must stay within what the
  guard allows, or that vocabulary limit gets fixed first.

### 5.6 Fixed-wing aircraft (F-16 class) — future

- **Objectives the literature uses:** range (Breguet), endurance, sustained
  turn rate, specific excess power SEP (excess thrust power per unit weight —
  the standard fighter energy metric).
- **Variables:** wing loading, thrust-to-weight, CG position (static margin —
  the F-16 model is deliberately unstable at xcg = 0.35 cbar).
- **Why it is future, not now:** the aero model is credible subsonic only (no
  Mach dependence in the tables' valid envelope), the Linearizer is
  pitch-only, and none of the airframe geometry is generated (the F-16 tables
  are fixed wind-tunnel data — there is no "make the wing bigger" path). Listed
  for completeness; do not start here.

### 5.7 Catalogs

**Objectives catalog** (all already measurable unless noted):

| Objective | Vehicle classes | Source today |
|---|---|---|
| min gross liftoff mass (GLOW) | all | mass_props.csv row 0 |
| max apogee | sounding, TVC | flight CSV (max altitude) |
| max range | ballistic, cruise | flight CSV (ground distance at impact) |
| min miss distance | interceptors | InterceptResult / `--json` |
| max Pk / min CEP, R90 | interceptors, ballistic | monte_carlo.py (needs fixed seeds for CRN) |
| min time-to-intercept | interceptors | InterceptResult.time |
| max delivered velocity @ altitude | TVC launcher | flight CSV |
| max impact velocity @ dive angle | AGM/SSM | flight CSV terminal rows |
| range / endurance / turn rate / SEP | fixed-wing | future (5.6) |

**Constraints catalog:**

| Constraint family | Concrete form | Source today |
|---|---|---|
| Static stability | static margin in [lo, hi] calibers across the burn | mass table xcg(t) + aero tables **(needs a small margin-vs-time tool)** |
| Controllability | allocator authority margin; gimbal arm × thrust ≥ demand | controlEffectiveness (logged via components) |
| Actuator | max deflection/rate observed ≤ k·limit (k ≈ 0.8) | flight CSV channel columns |
| Closed-loop dynamics | short-period damping ≥ floor; poles left-half-plane | design_autopilot.py per-candidate output |
| Loads | max qbar; max qbar·alpha | flight CSV |
| Trajectory validity | rail-exit velocity; dive angle; min range | flight CSV (rail length as input) |
| Oscillation health | no sustained limit cycle (roll!) | analyze.py FFT detector |
| Geometric sanity | fineness ratio, fin span vs body, propellant fits | generator-side checks (build (a)) |

---

## 6. Optimizer method selection guide

All literature-standard; pick by problem shape, in this order of escalation:

1. **Pattern search / Nelder–Mead** (Hooke & Jeeves 1961; Nelder & Mead 1965).
   Derivative-free local search. Right for: smooth sizing/trajectory variables
   (4.1/4.2), ≤ ~8 dimensions, single objective. Start here — trivially
   implementable or available in scipy (`minimize(method="Nelder-Mead")`,
   which is already adjacent to the repo's .venv-tools stack).
2. **CMA-ES** (Hansen — Covariance Matrix Adaptation Evolution Strategy; the
   `cma` package). The workhorse for multimodal continuous problems up to ~50
   dimensions; noise-tolerant. Right for: anything where restarts of method 1
   land in different places, and for aero-shape variables when they unlock.
3. **NSGA-II** (Deb et al. 2002; `pymoo` package). Multi-objective genetic
   algorithm producing a Pareto front directly. Right for: the trade-study
   deliverable (mass vs apogee, Pk vs GLOW). One NSGA-II study with a clean
   Pareto plot is the single best resume artifact this project can produce.
4. **Surrogate-based / EGO** (Jones, Schonlau & Welch 1998 — kriging +
   expected improvement). Only worth it when a single evaluation becomes
   expensive (large Monte Carlo per candidate). Skip unless run cost forces it.

Constraint handling for all of the above at this scale: penalty method (add a
large cost per unit violation) or simply rejecting infeasible candidates
("death penalty") — both standard; penalties behave better for evolutionary
methods. Report *which constraints are active at the optimum* — that sentence
("the optimum is actuator-rate and static-margin limited") is the actual
engineering conclusion of a study.

Noise: CRN as in 4.6; average only over a fixed common seed set; never compare
two candidates scored with different draws.

---

## 7. Gap analysis — what flightsim2 needs, per tier

Cross-referenced to `docs/ASSESSMENT-2026-07-26.md` (its numbering in
brackets). Ordered by which optimization tier each gap blocks.

**Blocking Phase 1 (first closed loop — sounding rocket):**

- **G1. Design-vector → vehicle generator with a stable interface.** Today
  `make_missiles.py` / `make_fleet.py` bake parameters in code. Needed: one
  entry point that takes a JSON/dict design vector and writes a complete
  vehicle folder (mass_props.csv, thrust CSV, rescaled aero tables,
  vehicle.json), then runs the `--linearize` + `design_autopilot.py` chain
  when the vehicle is LQR-flown. Mostly refactoring of existing generator
  code, not new physics.
- **G2. Batch runner + metrics contract.** A driver that runs N
  (scenario, vehicle) evaluations — serially is fine at first — and returns
  {objective, constraints} as JSON per candidate. `--json` and `analyze.py`
  provide the raw pieces; what is missing is the fixed contract the optimizer
  consumes, plus a static-margin-vs-time derivation (5.7 table note).

**Blocking miss-distance / Pk objectives (Phase 4):**

- **G3. Intercept segment interpolation** [assessment "actionable now"]. The
  hit test samples discrete committed states (`Simulation.cpp:94`): at
  1000+ m/s closing and dt = 5 ms, the pursuer moves 5+ m per step against a
  5 m hit sphere, so miss distance carries O(meters) sampling noise — poison
  for an optimizer chasing meter-level improvements (it optimizes the
  sampling artifact, not the design). Fix: closest-approach interpolation on
  the segment between steps. **Must land before any miss-distance objective.**
- **G4. Fixed-seed dispersions in monte_carlo.py** for common random numbers
  (4.6). Small change: seed the perturbation RNG per-candidate-index from a
  study-level seed.

**Blocking aero-shape variables (Phase 3):**

- **G5. A shape-responsive aero table generator.** Uniform rescaling of
  existing DATCOM tables is exact; shape change is not. Two literature-standard
  options: run DATCOM.exe per candidate (Windows binary; wine not installed —
  operationally painful inside a loop), or implement a **Barrowman-class
  estimator** (Barrowman 1967 — slender-body + fin lifting-surface theory;
  what OpenRocket uses, per Niskanen's technical documentation) as one more
  generator that emits the same aero_tables.csv/control_tables.csv format
  `RocketTableAero` already reads. The second fits the existing "new mass/aero
  source = a new CSV generator, not a new class" doctrine, and its fidelity is
  exactly "preliminary design" grade — matching this project's stated scope.
- (Optional, 5.3) canted-fin roll moment if spin-stabilized dispersion studies
  are ever wanted.

**Blocking specific constraint checks:**

- **G6. Linearizer growth** [assessment 1]: lateral-directional plant and TVC
  channels, so closed-loop constraints exist for roll/yaw and for the
  gimbal-flown vehicles (today: pitch + elevator only; the 5 hand-tuned fleet
  vehicles are untouchable by the design pipeline).

**Standing caveats (state in every study, no code needed):**

- Forward-Euler O(dt) bias [assessment 2]: consistent across candidates at
  fixed dt, so *ranking* survives; absolute damping/oscillation numbers are
  biased. Never mix dt values within one study.
- Truth-fed guidance [assessment 5]: relative Pk comparisons only.
- One intercept pair per scenario [assessment "smaller"]: multi-engagement
  scoring needs that limit lifted — not needed for any Phase 1–4 study.

---

## 8. Phased roadmap (the blueprint)

Each phase: entry criteria → work → deliverable (and the resume artifact it
produces). Phases are sequential; each is independently demo-able.

**Phase 0 — harness.**
Entry: none. Work: G1 (design-vector generator interface) + G2 (batch runner,
metrics JSON contract, static-margin-vs-time tool). Deliverable:
`evaluate(design_vector) -> {f, g}` callable from Python, demonstrated by
re-creating one existing fleet vehicle from a vector and scoring its flight.
*Artifact: the loop diagram of section 3 with every box real.*

**Phase 1 — first closed loop (sounding rocket, 5.1).**
Entry: Phase 0. Work: pattern search / Nelder–Mead over {propellant mass,
thrust scale, burn time, fin scale, launch elevation}; constraints as
penalties. Deliverable: baseline vs optimized rocket, convergence plot,
active-constraint statement. *Artifact: "optimizer found +X% apogee at equal
GLOW, limited by static margin and max-q" — one paragraph, two plots.*

**Phase 2 — constraints done properly + global method + trade study.**
Entry: Phase 1. Work: CMA-ES with restarts (checks whether Phase 1's optimum
was local); one NSGA-II study (GLOW vs apogee) producing a Pareto front;
carpet-plot style visualization of the design space. *Artifact: the Pareto
front figure — the classic preliminary-design deliverable.*

**Phase 3 — shape variables.**
Entry: G5 (Barrowman generator, validated against one existing DATCOM table
set for a matching geometry before being trusted). Work: add fin
area/aspect/sweep to the design vector; CMA-ES. *Artifact: optimized fin
planform with the aero-model-fidelity caveat stated.*

**Phase 4 — interceptor study.**
Entry: G3 (intercept interpolation — hard prerequisite) + G4 (CRN seeds).
Work: minimize mean miss over a fixed common seed set for the SAM engagement;
constraints per 5.2. *Artifact: Pk-vs-GLOW trade with honest
truth-fed-guidance caveat.*

Rule carried from the project's test culture: every phase's optimizer claims
get an oracle-grade check — e.g. Phase 1's optimum re-flown at dt/2 to confirm
the ranking is not an integration artifact, and one hand-computed
static-margin point per study to validate the margin tool.

---

## 9. References

- J.R.R.A. Martins, A. Ning — *Engineering Design Optimization*, Cambridge
  University Press, 2021. Free PDF from the authors (mdobook.github.io). The
  single best starting text for all of the above.
- J.R.R.A. Martins, A.B. Lambe — "Multidisciplinary Design Optimization: A
  Survey of Architectures", AIAA Journal 51(9), 2013. (MDF vs IDF etc.)
- E.L. Fleeman — *Tactical Missile Design* / *Missile Design and System
  Engineering*, AIAA Education Series. (Missile objectives, constraints, fin
  trades.)
- D.P. Raymer — *Aircraft Design: A Conceptual Approach*, AIAA. (Aircraft
  sizing, carpet plots.)
- D.R. Herber, J.T. Allison — "Nested and Simultaneous Solution Strategies for
  General Combined Plant and Control Design Problems", ASME Journal of
  Mechanical Design, 2019. (Control co-design; the nested strategy used here.)
- J.A. Nelder, R. Mead — "A Simplex Method for Function Minimization", 1965.
  R. Hooke, T.A. Jeeves — "Direct Search Solution of Numerical and Statistical
  Problems", 1961. (The two first-tool derivative-free methods.)
- N. Hansen — "The CMA Evolution Strategy: A Tutorial" (arXiv). (`cma` pkg.)
- K. Deb et al. — "A Fast and Elitist Multiobjective Genetic Algorithm:
  NSGA-II", IEEE Trans. Evolutionary Computation, 2002. (`pymoo` pkg.)
- D.R. Jones, M. Schonlau, W.J. Welch — "Efficient Global Optimization of
  Expensive Black-Box Functions", Journal of Global Optimization, 1998. (EGO.)
- NASA POST — Program to Optimize Simulated Trajectories (Brauer et al., NASA
  CR-2770, 1977) — the historical proof that "optimize through the flight sim"
  is standard practice. OpenMDAO (Gray et al., 2019) — its modern open-source
  descendant.
- J.S. Barrowman — *The Practical Calculation of the Aerodynamic
  Characteristics of Slender Finned Vehicles*, 1967. S. Niskanen — OpenRocket
  technical documentation. (The preliminary-grade aero estimator for G5.)
