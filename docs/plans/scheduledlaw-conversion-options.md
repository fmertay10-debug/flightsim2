# Converting the ScheduledLaw fleet to allocation (ADR-0004) — options

Status: DECIDED 2026-07-19 — **Option B**, implemented the same day. Outcome
matched the prediction: lqr rocket apogee/tracking identical, all four
missiles HIT with byte-same printed miss distances, fin deflections within
~1e-6 relative (allocation damping) of the direct-write originals.

## What's at stake

Five configs fly `ScheduledLaw` (registered as both `scheduled` and `lqr`):
`vehicles/generated/{aam,sam,agm,ssm}/vehicle.json` and
`vehicles/generated/datcom_rocket/vehicle_lqr.json`. Five scenarios gate them —
`aam_intercept`, `sam_intercept`, `agm_strike`, `ssm_strike` (all four must
HIT per test_scenario) and `lqr_rocket_launch` (apogee must match the PID
rocket's).

ScheduledLaw is the only law whose gains are *designed*, not hand-tuned:
`tools/design_autopilot.py` runs LQR on the DATCOM-derived short-period plant
over a Mach grid and writes `gain_schedule.csv`; the law interpolates
`u_fin = -(k_alpha*alpha + k_q*q + k_theta*e + k_i*z)` on Mach, mirrors pitch
onto yaw, and carries two hard-won protections (see CLAUDE.md): the roll
`qbarRef/qbar` attenuation (prevents the diving-missile roll limit cycle) and
conditional anti-windup (freeze the integrator when the fin saturates).

Converting these to allocation touches the *design pipeline*, not just tuning
— which is why they were left for last among the missiles.

## Option A — hand-tuned `allocated_attitude` (what vehicles 1–3 got)

Replace ScheduledLaw with the existing allocation law; tune each vehicle.

- **For:** completes ADR-0004 with zero new code; the interceptor showed
  hand-tuned allocation can out-track the old laws; allocation naturally fixes
  the roll limit cycle (a fixed angular-accel demand divided by effectiveness
  ∝ qbar is exactly the attenuation rollScale hacks in).
- **Against:** throws away the designed gains — `design_autopilot.py` and
  every `gain_schedule.csv` become dead weight, a step *backward* from the
  "offline pipeline designs the gains" goal. Fixed gains must be sized for the
  worst flight condition (interceptor lesson: dominate weathercock at endgame
  qbar), and the AGM/SSM dive profiles sweep a wide qbar envelope — one gain
  set may be marginal at both ends. Five vehicles × tune + prove (four must
  HIT). Alpha feedback (k_alpha) is also lost — `allocated_attitude` feeds
  back attitude/rate only.

## Option B — same designed gains, new currency (a `scheduled` law that emits WrenchCommand)  ← recommended

Keep the schedule and the state feedback exactly as designed, but convert the
law's OUTPUT from a fin angle to a moment demand: it computes the designed
deflection u per axis as today, then emits
`WrenchCommand.moment = effectiveness_column × u` (the effectiveness it
queries from the components — the same dM/du the Allocator uses). With one
effector per axis (all five vehicles), the Allocator's damped least squares
inverts that product back to ≈ u, so the flown trajectory is unchanged up to
the damping epsilon (~1e-6 relative).

- **For:** preserves the designed-gain pipeline AND retires direct-write —
  after this, every law in the sim emits WrenchCommand and ADR-0004's contract
  is complete. Near-zero flight risk for the four must-HIT missiles (goldens
  re-baseline for last-digit reasons, not behavior). rollScale and anti-windup
  carry over verbatim (they act on u before the currency conversion). One
  shared law change (~50 lines) + five config swaps.
- **Against:** it is knowingly a bridge — the law still *thinks* in fin
  angles internally and converts at the boundary. When the offline pipeline
  later learns the allocation topology (deferred by decision), this law's
  internals get replaced anyway. Redundant-effector vehicles would make the
  "convert then invert" round trip lossy — none of the five has redundancy,
  but the limitation should be documented in the law.

## Option C — offline pipeline learns the allocation topology first

`design_autopilot.py` designs in the angular-acceleration domain and emits a
ControllerSpec-style schedule for a scheduled *allocation* law; trim/linearize
eventually comes from the pure f(x,u). **Deferred by decision (2026-07-19)**
— recorded here only so the sequence is explicit: B now, C later, and C
replaces B's internals without touching configs again.

## Option D — leave the fleet on ScheduledLaw until C

- **For:** no interim work, no risk.
- **Against:** direct-write survives indefinitely; the dual contract in
  ControlLaw.h (and the retirement of the PID laws) stays blocked; ADR-0004
  stalls at 7/13 vehicles with the hardest part unstarted.

## Recommendation

**B.** It is the only option that advances ADR-0004 *and* keeps the designed
gains, and it is cheap (one law change, five config swaps, five re-baselines).
Its known throwaway cost (the internal fin-angle→moment conversion) is
precisely the piece C replaces later, so nothing is wasted: configs, schedule
CSVs, and the design tool all survive both steps.

Suggested order of execution: `datcom_rocket/vehicle.json` first (it is
rocket_pid, NOT ScheduledLaw — a sixth easy conversion that also derisks the
table-aero effectiveness columns), then the B law change proven on
`lqr_rocket_launch` (non-hitting, easiest to compare), then the four missiles.
