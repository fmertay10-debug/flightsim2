# F-16 aerodynamic + engine data (Stevens & Lewis low-fidelity model)

Tables for the F-16 subsonic low-fidelity model, ported into this project's
tidy-CSV schema by [`tools/f16/port_f16_tables.py`](../../tools/f16/port_f16_tables.py)
(reproducible: the script re-downloads its oracle, re-emits, and re-validates).

## Lineage & licensing

- Original data: **NASA TP-1538** (Nguyen et al., 1979 — US government work,
  public domain), as reprinted in **Stevens & Lewis, *Aircraft Control and
  Simulation*, Appendix A** (the classic table-driven model).
- Transcription oracle: **AeroBenchVVPython** (stanleybak, GPL-3). Used at
  port time only — its table functions were *evaluated* at grid breakpoints
  to recover the numbers, then the emitted CSVs were cross-checked against it
  at 400 random off-grid points per table (max deviation ~6e-17, i.e. exact).
  **No GPL code is vendored**; this repo contains only the (public-domain)
  numeric data and original loaders.

## Schema & units

All angles **radians**, lengths **m**, forces **N** (converted at port time;
original breakpoints were degrees / ft / lbf). Coefficients dimensionless;
damping derivatives per radian (used with nondimensional rates).

| File | Grid | Contents |
|---|---|---|
| `cx.csv` | α × δe (12×5) | axial-force coefficient CX(α, δe) |
| `cm.csv` | α × δe (12×5) | pitching moment CM(α, δe) |
| `cz.csv` | α (12) | normal-force static CZ(α); β/δe corrections are closed-form (see buildup) |
| `cl.csv`, `cn.csv` | α × β (12×13) | roll/yaw static moments, **antisymmetry in β pre-expanded** to ±30° so plain bilinear lookup needs no sign logic |
| `dlda.csv`, `dldr.csv`, `dnda.csv`, `dndr.csv` | α × β (12×7) | aileron/rudder roll & yaw control derivatives |
| `damping.csv` | α (12) | CXq, CYr, CYp, CZq, Clr, Clp, Cmq, Cnr, Cnp |
| `thrust.csv` | alt × Mach (6×6) | engine thrust: `idle_N`, `mil_N`, `max_N` columns |
| `reference.csv` | key,value | geometry + mass properties, SI (see below) |

Grids: α ∈ [−10°, 45°] step 5°; δe ∈ [−24°, 24°] step 12°; β ∈ ±30°;
alt ∈ [0, 15240 m] step 3048 m; Mach ∈ [0, 1] step 0.2.

## Valid envelope (do not silently exceed)

α ∈ [−10°, 45°], β ∈ [−30°, 30°], δe ∈ [−24°, 24°], subsonic (tables carry no
Mach dependence; the model is credible to ~M 0.6). The C++ `LookupTable2D`
**clamps** outside the grid, whereas the original Fortran scheme extrapolates —
stay inside the envelope so the difference never matters.

## The 6-coefficient buildup (for F16Aero — S&L Appendix A)

```
dail = δa/20°,  drdr = δr/30°,  cq = c̄·q/(2V),  b2v = b/(2V)   [rates in rad]
CXT = CX(α,δe)                              + cq·CXq(α)
CYT = −0.02·β° + 0.021·dail + 0.086·drdr    + b2v·(CYr(α)·r + CYp(α)·p)
CZT = CZ(α)·(1−(β°/57.3)²) − 0.19·(δe°/25)  + cq·CZq(α)
CLT = Cl(α,β) + dlda(α,β)·dail + dldr(α,β)·drdr + b2v·(Clr(α)·r + Clp(α)·p)
CMT = CM(α,δe)                              + cq·Cmq(α) + CZT·(xcgr−xcg)
CNT = Cn(α,β) + dnda(α,β)·dail + dndr(α,β)·drdr + b2v·(Cnr(α)·r + Cnp(α)·p)
                                            − CYT·(xcgr−xcg)·c̄/b
```
NOTE the historical unit mix: the closed-form terms marked ° take **degrees**
(β°, δe°) — convert explicitly inside F16Aero; table lookups here are radians.
Body-axis: X forward, Y right, Z **down**; CZ is negative for lift.
`tests/fixtures/f16_coeff_checks.csv` holds 20 oracle points of this full
buildup (with damping/rate terms, xcg = xcgr) for the F16Aero unit test.

## Engine model (for the F-16 engine upgrade, issue aircraft/05)

Thrust = table interpolation with power-level blending: below power 50,
`T = idle + (mil − idle)·(power/50)`; above, `T = mil + (max − mil)·((power−50)/50)`.
Power state dynamics (S&L): `Ṗ = f(P, throttle-gear)` first-order with
rate/time-constant scheduling (tgear/pdot/rtau) — implement from S&L App. A.

## reference.csv (SI)

Wing S = 27.871 m², span b = 9.144 m, c̄ = 3.450 m, `xcgr` = 0.35 (**fraction
of c̄**, not meters — unlike the missile's `xcg_m`), mass = 9295.4 kg,
Ixx/Iyy/Izz/Ixz = 12875 / 75674 / 85552 / 1331.4 kg·m² (products convention:
tensor off-diagonal = −Ixz), engine angular momentum hₑ = 216.9 kg·m²/s
(body-x; source of gyroscopic q–r coupling).
