# Visualizer v2 — interactive portfolio-grade viewer

Decisions from the grilling session (2026-07-11):

1. **Platform**: browser, single self-contained HTML per scenario (offline,
   double-click anywhere, GitHub-Pages friendly). Vendored libs — three.js
   (3D) + uPlot (plots) — live in `tools/vendor/` and are inlined at build
   time. The hand-rolled renderer is replaced; its NED/body-frame attitude
   exactness must be re-proven in three.js via the headless-node check.
2. **Data**: extend C++ telemetry/CsvLogger with tier-3 columns — qbar, total
   aero force/moment, and per-component moment contributions (the fin-vs-
   gimbal allocation story). Goldens re-baselined once, deliberately.
   Derived in the tool: g-load, flight-path angle, LOS direction, range,
   closing speed (from the guidance/intercept pairing in the scenario).
3. **3D overlays** (all individually toggleable, drawn on the FOCUS vehicle):
   body triad, air-relative velocity vector, alpha/beta arcs, LOS line with
   range+closing label, thrust vector deflected by the live gimbal channels,
   aero force vector, attitude-setpoint ghost, trajectory trail, floating
   info label — plus FIRST-CLASS animated fin and gimbal deflection on the
   meshes (mesh format grows articulated parts with hinge axes driven by
   channel columns).
4. **Plots**: PRESETS ONLY — no parameter catalog. A comprehensive story-
   preset library (~8) must jointly cover every logged/derived quantity:
   Tracking, Air data, Control activity, Propulsion & mass, Rates,
   Trajectory (altitude/speed/ground track), Intercept (range/closing/LOS),
   Allocation (per-component moment share). Series within a preset toggle
   via legend clicks. Shared time cursor, two-way sync with the animation
   (scrub a plot -> the 3D scene follows).
5. **Focus vehicle + 3 camera modes**: selector sets where overlays draw and
   what plots default to; cameras orbit / follow / chase; non-focus vehicles
   render clean with trails; one "show on all" switch for triad/labels.
6. **Gallery**: `docs/gallery/` holds committed, self-contained HTMLs for
   selected scenarios plus an index.html — built by a gallery step in the
   tool. Clone-and-click portfolio; same folder servable by GitHub Pages.

Implementation-owned details (not re-asked): exact new column names
(`qbar`, `aero_f{x,y,z}`, `aero_m{x,y,z}`, per-component `c<i>_<type>_m{x,y,z}`),
preset contents, dark scene theme, degrees in UI, uPlot/three versions,
full-resolution plot data with frame-capped 3D animation.

## Build order (each step verifiable)

A. C++ logging: Telemetry gains aero wrench + per-component moments + qbar;
   CsvLogger appends the columns; re-run everything; re-baseline golden/.
B. Vendor three.js + uPlot; new build pipeline in visualize.py (full-res
   series for plots, frame-capped track for animation, articulated meshes).
C. 3D scene: NED-correct three.js port + meshes + overlays 1–9 + articulated
   fins/gimbal + focus/camera system; headless-node attitude check.
D. Plot system: uPlot panes, preset library, legend toggles, two-way time
   cursor.
E. UI shell: toggle panel (grouped, compact), vehicle selector, playback
   controls; polish pass (dark theme, portfolio-ready).
F. Gallery builder + docs/gallery/index.html; regenerate goldens note;
   README/CLAUDE.md updates.
