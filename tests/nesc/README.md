# NESC 6-DOF check-case reference data

Reference trajectories from the NASA Engineering and Safety Center
assessment "Check-Cases for Verification of 6-Degree-of-Freedom Flight
Vehicle Simulations" (NASA/TM-2015-218675, NESC-RP-12-00770). Each case
directory holds the SAME scenario flown by ~5 independent simulation tools
(JEOD/Trick, LaSRS++, MAVERIC, POST II, Core, JSBSim — identities masked as
sim_01..06). Downloaded unmodified from
https://nescacademy.nasa.gov/flightsim/2015 (2026-07-23); US-government
work, public domain.

Vendored cases (see NASA/TM-2015-218675 Vol. II, Appendix C.1, for the
definitions; the constants are Table 73):

- `Atmos_02_TumblingBrickNoDamping` — check-case 2: torque-free tumbling
  brick, no aero. Verifies the rotational EOM. Gated on the ROTATIONAL
  channels only: the references fly a rotating WGS-84 Earth, and our
  non-rotating frame differs by exactly the centrifugal/Coriolis terms
  (50 ft altitude, 2.1 ft/s east velocity over 30 s — both matching the
  analytic prediction; gravity exerts no torque, so rates are exact).
- `Atmos_03_TumblingBrickDamping` — check-case 3: the same brick with
  Clp = Cmq = Cnr = -1 damping. Verifies inertial coupling + damping.
  Rotational channels, same reasoning.
- `Atmos_04_DroppedSphereRoundNonRotation` — check-case 4: drag sphere over
  a round NON-rotating Earth, inverse-square gravitation, US-76 atmosphere.
  The one case whose environment flightsim2 represents exactly — gated on
  ALL channels.

The sim side lives in data/vehicles/nesc/ + data/scenarios/nesc_atmos_0*.json;
the comparison (tools/nesc_compare.py, ctest `nesc_case_*`) requires our run
to sit within 1.5x the reference sims' own mutual scatter per channel.
