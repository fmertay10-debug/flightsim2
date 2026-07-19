# Target Architecture (Reference — do not implement yet)

> **Status (2026-07-19):** aspirational north-star, drafted before the
> implementation existed; names and details differ from the code. The ADRs in
> `docs/adr/` + the code are the source of truth. Progress toward this shape:
> externalized component state / pure f(x,u) is DONE (ADR-0003);
> allocation-only control is IN PROGRESS, 1 of 6 laws converted (ADR-0004);
> ControllerSpec / TrimSolver / Linearizer are NOT BUILT yet.

## Vision
Generic flight vehicle design & simulation framework. Two top-level domains:

1. **Core Dynamics (vehicle-agnostic):** 6DoF EOM integration, earth model,
   atmosphere, gravity, wind. Knows NOTHING about vehicle types. Consumes only:
   total mass properties + total wrench (F, M in body axes) + state.

2. **Vehicle (composition-based):** A vehicle is a container of components,
   not a class hierarchy of vehicle types.

## Key abstraction: ForceMomentComponent
Every force/moment producer implements one interface:
- `numStates()` — components may carry internal states (actuator dynamics,
  fuel mass, spool dynamics; later: rotor flapping/inflow)
- `derivatives(vehicleState, env, x_comp, u_comp) -> xdot_comp`
- `computeWrench(vehicleState, env, x_comp, u_comp) -> {F, M}`
- optional mass-property contribution (dm/dt, cg shift, inertia)

Instances: AirframeAero (stateless table lookup), Fin (2nd-order actuator),
TVCGimbal, RocketMotor (propellant state), ControlSurface, later RotorBlade.
"Controlled vs uncontrolled" = has u_comp or not, NOT two class trees.

## Control stack (layered, only allocation is vehicle-specific)
Mission -> Guidance (vehicle-agnostic) -> Controller (fixed topology +
scheduled gains) -> ControlAllocator (vehicle-specific mixing) -> Effectors.
Controller outputs a virtual WrenchCommand {Fx,Fy,Fz,Mx,My,Mz}.

## Controller topologies: small fixed library, NOT a generic block graph
Phase 1: ThreeLoopAutopilot (missile accel tracking),
CascadedAttitudeAutopilot (angle->rate PID), FixedWingAutopilot (TECS + lateral).
Genericity lives in configuration/parameters, not free composition
(same philosophy as ArduPilot/PX4).

## Offline/online split: ControllerSpec as the shared contract
- Offline design tool: TrimSolver -> numeric Linearizer (A,B) -> gain
  synthesis (LQR/pole placement) per envelope grid point -> serialize.
- Runtime: loads ControllerSpec (topology id + parameter schema + gain
  tables over schedule vars e.g. Mach/alt), interpolates in flight.
- Trim + linearization only need xdot = f(x,u) from core dynamics ->
  fully vehicle-agnostic. Vehicle-specific part is only the trim recipe
  and topology selection, both fields in VehicleDefinition config.

## Scope
Phase 1: rockets/missiles + fixed wing. DATCOM aero (rocket geometries now,
fixed-wing generalization later). Rotorcraft deferred to Phase 2 — but the
component interface must support stateful components from day one.