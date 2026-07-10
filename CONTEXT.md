# flightsim2

Scenario-driven multi-vehicle flight simulation. One context: the simulated world,
the physical crafts flying in it, and the GNC stacks steering them.

## Language

### The world

**Entity**:
An object existing in the simulated world: state plus an integrator, optionally
carrying a Vehicle and a GNC stack. A kinematic target is an Entity with no Vehicle.
_Avoid_: sim object, actor

**Vehicle**:
The complete physical craft that a `vehicles/*.json` file defines: mass properties,
force components, and their declared channels. Reusable across scenarios.
_Avoid_: airframe, plant, entity (a Vehicle is not the thing in the world; the Entity is)

### The craft

**ForceComponent**:
Anything that produces a wrench (force + moment) on the body from flight state,
environment, and channel inputs: aerodynamics, motors, gimbaled nozzles, RCS jets,
rotors, buoyancy. All peers behind one contract; each reports about its own
reference station.
_Avoid_: effector, aero model (as a special case), thrust term

**Channel**:
A named actuator degree of freedom declared by a ForceComponent (e.g. `elevator`,
`gimbal_pitch`, `collective`), with units and limits. Control writes channels by
name; the loader validates that every written channel has a consumer.
_Avoid_: control input (the retired fixed union struct)

**Actuator**:
The dynamics model of a single Channel — lag, rate limits, saturation — sitting
between commanded and achieved value. A property of the channel, not of the vehicle.

### The GNC stack

**Guidance**:
Decides where the vehicle should go; emits Commands from own state and the world
snapshot. Examples: flight plan following, proportional navigation.

**Command**:
A trajectory-level setpoint emitted by Guidance or a flight plan, at one of two
declared levels: attitude-level (pitch/heading/altitude holds) or
acceleration-level. Control laws declare which levels they accept; the pairing is
validated at load.
_Avoid_: setpoint, command set (as a fixed struct of every possible field)

**ControlLaw**:
Tracks Commands. Emits either pseudo-controls (for allocation) or channel values
directly — each ControlLaw declares which contract it uses.
_Avoid_: controller, autopilot (the autopilot is the whole stack, not this one piece)

**Pseudo-control**:
A desired net effect on the body — body moments (and optionally axial force) about
the CG — before any decision about which hardware produces it.
_Avoid_: virtual control, generalized force

**Allocator**:
Maps pseudo-controls onto channel commands using each ForceComponent's queried
effectiveness at the current flight condition, respecting channel limits.
_Avoid_: mixer, control mixer

### The run

**Scenario**:
A JSON-defined run: environment, the Entities in the world (each referencing a
Vehicle definition or kinematic), flight plans, guidance wiring, end conditions.

**WorldView**:
The immutable pre-step snapshot of every Entity that all Entities read during
propagation; the interaction seam that keeps multi-vehicle runs order-independent.
