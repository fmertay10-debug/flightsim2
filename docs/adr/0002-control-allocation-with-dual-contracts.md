# Control allocation layer, with direct channel writes still a legal contract

The GNC stack is Guidance → ControlLaw → Allocator → Channels. A ControlLaw tracks
Commands and emits pseudo-controls (desired body moments, optionally axial force);
the Allocator maps them onto channel commands using each ForceComponent's queried
control effectiveness (∂Wrench/∂channel) at the current flight condition,
respecting limits. This is what makes redundant/hybrid effector suites (TVC + fins
+ RCS blending with qbar and thrust) expressible with a single control law and no
mode-switching logic.

Deliberately, allocation is NOT the only contract: a ControlLaw may instead declare
that it writes channel values directly. This keeps the existing PID/LQR controllers
(whose gains bake in plant signs and map state → surface deflection) flying
unchanged during and after the migration, at the cost of two controller output
contracts coexisting. Do not "clean this up" by forcing everything through the
allocator — direct-write laws are re-validated vehicles we chose not to redesign.

Commands are two-level (attitude-level and acceleration-level); guidance laws
declare what they emit, control laws what they accept, and the loader validates the
pairing. GNC consumes true State — a NavState/estimation seam was considered and
deliberately deferred until sensors are actually needed.
