#pragma once

// Everything a propulsion model may need at one step, assembled by the Entity
// and passed in whole. Each concrete model reads only what it uses: a solid
// motor reads `time` (it burns its curve regardless of command); a
// throttleable engine reads `throttle` + flight condition and advances its
// own spool state by `dt` (stateful, like the actuator -- engine dynamics
// stay INSIDE the model, no extra entry in State).
struct PropulsionContext {
    double time     = 0.0;   // burn clock = state time [s]
    double throttle = 0.0;   // commanded, normalized [0..1]
    double mach     = 0.0;   // flight Mach number [-]
    double density  = 0.0;   // local air density [kg/m^3]
    double altitude = 0.0;   // geometric altitude [m]
    double dt       = 0.0;   // integration step [s]
};

// Strategy: engine/motor model. Thrust acts along body +x through the CG
// (force only, no thrust moment). thrust() is the LEGACY, self-integrating
// entry (a throttleable engine advances its own spool by ctx.dt here); it is
// being retired in favor of the externalized-state trio below (ADR-0003).
class PropulsionModel {
public:
    virtual ~PropulsionModel() = default;

    virtual double thrust(const PropulsionContext& ctx) = 0;      // [N], legacy

    // --- Externalized spool/internal state (ADR-0003) --------------------
    // A stateful engine (F-16 spool) exposes its state so the Entity integrates
    // it in the augmented vector and thrustFromState() is a PURE function of
    // it. Stateless motors (solid, tabulated, turbojet) keep the defaults, and
    // the bridge below makes thrustFromState() identical to thrust() for them.

    virtual int  numStates() const { return 0; }
    virtual void initializeState(double* x) const { (void)x; }

    // d/dt of the internal state at this condition. PURE. Default no-op.
    virtual void derivatives(const PropulsionContext& ctx,
                             const double* x, double* xdot) const {
        (void)ctx; (void)x; (void)xdot;
    }

    // Thrust [N] from the externalized state x (length numStates(); nullptr if
    // stateless). PURE. Default BRIDGE: forward to legacy thrust() -- exact for
    // stateless models (no state to advance); a stateful model overrides this.
    virtual double thrustFromState(const PropulsionContext& ctx, const double* x) const {
        (void)x;
        return const_cast<PropulsionModel*>(this)->thrust(ctx);
    }
    // --------------------------------------------------------------------

    // Onboard propellant remaining at sim time [kg]. Only used by the legacy
    // "dry mass + motor propellant" path; tabulated mass models ignore it.
    virtual double propellantMass(double time) const { (void)time; return 0.0; }

    // Whether the owning Propulsor declares a "throttle" channel for this
    // model. The channel is the propulsion DEMAND: motors that burn a fixed
    // curve (solid) still own it -- the pass-through command stays visible in
    // telemetry.
    virtual bool hasThrottleChannel() const { return true; }
};
