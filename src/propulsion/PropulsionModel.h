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
// (force only, no thrust moment). thrust() is NON-const: throttleable engines
// advance internal state by ctx.dt per call, so call it once per step.
class PropulsionModel {
public:
    virtual ~PropulsionModel() = default;

    virtual double thrust(const PropulsionContext& ctx) = 0;      // [N]

    // Onboard propellant remaining at sim time [kg]. Only used by the legacy
    // "dry mass + motor propellant" path; tabulated mass models ignore it.
    virtual double propellantMass(double time) const { (void)time; return 0.0; }
};

class NoPropulsion : public PropulsionModel {
public:
    double thrust(const PropulsionContext&) override { return 0.0; }
};
