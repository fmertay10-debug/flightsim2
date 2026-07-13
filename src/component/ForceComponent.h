#pragma once

#include <cmath>

#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "core/Wrench.h"

// Everything a force component may need at one step, assembled by the Entity.
struct ComponentContext {
    const State&   state;
    const AirData& air;
    double altitude = 0.0;   // geometric altitude [m]
    double dt       = 0.0;   // integration step [s]
    double xcg      = 0.0;   // CG station [m, nose datum, aft positive]; may be NaN
};

// One column of the control-effectiveness matrix B: how one channel moves the
// vehicle. Sensitivities are about the CG. dMoment drives attitude allocation;
// dForce (ADR-0004) lets the allocator honour a WrenchCommand's force demand
// (e.g. TVC lateral force, direct-lift). Default dForce = 0: a component that
// reports only moment keeps the moment-only allocation path unchanged.
struct ControlEffect {
    ChannelHandle channel;
    Vector3 dMoment;      // d(Mx,My,Mz)/d(channel) at the current condition [Nm/rad]
    Vector3 dForce = {};  // d(Fx,Fy,Fz)/d(channel) [N/rad]; default 0 keeps the
                          // moment-only aggregate initializers {chan, dMoment} valid
};

// Strategy: a FORCE COMPONENT -- anything that produces a body-frame wrench on
// the vehicle from flight state, environment, and channel inputs. Aerodynamics,
// motors, gimbaled nozzles, and future RCS/rotors/buoyancy are peers behind
// this one contract; the Entity sums their wrenches (gravity is applied by the
// Entity itself -- it is the ambient field, not a component).
//
// Adding a new way to produce force/moment = subclass + one
// component::Factory::registerComponent call. Nothing in the core changes.
class ForceComponent {
public:
    virtual ~ForceComponent() = default;

    // Declare the actuator channels this component consumes into the vehicle's
    // table and keep the returned handles. Called once at load.
    virtual void declareChannels(ChannelTable& table) { (void)table; }

    // Body-frame wrench about momentReferenceStation(). Non-const: stateful
    // components (engine spool) advance by ctx.dt -- called once per step.
    //
    // LEGACY ENTRY POINT (ADR-0003 migration). Self-integrating: a stateful
    // component mutates its own internals here. Being retired in favor of the
    // externalized-state contract below (numStates/derivatives/computeWrench).
    // Still the entry the Entity calls until a component is converted; do not
    // remove until every stateful component overrides computeWrench().
    virtual Wrench compute(const ComponentContext& ctx, const ChannelValues& u) = 0;

    // --- Externalized component state (ADR-0003) -------------------------
    // A component may carry internal states (engine spool, actuator lag, fuel)
    // that the Entity integrates alongside the vehicle's 6-DOF state in one
    // augmented vector, so the whole dynamics xdot = f(x,u) is a PURE function
    // the offline design tool can trim and linearize. Stateless components
    // (all aero) keep the defaults and are unaffected.

    // How many internal states this component owns. 0 (default) = stateless.
    virtual int numStates() const { return 0; }

    // Fill this component's initial state (a slice of length numStates()).
    // Called once at load, after channels bind. Default: nothing to seed.
    virtual void initializeState(double* x) const { (void)x; }

    // d/dt of this component's own state at (state, air, inputs). PURE: must
    // read x (length numStates()) and write xdot (same length) and touch no
    // member. dt in ctx is NOT used here -- the integrator applies it. Default
    // no-op (stateless, or not yet converted). This is the f(x,u) seam.
    virtual void derivatives(const ComponentContext& ctx, const ChannelValues& u,
                             const double* x, double* xdot) const {
        (void)ctx; (void)u; (void)x; (void)xdot;
    }

    // Body-frame wrench about momentReferenceStation(), given the component's
    // externalized state x (length numStates(); nullptr when stateless). PURE.
    // Default BRIDGE: forward to the legacy compute() so unconverted components
    // work unchanged during the ADR-0003 migration. The const_cast is the
    // deliberate, temporary cost of that bridge -- a converted component
    // overrides this with a genuinely pure implementation that reads x, and the
    // bridge (and compute()) go away once all stateful components are converted.
    virtual Wrench computeWrench(const ComponentContext& ctx, const ChannelValues& u,
                                 const double* x) const {
        (void)x;
        return const_cast<ForceComponent*>(this)->compute(ctx, u);
    }
    // --------------------------------------------------------------------

    // Station (meters, increasing aft, same datum as MassState::xcg) the
    // reported moment is taken about; the Entity transfers it to the current
    // CG. NaN (default) = already about the CG, no transfer.
    virtual double momentReferenceStation() const { return std::nan(""); }

    // Telemetry: propulsive thrust magnitude produced by the last compute()
    // [N]; 0 for non-propulsive components.
    virtual double thrustNewtons() const { return 0.0; }

    // Onboard propellant remaining at sim time [kg]. Only used by the legacy
    // "dry mass + motor propellant" mass path; 0 for everything else.
    virtual double propellantMass(double time) const { (void)time; return 0.0; }

    // Control effectiveness at the current condition, for allocation-based
    // control laws: append one ControlEffect per control channel this
    // component consumes (linearized about zero deflection, CG-referenced).
    // At most maxOut entries; returns the count appended. Components with no
    // control authority (or that nobody allocates over) keep the default 0.
    virtual int controlEffectiveness(const ComponentContext& ctx,
                                     ControlEffect* out, int maxOut) const {
        (void)ctx; (void)out; (void)maxOut;
        return 0;
    }
};
