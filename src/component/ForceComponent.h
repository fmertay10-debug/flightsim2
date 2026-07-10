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
    virtual Wrench compute(const ComponentContext& ctx, const ChannelValues& u) = 0;

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
};
