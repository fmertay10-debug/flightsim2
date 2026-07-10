#include "sim/Entity.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "sim/WorldView.h"

Entity::Entity(std::string name,
               std::unique_ptr<Vehicle>           vehicle,
               std::unique_ptr<ControlLaw>        controller,
               std::unique_ptr<ActuatorBank>      actuators,
               std::unique_ptr<EquationsOfMotion> eom,
               FlightPlan                         flightPlan,
               const State&                       initialState,
               ChannelTable                       channels)
    : name_(std::move(name)),
      vehicle_(std::move(vehicle)),
      controlLaw_(std::move(controller)),
      actuators_(std::move(actuators)),
      eom_(std::move(eom)),
      flightPlan_(std::move(flightPlan)),
      channels_(std::move(channels)),
      state_(initialState)
{
    if (!eom_)
        throw std::invalid_argument("Entity '" + name_ + "': EOM is required");
    telem_.name = name_;
    telem_.channels = &channels_;
}

State Entity::propagate(const Environment& env, const WorldView& world, double dt) {
    const State& s = state_;

    // 1. Environment and air-relative flight condition at this instant.
    const double altitude = s.altitude();
    const AtmosphereState atm = env.atmosphere(altitude);
    const double g = env.gravity(altitude);

    AirData air;
    air.atmosphere = atm;
    const Vector3 vAirNed = s.velocity - env.wind(s.position, s.time);
    air.velocityBody = s.attitude.rotate(vAirNed);          // inertial -> body
    air.airspeed = air.velocityBody.norm();
    if (air.airspeed > 1e-6) {
        air.alpha = std::atan2(air.velocityBody.z, air.velocityBody.x);
        air.beta  = std::atan2(air.velocityBody.y, air.velocityBody.x);
    }
    air.mach = (atm.soundSpeed > 1e-6) ? air.airspeed / atm.soundSpeed : 0.0;
    air.qbar = 0.5 * atm.density * air.airspeed * air.airspeed;

    // 2. Flight plan + guidance overlay -> controller (COMMANDED) -> actuator
    //    (ACTUAL). Guidance reads the target from the shared world snapshot
    //    and wins over the scripted plan on the fields it sets.
    ChannelValues commanded(channels_);
    CommandSet cmd;
    if (controlLaw_) {
        cmd = flightPlan_.at(s.time);
        if (guidance_) {
            const CommandSet g = guidance_->update(s, world, dt);
            if (g.pitch)    cmd.pitch    = g.pitch;
            if (g.roll)     cmd.roll     = g.roll;
            if (g.heading)  cmd.heading  = g.heading;
            if (g.altitude) cmd.altitude = g.altitude;
            if (g.speed)    cmd.speed    = g.speed;
            if (g.throttle) cmd.throttle = g.throttle;
        }
        controlLaw_->update(s, air, cmd, dt, commanded);
    }
    const ChannelValues actual = actuators_ ? actuators_->apply(commanded, dt)
                                            : commanded;

    // 3. Mass properties. Mass/inertia/CG are time-varying (burn).
    double mass = 1.0;
    Matrix3x3 inertia = Matrix3x3::identity();
    double xcg = 0.0;
    if (vehicle_) {
        const MassState ms = vehicle_->massState(s.time);
        mass = ms.mass;
        inertia = ms.inertia;
        xcg = ms.xcg;
    }

    // 4. Force components (aero, motors, ...): each produces a body-frame
    //    wrench about its own reference station; transfer each to the CG:
    //      M_cg = M_ref + (xcg - xref, 0, 0) x F   (body x forward,
    //    aft-positive station). NaN reference = already about the CG.
    //    Gravity seeds the force sum (it is the ambient field, not a
    //    component); components are summed in their declared list order.
    Vector3 force = s.attitude.rotate(Vector3(0.0, 0.0, mass * g));
    Vector3 moment;
    if (vehicle_) {
        const ComponentContext cctx{ s, air, altitude, dt, xcg };
        for (const auto& c : vehicle_->components()) {
            Wrench w = c->compute(cctx, actual);
            const double xref = c->momentReferenceStation();
            if (std::isfinite(xref) && std::isfinite(xcg)) {
                const double dx = xcg - xref;
                w.moment.y -= dx * w.force.z;
                w.moment.z += dx * w.force.y;
            }
            force  = force + w.force;
            moment = moment + w.moment;
        }
    }

    // Telemetry snapshot for this instant, consistent at s.time.
    telem_.state      = s;
    telem_.control    = actual;
    telem_.controlCmd = commanded;
    telem_.setpoint   = cmd;
    telem_.air        = air;
    telem_.mass       = mass;
    telem_.thrust     = vehicle_ ? vehicle_->thrustNewtons() : 0.0;

    // 5. One integration step (committed later by the Simulation).
    return eom_->solve(s, force, moment, mass, inertia, dt);
}
