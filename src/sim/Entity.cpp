#include "sim/Entity.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "sim/WorldView.h"

Entity::Entity(std::string name,
               std::unique_ptr<Vehicle>           vehicle,
               std::unique_ptr<AeroModel>         aero,
               std::unique_ptr<Controller>        controller,
               std::unique_ptr<Actuator>          actuator,
               std::unique_ptr<EquationsOfMotion> eom,
               FlightPlan                         flightPlan,
               const State&                       initialState)
    : name_(std::move(name)),
      vehicle_(std::move(vehicle)),
      aero_(std::move(aero)),
      controller_(std::move(controller)),
      actuator_(std::move(actuator)),
      eom_(std::move(eom)),
      flightPlan_(std::move(flightPlan)),
      state_(initialState)
{
    if (!eom_)
        throw std::invalid_argument("Entity '" + name_ + "': EOM is required");
    telem_.name = name_;
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
    ControlInput commanded;
    CommandSet cmd;
    if (controller_) {
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
        commanded = controller_->update(s, air, cmd, dt);
    }
    const ControlInput actual = actuator_ ? actuator_->apply(commanded, dt)
                                          : commanded;

    // 3. Aerodynamic loads (body frame, about the aero moment reference).
    AeroForces aero;
    if (aero_) aero = aero_->compute(s, air, actual);

    // 4. Mass properties and thrust. Mass/inertia/CG are time-varying (burn);
    //    the engine advances its own spool state, so build its full context.
    double mass = 1.0;
    Matrix3x3 inertia = Matrix3x3::identity();
    double xcg = 0.0;
    double thrust = 0.0;
    if (vehicle_) {
        const MassState ms = vehicle_->massState(s.time);
        mass = ms.mass;
        inertia = ms.inertia;
        xcg = ms.xcg;

        PropulsionContext pc;
        pc.time = s.time;
        pc.throttle = actual.throttle;
        pc.mach = air.mach;
        pc.density = atm.density;
        pc.altitude = altitude;
        pc.dt = dt;
        thrust = vehicle_->thrust(pc);
    }

    // Transfer the aero moment from the model's reference station to the CG:
    //   M_cg = M_ref + (xcg - xref, 0, 0) x F   (body x forward, aft-positive
    //   station). Only when the model declares a finite reference; derivative
    //   models report about the CG already and opt out with NaN.
    if (aero_) {
        const double xref = aero_->momentReferenceStation();
        if (std::isfinite(xref) && std::isfinite(xcg)) {
            const double dx = xcg - xref;
            aero.moment.y -= dx * aero.force.z;
            aero.moment.z += dx * aero.force.y;
        }
    }

    // Telemetry snapshot for this instant, consistent at s.time.
    telem_.state      = s;
    telem_.control    = actual;
    telem_.controlCmd = commanded;
    telem_.setpoint   = cmd;
    telem_.air        = air;
    telem_.mass       = mass;
    telem_.thrust     = thrust;

    // 5. Control effectors turn the available thrust + commands into body-frame
    //    wrenches: an axial ThrustEffector (thrust along +x, no moment) or a
    //    TvcEffector (gimbaled thrust -> pitch/yaw moment), plus any future
    //    additive effectors. This replaces the old inline axial thrust term.
    Wrench effectorLoad;
    if (!effectors_.empty()) {
        const EffectorContext ectx{ s, air, thrust, xcg };
        for (const auto& eff : effectors_)
            effectorLoad = effectorLoad + eff->compute(ectx, actual);
    }

    // 6. Sum body-frame loads: aero + gravity (rotated in) + effectors.
    const Vector3 gravityBody = s.attitude.rotate(Vector3(0.0, 0.0, mass * g));
    const Vector3 force  = aero.force + gravityBody + effectorLoad.force;
    const Vector3 moment = aero.moment + effectorLoad.moment;

    // 7. One integration step (committed later by the Simulation).
    return eom_->solve(s, force, moment, mass, inertia, dt);
}
