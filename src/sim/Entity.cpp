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
    if (vehicle_) {
        telem_.componentNames = &vehicle_->componentNames();
        telem_.componentLoads.resize(vehicle_->components().size());

        // Lay out the externalized component-state vector (ADR-0003): one
        // contiguous slice per component, sized by numStates(), then seed each
        // from its initial condition. Stateless components contribute nothing.
        auto& comps = vehicle_->components();
        compStateOffsets_.reserve(comps.size());
        int total = 0;
        for (auto& c : comps) {
            compStateOffsets_.push_back(total);
            total += c->numStates();
        }
        compState_.assign(static_cast<std::size_t>(total), 0.0);
        compRate_.assign(static_cast<std::size_t>(total), 0.0);
        for (std::size_t i = 0; i < comps.size(); ++i)
            if (comps[i]->numStates() > 0)
                comps[i]->initializeState(compState_.data() + compStateOffsets_[i]);
        nextCompState_ = compState_;
    }
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

    // 2. Mass properties. Mass/inertia/CG are time-varying (burn); computed
    //    before control so allocation-based laws can use inertia and CG.
    MassState ms;   // defaults: mass 1, identity inertia, NaN xcg
    if (vehicle_) ms = vehicle_->massState(s.time);
    const double mass = ms.mass;

    // 3. Flight plan + guidance overlay -> control law (COMMANDED) -> actuator
    //    (ACTUAL). Guidance reads the target from the shared world snapshot
    //    and wins over the scripted plan on the fields it sets.
    ChannelValues commanded(channels_);
    CommandSet cmd;
    if (controlLaw_) {
        cmd = flightPlan_.at(s.time);
        if (guidance_) {
            const CommandSet g = guidance_->update(s, world, dt);
            if (g.pitch)      cmd.pitch      = g.pitch;
            if (g.roll)       cmd.roll       = g.roll;
            if (g.heading)    cmd.heading    = g.heading;
            if (g.altitude)   cmd.altitude   = g.altitude;
            if (g.speed)      cmd.speed      = g.speed;
            if (g.throttle)   cmd.throttle   = g.throttle;
            if (g.accelUp)    cmd.accelUp    = g.accelUp;
            if (g.accelRight) cmd.accelRight = g.accelRight;
        }
        const GncContext gctx{ s, air, ms, dt };
        controlLaw_->update(gctx, cmd, commanded);
    }
    const ChannelValues actual = actuators_ ? actuators_->apply(commanded, dt)
                                            : commanded;

    // 4. Force components (aero, motors, ...): each produces a body-frame
    //    wrench about its own reference station; transfer each to the CG:
    //      M_cg = M_ref + (xcg - xref, 0, 0) x F   (body x forward,
    //    aft-positive station). NaN reference = already about the CG.
    //    Gravity seeds the force sum (it is the ambient field, not a
    //    component); components are summed in their declared list order.
    Vector3 force = s.attitude.rotate(Vector3(0.0, 0.0, mass * g));
    Vector3 moment;
    if (vehicle_) {
        const ComponentContext cctx{ s, air, altitude, dt, ms.xcg };
        auto& comps = vehicle_->components();
        for (std::size_t i = 0; i < comps.size(); ++i) {
            const int off = compStateOffsets_[i];
            const int ns  = comps[i]->numStates();
            const double* x = ns > 0 ? compState_.data() + off : nullptr;

            Wrench w = comps[i]->computeWrench(cctx, actual, x);
            const double xref = comps[i]->momentReferenceStation();
            if (std::isfinite(xref) && std::isfinite(ms.xcg)) {
                const double dx = ms.xcg - xref;
                w.moment.y -= dx * w.force.z;
                w.moment.z += dx * w.force.y;
            }
            telem_.componentLoads[i] = w;   // CG-referenced, for observers
            force  = force + w.force;
            moment = moment + w.moment;

            // Integrate this component's own state (forward Euler, same fixed
            // dt as the EOM), staged into nextCompState_. No-op when stateless.
            if (ns > 0) {
                comps[i]->derivatives(cctx, actual, x, compRate_.data() + off);
                for (int k = 0; k < ns; ++k)
                    nextCompState_[off + k] = x[k] + compRate_[off + k] * dt;
            }
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
    return eom_->solve(s, force, moment, mass, ms.inertia, dt);
}

void Entity::setGuidance(std::unique_ptr<GuidanceLaw> guidance) {
    // Vocabulary check: the guidance law's emitted command level must be one
    // the control law tracks -- same validated-pairing idea as the channels.
    if (guidance && controlLaw_ && !controlLaw_->accepts(guidance->emits()))
        throw std::invalid_argument(
            "Entity '" + name_ + "': guidance emits a command level the "
            "control law does not accept");
    guidance_ = std::move(guidance);
}
