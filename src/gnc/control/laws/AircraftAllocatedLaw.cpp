#include "gnc/control/laws/AircraftAllocatedLaw.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "component/ForceComponent.h"
#include "math/Units.h"

AircraftAllocatedLaw::AircraftAllocatedLaw(const Gains& g, double allocDamping)
    : AllocatingLaw(allocDamping),
      g_(g),
      vsPid_(g.vsKp, g.vsKi, 0.0, -g.maxPitch, g.maxPitch),
      speedPid_(g.speedKp, g.speedKi, 0.0, 0.0, 1.0) {}

std::unique_ptr<ControlLaw> AircraftAllocatedLaw::fromJson(const json::Value& cfg) {
    Gains g;
    if (cfg.has("gains")) {
        const json::Value& j = cfg.at("gains");
        g.headingKp = j.num("heading_kp", g.headingKp);
        g.altToVs   = j.num("alt_to_vs", g.altToVs);
        g.vsKp      = j.num("vs_kp", g.vsKp);
        g.vsKi      = j.num("vs_ki", g.vsKi);
        g.speedKp   = j.num("speed_kp", g.speedKp);
        g.speedKi   = j.num("speed_ki", g.speedKi);
        g.pitchKp   = j.num("pitch_kp", g.pitchKp);
        g.pitchKd   = j.num("pitch_kd", g.pitchKd);
        g.pitchKi   = j.num("pitch_ki", g.pitchKi);
        g.rollKp    = j.num("roll_kp", g.rollKp);
        g.rollKd    = j.num("roll_kd", g.rollKd);
        g.yawDamp   = j.num("yaw_damp", g.yawDamp);
    }
    double damping = 1e-6;
    if (cfg.has("limits")) {
        const json::Value& j = cfg.at("limits");
        g.maxBank      = units::deg2rad(j.num("max_bank_deg", units::rad2deg(g.maxBank)));
        g.maxPitch     = units::deg2rad(j.num("max_pitch_deg", units::rad2deg(g.maxPitch)));
        g.maxClimbRate = j.num("max_climb_rate_ms", g.maxClimbRate);
        g.maxAngAccel  = units::deg2rad(
            j.num("max_ang_accel_dps2", units::rad2deg(g.maxAngAccel)));
        g.intLimit     = j.num("int_limit", g.intLimit);
        damping        = j.num("alloc_damping", damping);
    }
    return std::make_unique<AircraftAllocatedLaw>(g, damping);
}

std::vector<ChannelHandle> AircraftAllocatedLaw::bindChannels(const ChannelTable& t) {
    table_    = t;   // allocation needs the declared channel limits per step
    throttle_ = t.find(channels::kThrottle);

    // The allocator writes whichever surfaces exist; require at least one.
    std::vector<ChannelHandle> bound{ throttle_ };
    bool any = false;
    for (const char* name : { channels::kElevator, channels::kAileron,
                              channels::kRudder }) {
        const ChannelHandle h = t.find(name);
        if (h.valid()) any = true;
        bound.push_back(h);
    }
    if (!any)
        throw std::invalid_argument(
            "aircraft_allocated: the vehicle declares no control surfaces to "
            "allocate over");
    return bound;
}

void AircraftAllocatedLaw::update(const GncContext& gc, const CommandSet& cmd,
                                  ChannelValues& out) {
    const State& state = gc.state;
    const AirData& air = gc.air;
    const double dt = gc.dt;
    const Vector3 euler = state.eulerAngles();
    const double phi = euler.x, theta = euler.y, psi = euler.z;
    const double p = state.angularRate.x;
    const double q = state.angularRate.y;
    const double r = state.angularRate.z;

    // ---- Outer loops (verbatim from aircraft_pid) ----
    // Lateral: heading -> bank command.
    double phiCmd = 0.0;
    if (cmd.roll) {
        phiCmd = *cmd.roll;
    } else if (cmd.heading) {
        const double psiErr = units::wrapAngle(*cmd.heading - psi);
        phiCmd = std::clamp(g_.headingKp * psiErr, -g_.maxBank, g_.maxBank);
    }

    // Longitudinal: altitude -> climb rate -> pitch command.
    double thetaCmd = theta;   // no command -> hold current pitch
    if (cmd.pitch) {
        thetaCmd = *cmd.pitch;
    } else if (cmd.altitude) {
        const double climbRate = -state.velocity.z;             // NED: up = -z
        const double vsCmd = std::clamp(g_.altToVs * (*cmd.altitude - state.altitude()),
                                        -g_.maxClimbRate, g_.maxClimbRate);
        thetaCmd = vsPid_.update(vsCmd - climbRate, 0.0, dt);
    }
    thetaCmd = std::clamp(thetaCmd, -g_.maxPitch, g_.maxPitch);

    // Speed -> throttle (direct pass-through channel, like every converted law).
    if (cmd.throttle) {
        out.set(throttle_, std::clamp(*cmd.throttle, 0.0, 1.0));
    } else if (cmd.speed) {
        out.set(throttle_, speedPid_.update(*cmd.speed - air.airspeed, 0.0, dt));
    }

    // ---- Inner loop: attitude errors -> desired angular accelerations ----
    const auto clampA = [&](double a) {
        return std::clamp(a, -g_.maxAngAccel, g_.maxAngAccel);
    };
    const double ePitch = thetaCmd - theta;
    ziPitch_ = std::clamp(ziPitch_ + ePitch * dt, -g_.intLimit, g_.intLimit);
    const double aPitch =
        clampA(g_.pitchKp * ePitch - g_.pitchKd * q + g_.pitchKi * ziPitch_);
    const double aRoll = clampA(g_.rollKp * (phiCmd - phi) - g_.rollKd * p);
    const double aYaw  = clampA(-g_.yawDamp * r);   // damper only, no channel signs

    commandAngularAccel(Vector3(aRoll, aPitch, aYaw), gc, out);
}
