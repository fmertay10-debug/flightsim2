#include "gnc/RocketPidLaw.h"

#include <algorithm>

#include "math/Units.h"

RocketPidLaw::RocketPidLaw(const Gains& g)
    : g_(g),
      pitchPid_(g.pitchKp, g.pitchKi, g.pitchKd, -g.maxFin, g.maxFin),
      yawPid_(g.yawKp, g.yawKi, g.yawKd, -g.maxFin, g.maxFin) {}

std::unique_ptr<ControlLaw> RocketPidLaw::fromJson(const json::Value& cfg) {
    Gains g;
    if (cfg.has("gains")) {
        const json::Value& j = cfg.at("gains");
        g.pitchKp = j.num("pitch_kp", g.pitchKp);
        g.pitchKi = j.num("pitch_ki", g.pitchKi);
        g.pitchKd = j.num("pitch_kd", g.pitchKd);
        g.yawKp   = j.num("yaw_kp", g.yawKp);
        g.yawKi   = j.num("yaw_ki", g.yawKi);
        g.yawKd   = j.num("yaw_kd", g.yawKd);
        g.rollKp  = j.num("roll_kp", g.rollKp);
        g.rollKd  = j.num("roll_kd", g.rollKd);
    }
    if (cfg.has("limits")) {
        const json::Value& j = cfg.at("limits");
        g.maxFin        = units::deg2rad(j.num("max_fin_deg", units::rad2deg(g.maxFin)));
        g.minAirspeed   = j.num("min_airspeed_ms", g.minAirspeed);
        g.verticalGuard = units::deg2rad(j.num("vertical_guard_deg",
                                               units::rad2deg(g.verticalGuard)));
    }
    return std::make_unique<RocketPidLaw>(g);
}

std::vector<ChannelHandle> RocketPidLaw::bindChannels(const ChannelTable& t) {
    elevator_ = t.require(channels::kElevator);
    rudder_   = t.require(channels::kRudder);
    aileron_  = t.find(channels::kAileron);
    throttle_ = t.find(channels::kThrottle);
    return {elevator_, rudder_, aileron_, throttle_};
}

void RocketPidLaw::update(const State& state, const AirData& air,
                              const CommandSet& cmd, double dt,
                              ChannelValues& out) {
    // Throttle passes through regardless (solid motors ignore it anyway).
    out.set(throttle_, cmd.throttle ? std::clamp(*cmd.throttle, 0.0, 1.0) : 1.0);

    // Fins are useless below min airspeed -- hold zero, don't wind up.
    if (air.airspeed < g_.minAirspeed) return;

    const Vector3 euler = state.eulerAngles();
    const double phi = euler.x, theta = euler.y, psi = euler.z;
    const double p = state.angularRate.x;
    const double q = state.angularRate.y;
    const double r = state.angularRate.z;

    // ---- Pitch program -> elevator (positive elevator pitches DOWN) ----
    const double thetaCmd = cmd.pitch ? *cmd.pitch : theta;
    out.set(elevator_, -pitchPid_.update(thetaCmd - theta, q, dt));

    // Near vertical, roll (phi) and heading (psi) are ill-conditioned Euler
    // angles: tiny lateral tilts read as huge angle swings. Tracking them
    // there pumps energy into the airframe, so damp the body rates instead
    // and let weathercock stability keep the vehicle straight.
    if (std::abs(theta) > g_.verticalGuard) {
        out.set(rudder_,  std::clamp(g_.yawKd * r, -g_.maxFin, g_.maxFin));
        out.set(aileron_, std::clamp(-g_.rollKd * p, -g_.maxFin, g_.maxFin));
        return;
    }

    // ---- Heading hold -> rudder (positive rudder yaws LEFT) ----
    const double psiCmd = cmd.heading ? *cmd.heading : psi;
    out.set(rudder_, -yawPid_.update(units::wrapAngle(psiCmd - psi), r, dt));

    // ---- Roll hold -> aileron (positive aileron rolls RIGHT) ----
    const double phiCmd = cmd.roll ? *cmd.roll : 0.0;
    out.set(aileron_,
            std::clamp(g_.rollKp * units::wrapAngle(phiCmd - phi) - g_.rollKd * p,
                       -g_.maxFin, g_.maxFin));
}
