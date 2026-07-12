#include "gnc/control/laws/AircraftPidLaw.h"

#include <algorithm>
#include <cmath>

#include "math/Units.h"

AircraftPidLaw::AircraftPidLaw(const Gains& g)
    : g_(g),
      pitchPid_(g.pitchKp, g.pitchKi, g.pitchKd, -g.maxElevator, g.maxElevator),
      vsPid_(g.vsKp, g.vsKi, 0.0, -g.maxPitch, g.maxPitch),
      speedPid_(g.speedKp, g.speedKi, 0.0, 0.0, 1.0) {}

std::unique_ptr<ControlLaw> AircraftPidLaw::fromJson(const json::Value& cfg) {
    Gains g;
    if (cfg.has("gains")) {
        const json::Value& j = cfg.at("gains");
        g.pitchKp   = j.num("pitch_kp", g.pitchKp);
        g.pitchKi   = j.num("pitch_ki", g.pitchKi);
        g.pitchKd   = j.num("pitch_kd", g.pitchKd);
        g.rollKp    = j.num("roll_kp", g.rollKp);
        g.rollKd    = j.num("roll_kd", g.rollKd);
        g.headingKp = j.num("heading_kp", g.headingKp);
        g.altToVs   = j.num("alt_to_vs", g.altToVs);
        g.vsKp      = j.num("vs_kp", g.vsKp);
        g.vsKi      = j.num("vs_ki", g.vsKi);
        g.speedKp   = j.num("speed_kp", g.speedKp);
        g.speedKi   = j.num("speed_ki", g.speedKi);
        g.yawDamper = j.num("yaw_damper", g.yawDamper);
    }
    if (cfg.has("limits")) {
        const json::Value& j = cfg.at("limits");
        g.maxBank      = units::deg2rad(j.num("max_bank_deg",     units::rad2deg(g.maxBank)));
        g.maxPitch     = units::deg2rad(j.num("max_pitch_deg",    units::rad2deg(g.maxPitch)));
        g.maxClimbRate = j.num("max_climb_rate_ms", g.maxClimbRate);
        g.maxElevator  = units::deg2rad(j.num("max_elevator_deg", units::rad2deg(g.maxElevator)));
        g.maxAileron   = units::deg2rad(j.num("max_aileron_deg",  units::rad2deg(g.maxAileron)));
        g.maxRudder    = units::deg2rad(j.num("max_rudder_deg",   units::rad2deg(g.maxRudder)));
    }
    return std::make_unique<AircraftPidLaw>(g);
}

std::vector<ChannelHandle> AircraftPidLaw::bindChannels(const ChannelTable& t) {
    elevator_ = t.require(channels::kElevator);
    aileron_  = t.require(channels::kAileron);
    rudder_   = t.require(channels::kRudder);
    throttle_ = t.find(channels::kThrottle);
    return {elevator_, aileron_, rudder_, throttle_};
}

void AircraftPidLaw::update(const GncContext& gc, const CommandSet& cmd,
                      ChannelValues& out) {
    const State& state = gc.state;
    const AirData& air = gc.air;
    const double dt = gc.dt;
    const Vector3 euler = state.eulerAngles();
    const double phi = euler.x, theta = euler.y, psi = euler.z;
    const double p = state.angularRate.x;
    const double q = state.angularRate.y;
    const double r = state.angularRate.z;

    // ---- Lateral: heading -> bank -> aileron ----
    double phiCmd = 0.0;
    if (cmd.roll) {
        phiCmd = *cmd.roll;
    } else if (cmd.heading) {
        const double psiErr = units::wrapAngle(*cmd.heading - psi);
        phiCmd = std::clamp(g_.headingKp * psiErr, -g_.maxBank, g_.maxBank);
    }
    out.set(aileron_, std::clamp(g_.rollKp * (phiCmd - phi) - g_.rollKd * p,
                                 -g_.maxAileron, g_.maxAileron));

    // ---- Longitudinal: altitude -> climb rate -> pitch -> elevator ----
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
    // Elevator sign: positive elevator pitches DOWN (cmde < 0), so flip.
    out.set(elevator_, -pitchPid_.update(thetaCmd - theta, q, dt));

    // ---- Speed -> throttle ----
    if (cmd.throttle) {
        out.set(throttle_, std::clamp(*cmd.throttle, 0.0, 1.0));
    } else if (cmd.speed) {
        out.set(throttle_, speedPid_.update(*cmd.speed - air.airspeed, 0.0, dt));
    }

    // ---- Yaw damper ----
    // rudder > 0 yields a nose-left moment (cndr < 0), so +r feedback damps.
    out.set(rudder_, std::clamp(g_.yawDamper * r, -g_.maxRudder, g_.maxRudder));
}
