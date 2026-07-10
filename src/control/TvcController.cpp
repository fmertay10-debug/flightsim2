#include "control/TvcController.h"

#include <algorithm>
#include <cmath>

#include "math/Units.h"

TvcController::TvcController(const Gains& g)
    : g_(g),
      pitchPid_(g.pitchKp, g.pitchKi, g.pitchKd, -g.maxGimbal, g.maxGimbal),
      yawPid_(g.yawKp, g.yawKi, g.yawKd, -g.maxGimbal, g.maxGimbal) {}

std::unique_ptr<Controller> TvcController::fromJson(const json::Value& cfg) {
    Gains g;
    if (cfg.has("gains")) {
        const json::Value& j = cfg.at("gains");
        g.pitchKp = j.num("pitch_kp", g.pitchKp);
        g.pitchKi = j.num("pitch_ki", g.pitchKi);
        g.pitchKd = j.num("pitch_kd", g.pitchKd);
        g.yawKp   = j.num("yaw_kp", g.yawKp);
        g.yawKi   = j.num("yaw_ki", g.yawKi);
        g.yawKd   = j.num("yaw_kd", g.yawKd);
    }
    if (cfg.has("limits")) {
        const json::Value& j = cfg.at("limits");
        g.maxGimbal     = units::deg2rad(j.num("max_gimbal_deg", units::rad2deg(g.maxGimbal)));
        g.verticalGuard = units::deg2rad(j.num("vertical_guard_deg",
                                               units::rad2deg(g.verticalGuard)));
    }
    return std::make_unique<TvcController>(g);
}

ControlInput TvcController::update(const State& state, const AirData& /*air*/,
                                   const CommandSet& cmd, double dt) {
    ControlInput out;
    out.throttle = cmd.throttle ? std::clamp(*cmd.throttle, 0.0, 1.0) : 1.0;

    const Vector3 euler = state.eulerAngles();
    const double theta = euler.y, psi = euler.z;
    const double q = state.angularRate.y, r = state.angularRate.z;

    // Pitch: gimbal to drive theta -> command (+tvcPitch = nose-up, so direct).
    const double thetaCmd = cmd.pitch ? *cmd.pitch : theta;
    out.tvcPitch = pitchPid_.update(thetaCmd - theta, q, dt);

    // Near vertical the heading angle is ill-conditioned: just damp yaw rate.
    if (std::abs(theta) > g_.verticalGuard) {
        out.tvcYaw = std::clamp(-g_.yawKd * r, -g_.maxGimbal, g_.maxGimbal);
        return out;
    }

    // Yaw: gimbal to drive heading -> command (+tvcYaw = nose-right, direct).
    const double psiCmd = cmd.heading ? *cmd.heading : psi;
    out.tvcYaw = yawPid_.update(units::wrapAngle(psiCmd - psi), r, dt);
    return out;
}
