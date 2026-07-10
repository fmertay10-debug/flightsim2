#include "control/FirstOrderActuator.h"

#include <algorithm>

#include "math/Units.h"

FirstOrderActuator::FirstOrderActuator(double tau, double rateLimit,
                                       double posLimit, double throttleTau,
                                       double gimbalLimit)
    : tau_(tau), rateLimit_(rateLimit), posLimit_(posLimit),
      throttleTau_(throttleTau), gimbalLimit_(gimbalLimit) {}

std::unique_ptr<Actuator> FirstOrderActuator::fromJson(const json::Value& cfg) {
    return std::make_unique<FirstOrderActuator>(
        cfg.num("tau_s", 0.05),
        units::deg2rad(cfg.num("rate_dps", 120.0)),
        units::deg2rad(cfg.num("limit_deg", 25.0)),
        cfg.num("throttle_tau_s", 0.5),
        units::deg2rad(cfg.num("gimbal_limit_deg", 10.0)));
}

double FirstOrderActuator::stepChannel(double current, double commanded,
                                       double dt, double limit) const {
    // First-order lag toward the command, slew-rate limited, against a stop.
    double rate = (commanded - current) / tau_;
    rate = std::clamp(rate, -rateLimit_, rateLimit_);
    return std::clamp(current + rate * dt, -limit, limit);
}

ControlInput FirstOrderActuator::apply(const ControlInput& cmd, double dt) {
    state_.elevator = stepChannel(state_.elevator, cmd.elevator, dt, posLimit_);
    state_.aileron  = stepChannel(state_.aileron,  cmd.aileron,  dt, posLimit_);
    state_.rudder   = stepChannel(state_.rudder,   cmd.rudder,   dt, posLimit_);
    state_.tvcPitch = stepChannel(state_.tvcPitch, cmd.tvcPitch, dt, gimbalLimit_);
    state_.tvcYaw   = stepChannel(state_.tvcYaw,   cmd.tvcYaw,   dt, gimbalLimit_);

    const double tRate = (std::clamp(cmd.throttle, 0.0, 1.0) - state_.throttle) / throttleTau_;
    state_.throttle = std::clamp(state_.throttle + tRate * dt, 0.0, 1.0);

    return state_;
}
