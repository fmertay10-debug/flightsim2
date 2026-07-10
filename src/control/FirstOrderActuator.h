#pragma once

#include "control/Actuator.h"
#include "io/Json.h"

#include <memory>

// First-order servo on each control channel: lag (time constant tau) +
// slew-rate limit + hard position stop. The same servo dynamics apply to the
// aero surfaces and the TVC gimbal (a gimbal actuator behaves the same way);
// gimbal channels are clamped to their own position limit. Throttle gets its
// own slower lag, clamped to [0..1], with no position/rate limits beyond that.
class FirstOrderActuator : public Actuator {
public:
    FirstOrderActuator(double tau,          // time constant [s]
                       double rateLimit,    // max deflection rate [rad/s]
                       double posLimit,     // max |surface deflection| [rad]
                       double throttleTau = 0.5,
                       double gimbalLimit = 0.1745);  // max |TVC gimbal| [rad]

    // Builder for the scenario loader: reads the "actuator" config block.
    //   {"tau_s": 0.05, "rate_dps": 120, "limit_deg": 25, "throttle_tau_s": 0.5}
    static std::unique_ptr<Actuator> fromJson(const json::Value& cfg);

    ControlInput apply(const ControlInput& commanded, double dt) override;

private:
    double stepChannel(double current, double commanded, double dt, double limit) const;

    double tau_, rateLimit_, posLimit_, throttleTau_, gimbalLimit_;
    ControlInput state_;   // current actual positions
};
