#include "gnc/ActuatorBank.h"

#include <algorithm>
#include <limits>

#include "math/Units.h"

std::unique_ptr<ActuatorBank> ActuatorBank::fromJson(const json::Value& cfg,
                                                     const ChannelTable& table) {
    return std::make_unique<ActuatorBank>(
        table,
        cfg.num("tau_s", 0.05),
        units::deg2rad(cfg.num("rate_dps", 120.0)),
        units::deg2rad(cfg.num("limit_deg", 25.0)),
        cfg.num("throttle_tau_s", 0.5),
        units::deg2rad(cfg.num("gimbal_limit_deg", 10.0)));
}

ActuatorBank::ActuatorBank(const ChannelTable& table,
                           double tau, double rateLimit, double posLimit,
                           double throttleTau, double gimbalLimit)
    : state_(table) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    servos_.reserve(table.size());
    for (int i = 0; i < table.size(); ++i) {
        switch (table.def(i).kind) {
            case ChannelKind::Surface:
                servos_.push_back({tau, rateLimit, -posLimit, posLimit, false});
                break;
            case ChannelKind::Gimbal:
                servos_.push_back({tau, rateLimit, -gimbalLimit, gimbalLimit, false});
                break;
            case ChannelKind::Throttle:
                servos_.push_back({throttleTau, kInf, 0.0, 1.0, true});
                break;
        }
    }
}

ChannelValues ActuatorBank::apply(const ChannelValues& commanded, double dt) {
    for (int i = 0; i < static_cast<int>(servos_.size()); ++i) {
        const Servo& s = servos_[i];
        const double current = state_.at(i);
        double cmd = commanded.at(i);
        if (s.clampCommand) cmd = std::clamp(cmd, s.minPos, s.maxPos);
        // First-order lag toward the command, slew-rate limited, against a stop.
        double rate = (cmd - current) / s.tau;
        rate = std::clamp(rate, -s.rateLimit, s.rateLimit);
        state_.setAt(i, std::clamp(current + rate * dt, s.minPos, s.maxPos));
    }
    return state_;
}
