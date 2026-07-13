#pragma once

#include <memory>
#include <vector>

#include "core/Channel.h"
#include "io/Json.h"

// Per-channel actuator dynamics between COMMANDED and ACTUAL channel values:
// first-order lag + slew-rate limit + hard position stop, one servo state per
// declared channel. Replaces the monolithic FirstOrderActuator; behavior per
// channel is identical, with the parameter set picked by the channel's kind:
//   Surface  -> tau_s / rate_dps / +-limit_deg
//   Gimbal   -> tau_s / rate_dps / +-gimbal_limit_deg
//   Throttle -> throttle_tau_s, command and position clamped to [0..1],
//               no rate/position limits beyond that
// A null bank on an entity means ideal (actual == commanded).
//
// NOTE: the position stop comes from the "actuator" config block, NOT from the
// channel's declared min/max -- exactly like the old actuator. Channel limits
// stay the owning component's contract (e.g. the TVC effector clamps its own
// gimbal); unifying the two is future work, not this refactor.
class ActuatorBank {
public:
    // Reads the vehicle's "actuator" config block:
    //   {"tau_s": 0.05, "rate_dps": 120, "limit_deg": 25,
    //    "throttle_tau_s": 0.5, "gimbal_limit_deg": 10}
    static std::unique_ptr<ActuatorBank> fromJson(const json::Value& cfg,
                                                  const ChannelTable& table);

    ActuatorBank(const ChannelTable& table,
                 double tau, double rateLimit, double posLimit,
                 double throttleTau, double gimbalLimit);

    // Stateless per-channel servo step (ADR-0003): advance the CURRENT actual
    // positions toward `commanded` by dt (first-order lag + slew limit + hard
    // stop) and return the new positions. The bank owns no state -- the Entity
    // holds the actuator positions in the augmented state vector and integrates
    // them here, so the servo dynamics are visible to the offline linearizer.
    ChannelValues step(const ChannelValues& current,
                       const ChannelValues& commanded, double dt) const;

    int size() const { return static_cast<int>(servos_.size()); }

private:
    struct Servo {
        double tau;
        double rateLimit;   // [1/s]; infinity for throttle
        double minPos, maxPos;
        bool   clampCommand;   // throttle: clamp the command to [0..1] first
    };

    std::vector<Servo> servos_;
};
