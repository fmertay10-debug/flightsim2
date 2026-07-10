#pragma once

#include <vector>

#include "gnc/Allocator.h"
#include "gnc/ControlLaw.h"
#include "io/Json.h"

// Allocation-based attitude autopilot (ADR-0002's standard path): a PID on
// attitude errors produces desired angular ACCELERATIONS, scaled by the
// current inertia into pseudo-controls (desired body moments), which the
// Allocator distributes over whatever control channels the vehicle's
// components declare -- fins, TVC gimbal, later RCS -- by their queried
// effectiveness at the current flight condition.
//
// This is what makes a hybrid vehicle (TVC + fins) fly with ONE law and no
// mode switching: on the pad the gimbal has all the authority, as qbar builds
// the fins take over, after burnout the gimbal column dies. The law never
// mentions either by name, and it carries NO plant sign knowledge -- signs
// live in the effectiveness columns the components report.
class AllocatedAttitudeLaw : public ControlLaw {
public:
    struct Gains {
        // attitude error -> desired angular acceleration [1/s^2 per rad]
        double pitchKp = 4.0, pitchKd = 3.0, pitchKi = 0.0;
        double yawKp   = 4.0, yawKd   = 3.0, yawKi   = 0.0;
        double rollKp  = 2.0, rollKd  = 1.0;
        double maxAngAccel   = 0.35;     // clamp on demanded accel [rad/s^2]
        double intLimit      = 0.5;      // clamp on error integrals [rad s]
        double verticalGuard = 1.4661;   // 84 deg: rate-damp roll/yaw near vertical
    };

    AllocatedAttitudeLaw(const Gains& g, double allocDamping)
        : g_(g), allocator_(allocDamping) {}

    // Builder for gnc::Factory (type "allocated_attitude").
    static std::unique_ptr<ControlLaw> fromJson(const json::Value& cfg);

    std::vector<ChannelHandle> bindChannels(const ChannelTable& table) override;
    void bindComponents(
        const std::vector<std::unique_ptr<ForceComponent>>& components) override;

    void update(const GncContext& gc, const CommandSet& cmd,
                ChannelValues& out) override;

private:
    Gains g_;
    Allocator allocator_;
    std::vector<const ForceComponent*> components_;
    ChannelTable table_;   // copy for allocation-time channel limits
    ChannelHandle throttle_;
    double ziPitch_ = 0.0, ziYaw_ = 0.0;   // tracking-error integrals
};
