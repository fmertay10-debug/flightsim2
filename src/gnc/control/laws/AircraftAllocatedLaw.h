#pragma once

#include <memory>
#include <vector>

#include "gnc/control/Allocator.h"
#include "gnc/control/ControlLaw.h"
#include "gnc/control/Pid.h"
#include "io/Json.h"

// Cascaded fixed-wing autopilot with an ALLOCATED inner loop (ADR-0004,
// option A1 of docs/plans/aircraft-conversion-options.md). The outer loops
// are AircraftPidLaw's proven cascades, kept verbatim:
//   heading error -> bank command (clamped)      [lateral]
//   altitude error -> climb-rate cmd -> pitch command (clamped)  [longitudinal]
//   speed error -> throttle (direct channel write, like every converted law)
// The inner stage differs: attitude errors become desired angular
// ACCELERATIONS, then a WrenchCommand (moment = I * alpha_des), which the
// Allocator maps onto whatever surfaces the airframe declares via their live
// effectiveness. No channel sign knowledge lives here -- the F-16's inverted
// aileron, the elevator's nose-down polarity, all come from the columns.
class AircraftAllocatedLaw : public ControlLaw {
public:
    struct Gains {
        // Outer loops -- same config keys and semantics as aircraft_pid.
        double headingKp = 1.0;                      // heading err -> bank cmd
        double altToVs = 0.3;                        // alt err -> climb-rate cmd
        double vsKp = 0.02, vsKi = 0.01;             // climb-rate err -> pitch cmd
        double speedKp = 0.05, speedKi = 0.01;       // speed err -> throttle
        // Inner attitude loop, angular-acceleration domain [rad/s^2 per rad].
        double pitchKp = 60.0, pitchKd = 25.0, pitchKi = 10.0;
        double rollKp  = 120.0, rollKd = 30.0;
        double yawDamp = 4.0;                        // accel per rad/s of r
        // Limits.
        double maxBank      = 0.5236;    // 30 deg
        double maxPitch     = 0.2618;    // 15 deg
        double maxClimbRate = 5.0;       // m/s
        double maxAngAccel  = 1.7453;    // 100 dps^2 * ... (rad/s^2)
        double intLimit     = 1.0;       // pitch integrator clamp [rad*s]
    };

    AircraftAllocatedLaw(const Gains& g, double allocDamping);

    // Builder for gnc::Factory (type "aircraft_allocated").
    static std::unique_ptr<ControlLaw> fromJson(const json::Value& cfg);

    bool allocates() const override { return true; }

    std::vector<ChannelHandle> bindChannels(const ChannelTable& table) override;
    void bindComponents(
        const std::vector<std::unique_ptr<ForceComponent>>& components) override;

    void update(const GncContext& gc, const CommandSet& cmd,
                ChannelValues& out) override;

private:
    Gains g_;
    Pid   vsPid_;      // climb-rate error -> pitch command
    Pid   speedPid_;   // speed error -> throttle
    double ziPitch_ = 0.0;   // inner pitch-attitude integral

    ChannelTable table_;                            // declared limits for allocation
    ChannelHandle throttle_;
    std::vector<const ForceComponent*> components_; // non-owning
    Allocator allocator_;
};
