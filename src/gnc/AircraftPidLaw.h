#pragma once

#include "gnc/ControlLaw.h"
#include "gnc/Pid.h"
#include "io/Json.h"

// Fixed-wing autopilot: cascaded PID loops.
//
//   heading cmd  -> bank cmd      (P, clamped to maxBank)
//   altitude cmd -> climb-rate cmd -> pitch cmd (PI, clamped to maxPitch)
//   pitch cmd    -> elevator      (PID, rate damping on q)
//   roll cmd     -> aileron       (PD, rate damping on p)
//   speed cmd    -> throttle      (PI)
//   yaw damper   -> rudder        (r feedback)
//
// Direct pitch/roll/throttle commands bypass their outer loops.
class AircraftPidLaw : public ControlLaw {
public:
    struct Gains {
        // Inner attitude loops
        double pitchKp = 2.0, pitchKi = 0.4, pitchKd = 0.8;
        double rollKp  = 2.0, rollKd  = 0.5;
        // Outer loops
        double headingKp = 1.5;                    // psi err -> bank cmd
        double altToVs   = 0.3;                    // alt err -> climb-rate cmd [1/s]
        double vsKp = 0.02, vsKi = 0.01;           // climb-rate err -> pitch cmd
        double speedKp = 0.05, speedKi = 0.01;     // speed err -> throttle
        double yawDamper = 0.5;                    // r -> rudder
        // Limits
        double maxBank      = 0.5236;   // 30 deg
        double maxPitch     = 0.2618;   // 15 deg
        double maxClimbRate = 5.0;      // [m/s]
        double maxElevator  = 0.4363;   // 25 deg
        double maxAileron   = 0.3491;   // 20 deg
        double maxRudder    = 0.3491;   // 20 deg
    };

    explicit AircraftPidLaw(const Gains& g);

    // Builder for gnc::Factory: reads the "controller" config block.
    static std::unique_ptr<ControlLaw> fromJson(const json::Value& cfg);

    // All three surfaces are essential on a fixed-wing aircraft.
    std::vector<ChannelHandle> bindChannels(const ChannelTable& table) override;

    void update(const GncContext& gc, const CommandSet& cmd,
                ChannelValues& out) override;

private:
    Gains g_;
    ChannelHandle elevator_, aileron_, rudder_, throttle_;
    Pid   pitchPid_;   // pitch err -> +up demand (sign flipped onto elevator)
    Pid   vsPid_;      // climb-rate err -> pitch cmd
    Pid   speedPid_;   // speed err -> throttle
};
