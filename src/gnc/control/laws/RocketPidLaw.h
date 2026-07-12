#pragma once

#include "gnc/control/ControlLaw.h"
#include "gnc/control/Pid.h"
#include "io/Json.h"

// Finned-rocket attitude autopilot: three independent PID loops on the fins.
//
//   pitch cmd   -> elevator (PID, q damping)   e.g. a pitch program over time
//   heading cmd -> rudder   (PID, r damping)
//   roll cmd    -> aileron  (PD,  p damping)   default: hold 0 (no spin)
//
// Fins are ineffective at low dynamic pressure, so all loops hold zero output
// below a minimum airspeed (e.g. on the launch rail) to avoid windup.
class RocketPidLaw : public ControlLaw {
public:
    struct Gains {
        double pitchKp = 0.6, pitchKi = 0.15, pitchKd = 0.35;
        double yawKp   = 0.6, yawKi   = 0.15, yawKd   = 0.35;
        double rollKp  = 0.4, rollKd  = 0.15;
        double maxFin  = 0.2618;    // 15 deg
        double minAirspeed = 15.0;  // loops inactive below this [m/s]
        // Above this pitch angle the roll/heading Euler angles degenerate
        // (gimbal region), so those loops fall back to pure rate damping.
        double verticalGuard = 1.2217;   // 70 deg
    };

    explicit RocketPidLaw(const Gains& g);

    // Builder for gnc::Factory: reads the "controller" config block.
    static std::unique_ptr<ControlLaw> fromJson(const json::Value& cfg);

    // Pitch/yaw fins are essential; roll assist and throttle are optional
    // (a vehicle without roll fins or a throttleable motor is legitimate).
    std::vector<ChannelHandle> bindChannels(const ChannelTable& table) override;

    void update(const GncContext& gc, const CommandSet& cmd,
                ChannelValues& out) override;

private:
    Gains g_;
    Pid   pitchPid_;
    Pid   yawPid_;
    ChannelHandle elevator_, aileron_, rudder_, throttle_;
};
