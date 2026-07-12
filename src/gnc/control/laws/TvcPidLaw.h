#pragma once

#include "gnc/control/ControlLaw.h"
#include "gnc/control/Pid.h"
#include "io/Json.h"

// Thrust-vector-control autopilot for a boost-phase rocket. Same pitch/yaw
// control law as the finned RocketPidLaw, but the output drives the nozzle
// GIMBAL (tvcPitch/tvcYaw) instead of fins -- so it is selected by the control
// "method", independent of the airframe, and paired with a TvcEffector.
//
// Single-nozzle TVC gives pitch + yaw only; roll is left to the airframe's
// aerodynamic roll damping (the demo vehicle is roll-stable). Near vertical the
// heading loop is disabled (ill-conditioned) and yaw rate is simply damped.
//
// TVC authority is proportional to thrust, so the loops go inert after burnout
// by construction -- the effector produces no moment with zero thrust.
class TvcPidLaw : public ControlLaw {
public:
    struct Gains {
        double pitchKp = 3.0, pitchKi = 0.4, pitchKd = 1.2;
        double yawKp   = 3.0, yawKi   = 0.4, yawKd   = 1.2;
        double maxGimbal = 0.1396;       // 8 deg
        double verticalGuard = 1.4835;   // 85 deg
    };

    explicit TvcPidLaw(const Gains& g);

    // Builder for gnc::Factory (method "tvc" / "tvc_pid").
    static std::unique_ptr<ControlLaw> fromJson(const json::Value& cfg);

    // The gimbal channels are essential -- pairing this controller with a
    // vehicle that has no thrust_vectoring block fails at load.
    std::vector<ChannelHandle> bindChannels(const ChannelTable& table) override;

    void update(const GncContext& gc, const CommandSet& cmd,
                ChannelValues& out) override;

private:
    Gains g_;
    Pid   pitchPid_;
    Pid   yawPid_;
    ChannelHandle tvcPitch_, tvcYaw_, throttle_;
};
