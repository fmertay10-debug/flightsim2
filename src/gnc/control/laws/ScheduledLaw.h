#pragma once

#include <string>

#include <memory>
#include <vector>

#include "gnc/control/Allocator.h"
#include "gnc/control/ControlLaw.h"
#include "gnc/control/Pid.h"
#include "math/LookupTable1D.h"
#include "io/Json.h"

// Gain-scheduled state-feedback autopilot for a symmetric rocket/missile. The
// pitch (and mirrored yaw) gains come from tools/design_autopilot.py -- an LQR
// / pole-placement design against the vehicle's OWN aero + mass, scheduled on
// Mach. The control block reads the gains the aero/mass blocks produced.
//
// Pitch law (state feedback on [alpha, q, e, integral(e)], e = theta-theta_cmd):
//   a_pitch = -(k_alpha_acc*alpha + k_q_acc*q + k_theta_acc*e + k_i_acc*z)
// Yaw mirrors it on [beta, r, heading error] by axisymmetry; roll is a PD
// damper holding wings level. Gains interpolate on Mach from the schedule CSV
//   mach, k_alpha_acc, k_q_acc, k_theta_acc, k_i_acc
//
// OUTPUT CONTRACT (ADR-0004 Option C1, superseding the Option-B fin-angle
// bridge -- see docs/plans/scheduledlaw-conversion-options.md): the scheduled
// gains are in the ANGULAR-ACCELERATION domain (rad/s^2 per unit of state);
// the law emits WrenchCommand.moment = I * a_des and the Allocator divides by
// the components' LIVE effectiveness. Consequences: the loop self-adjusts as
// qbar leaves the design condition, redundant effectors share properly, and
// the old roll qbarRef attenuation is unnecessary (a fixed accel demand over
// qbar-growing effectiveness attenuates the deflection by construction).
// Anti-windup: the integrators freeze while the accel demand is clamped.
class ScheduledLaw : public ControlLaw {
public:
    struct Config {
        LookupTable1D kAlpha, kQ, kTheta, kI;   // accel-domain gains vs Mach
        double maxAngAccel = 698.13;            // accel-demand clamp [rad/s^2] (40000 dps^2)
        double minAirspeed = 20.0;
        double rollKp = 40.0, rollKd = 15.0;    // roll-hold PD [rad/s^2 per rad, per rad/s]
        double verticalGuard = 1.2217;          // 70 deg: rate-damp near vertical
    };

    explicit ScheduledLaw(Config cfg) : c_(std::move(cfg)) {}

    // Builder for gnc::Factory (method "lqr"/"scheduled"): reads the
    // "schedule" CSV path (relative to baseDir) + optional limits.
    static std::unique_ptr<ControlLaw> fromJson(const json::Value& cfg,
                                                const std::string& baseDir);

    // Pitch/yaw fins are essential; roll assist and throttle are optional.
    bool allocates() const override { return true; }

    std::vector<ChannelHandle> bindChannels(const ChannelTable& table) override;

    // Keeps component references to query control effectiveness (the B whose
    // columns turn the designed deflections into the emitted WrenchCommand).
    void bindComponents(
        const std::vector<std::unique_ptr<ForceComponent>>& components) override;

    void update(const GncContext& gc, const CommandSet& cmd,
                ChannelValues& out) override;

private:
    Config c_;
    double ziTheta_ = 0.0;   // pitch tracking-error integral
    double ziPsi_   = 0.0;   // yaw tracking-error integral
    ChannelHandle elevator_, aileron_, rudder_, throttle_;
    ChannelTable table_;                            // declared limits for allocation
    std::vector<const ForceComponent*> components_; // non-owning (Vehicle outlives law)
    Allocator allocator_;
};
