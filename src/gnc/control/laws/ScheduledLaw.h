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
//   elevator = -(k_alpha*alpha + k_q*q + k_theta*e + k_i*z)
// Yaw mirrors it on [beta, r, heading error] by axisymmetry; roll is a PD
// damper holding wings level. Gains interpolate on Mach from the schedule CSV
//   mach, k_alpha, k_q, k_theta, k_i
//
// OUTPUT CONTRACT (ADR-0004, Option B of
// docs/plans/scheduledlaw-conversion-options.md): the designed per-channel
// deflections u are converted to a WrenchCommand at the boundary
// (moment = B*u with B the components' queried effectiveness) and handed to
// the Allocator, which inverts the product back to ~u when each axis has one
// effector -- true for every vehicle on this law. The designed gains, roll
// qbar attenuation, and anti-windup are untouched. When the offline pipeline
// learns to design in the acceleration domain (Option C), the internals of
// this law get replaced and the configs stay.
// NOTE: with REDUNDANT effectors per axis the B*u -> allocate round trip is
// lossy (min-norm redistributes); pair such airframes with allocated_attitude
// or wait for Option C.
class ScheduledLaw : public ControlLaw {
public:
    struct Config {
        LookupTable1D kAlpha, kQ, kTheta, kI;   // vs Mach
        double maxFin = 0.2618;                 // 15 deg
        double minAirspeed = 20.0;
        double rollKp = 0.4, rollKd = 0.15;     // roll-hold PD (at/below qbarRef)
        double qbarRef = 8000.0;                // attenuate roll gains above this qbar
        double verticalGuard = 1.2217;          // 70 deg: rate-damp near vertical
    };

    explicit ScheduledLaw(Config cfg) : c_(std::move(cfg)) {}

    // Builder for gnc::Factory (method "lqr"/"scheduled"): reads the
    // "schedule" CSV path (relative to baseDir) + optional limits.
    static std::unique_ptr<ControlLaw> fromJson(const json::Value& cfg,
                                                const std::string& baseDir);

    // Pitch/yaw fins are essential; roll assist and throttle are optional.
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
