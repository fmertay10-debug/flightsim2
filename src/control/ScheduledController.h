#pragma once

#include <string>

#include "control/Controller.h"
#include "control/Pid.h"
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
class ScheduledController : public Controller {
public:
    struct Config {
        LookupTable1D kAlpha, kQ, kTheta, kI;   // vs Mach
        double maxFin = 0.2618;                 // 15 deg
        double minAirspeed = 20.0;
        double rollKp = 0.4, rollKd = 0.15;     // roll-hold PD (at/below qbarRef)
        double qbarRef = 8000.0;                // attenuate roll gains above this qbar
        double verticalGuard = 1.2217;          // 70 deg: rate-damp near vertical
    };

    explicit ScheduledController(Config cfg) : c_(std::move(cfg)) {}

    // Builder for control::Factory (method "lqr"/"scheduled"): reads the
    // "schedule" CSV path (relative to baseDir) + optional limits.
    static std::unique_ptr<Controller> fromJson(const json::Value& cfg,
                                                const std::string& baseDir);

    ControlInput update(const State& state, const AirData& air,
                        const CommandSet& cmd, double dt) override;

private:
    Config c_;
    double ziTheta_ = 0.0;   // pitch tracking-error integral
    double ziPsi_   = 0.0;   // yaw tracking-error integral
};
