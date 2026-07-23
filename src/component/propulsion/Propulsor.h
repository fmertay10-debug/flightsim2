#pragma once

#include <memory>
#include <optional>

#include "component/ForceComponent.h"
#include "component/propulsion/PropulsionModel.h"

// A motor and its nozzle as ONE force component: the PropulsionModel supplies
// the thrust magnitude, the mount turns it into a wrench. Axial mount (the
// default) is pure +x force through the CG; a gimbaled mount ("gimbal" block
// in the component config) deflects the thrust vector for pitch/yaw control
// moments -- what used to be the ThrustEffector/TvcEffector pair.
//
// Channels: declares "throttle" (the propulsion demand -- motors that burn a
// fixed curve still own it so the command stays visible in telemetry), plus
// "tvc_pitch"/"tvc_yaw" when gimbaled (see core/Channel.h for signs).
//
// Gimbal physics: the moment arm is (nozzle station - CG station), so it GROWS
// as the CG migrates forward during the burn, and control authority is
// proportional to thrust -- zero after burnout, by construction. The arm
// cannot be formed without a CG, so a gimbaled propulsor requires a mass
// table with an xcg_m column (same nose datum, aft positive, as
// nozzle_station_m) -- enforced at load via requiresCgStation().
class Propulsor : public ForceComponent {
public:
    struct Gimbal {
        double nozzleStation = 0.0;   // [m, nose datum, aft positive]
        double maxGimbal     = 0.0;   // [rad]
    };

    Propulsor(std::unique_ptr<PropulsionModel> model, std::optional<Gimbal> gimbal)
        : model_(std::move(model)), gimbal_(gimbal) {}

    void declareChannels(ChannelTable& table) override;

    // Externalized state (ADR-0003): forward to the wrapped model, which owns
    // any spool/internal state. computeWrench() is the pure hot-path entry.
    int    numStates() const override { return model_->numStates(); }
    void   initializeState(double* x) const override { model_->initializeState(x); }
    void   derivatives(const ComponentContext& ctx, const ChannelValues& u,
                       const double* x, double* xdot) const override;
    Wrench computeWrench(const ComponentContext& ctx, const ChannelValues& u,
                         const double* x) const override;

    double thrustNewtons() const override { return lastThrust_; }

    // The gimbal moment arm is (nozzleStation - xcg): meaningless without a
    // CG station. Axial mounts thrust through the CG and need none.
    bool requiresCgStation() const override { return gimbal_.has_value(); }

    // Gimbal moment sensitivities: dMy/d(tvc_pitch) = dMz/d(tvc_yaw) = L*T,
    // using the LAST computed thrust (one-step lag; deterministic, and the
    // first step's zero simply allocates nothing to the gimbal).
    int controlEffectiveness(const ComponentContext& ctx,
                             ControlEffect* out, int maxOut) const override;

private:
    // Assemble the PropulsionContext this component feeds its model.
    PropulsionContext makeContext(const ComponentContext& ctx,
                                  const ChannelValues& u) const;
    // Turn a thrust magnitude into the body wrench (axial or gimbaled) and
    // cache it for telemetry/effectiveness.
    Wrench wrenchFromThrust(double thrust, const ComponentContext& ctx,
                            const ChannelValues& u) const;

    std::unique_ptr<PropulsionModel> model_;
    std::optional<Gimbal> gimbal_;
    ChannelHandle throttle_, tvcPitch_, tvcYaw_;
    mutable double lastThrust_ = 0.0;   // telemetry/effectiveness cache (mutable:
                                        // set by the const pure computeWrench)
};
