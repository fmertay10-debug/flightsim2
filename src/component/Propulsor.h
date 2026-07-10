#pragma once

#include <memory>
#include <optional>

#include "component/ForceComponent.h"
#include "propulsion/PropulsionModel.h"

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
// proportional to thrust -- zero after burnout, by construction.
class Propulsor : public ForceComponent {
public:
    struct Gimbal {
        double nozzleStation = 0.0;   // [m, nose datum, aft positive]
        double maxGimbal     = 0.0;   // [rad]
    };

    Propulsor(std::unique_ptr<PropulsionModel> model, std::optional<Gimbal> gimbal)
        : model_(std::move(model)), gimbal_(gimbal) {}

    void declareChannels(ChannelTable& table) override;

    Wrench compute(const ComponentContext& ctx, const ChannelValues& u) override;

    double thrustNewtons() const override { return lastThrust_; }
    double propellantMass(double time) const override {
        return model_->propellantMass(time);
    }

    // Gimbal moment sensitivities: dMy/d(tvc_pitch) = dMz/d(tvc_yaw) = L*T,
    // using the LAST computed thrust (one-step lag; deterministic, and the
    // first step's zero simply allocates nothing to the gimbal).
    int controlEffectiveness(const ComponentContext& ctx,
                             ControlEffect* out, int maxOut) const override;

private:
    std::unique_ptr<PropulsionModel> model_;
    std::optional<Gimbal> gimbal_;
    ChannelHandle throttle_, tvcPitch_, tvcYaw_;
    double lastThrust_ = 0.0;
};
