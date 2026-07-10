#pragma once

#include <memory>
#include <utility>

#include "component/AeroModel.h"
#include "component/ForceComponent.h"

// Adapts an AeroModel to the ForceComponent contract. The aero models keep
// their own richer interface (AeroForces, AeroReference, alpha/beta inputs);
// this wrapper is how they participate in the vehicle's component list.
class AeroComponent : public ForceComponent {
public:
    explicit AeroComponent(std::unique_ptr<AeroModel> model)
        : model_(std::move(model)) {}

    void declareChannels(ChannelTable& table) override {
        model_->declareChannels(table);
    }

    Wrench compute(const ComponentContext& ctx, const ChannelValues& u) override {
        const AeroForces f = model_->compute(ctx.state, ctx.air, u);
        return { f.force, f.moment };
    }

    double momentReferenceStation() const override {
        return model_->momentReferenceStation();
    }

private:
    std::unique_ptr<AeroModel> model_;
};
