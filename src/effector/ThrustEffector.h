#pragma once

#include "effector/Effector.h"

// Plain axial thrust: the engine's thrust acts along body +x through the CG,
// so it is pure force with no moment. This is the default thrust application
// for a fin-controlled or unpowered vehicle -- it replaces the inline thrust
// term the Entity used to add directly. Consumes no channels.
class ThrustEffector : public Effector {
public:
    Wrench compute(const EffectorContext& ctx, const ChannelValues&) const override {
        return { Vector3(ctx.thrust, 0.0, 0.0), Vector3() };
    }
};
