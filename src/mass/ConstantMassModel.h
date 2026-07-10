#pragma once

#include "mass/MassModel.h"

// Fixed mass properties. Optionally adds a time-varying propellant mass
// supplied by a callback (the legacy "dry mass + solid-motor propellant"
// path), keeping mass draining with the burn while inertia/CG stay fixed.
class ConstantMassModel : public MassModel {
public:
    explicit ConstantMassModel(const MassState& base) : base_(base) {}

    MassState at(double /*time*/) const override { return base_; }

private:
    MassState base_;
};
