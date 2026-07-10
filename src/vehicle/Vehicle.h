#pragma once

#include <memory>
#include <string>
#include <utility>

#include "mass/MassModel.h"
#include "propulsion/PropulsionModel.h"

// A vehicle's physical plant: a mass block + a propulsion block. Aerodynamics
// live in the AeroModel and control in the Controller -- this class is purely
// the mass/inertia/CG and thrust provider, composed from two swappable Lego
// blocks.
//
// mass properties are time-varying (burn-time mass/inertia/CG travel via the
// MassModel). Thrust acts along body +x through the CG (force only).
class Vehicle {
public:
    Vehicle(std::string typeName,
            std::unique_ptr<MassModel>       mass,
            std::unique_ptr<PropulsionModel> propulsion,
            bool addMotorPropellant = false)
        : typeName_(std::move(typeName)),
          mass_(std::move(mass)),
          propulsion_(std::move(propulsion) ? std::move(propulsion)
                                            : nullptr),
          addMotorPropellant_(addMotorPropellant)
    {
        if (!propulsion_) propulsion_ = std::make_unique<NoPropulsion>();
    }

    // Mass, inertia, CG at sim time. The legacy path (scalar dry mass + solid
    // motor) adds the motor's remaining propellant to the constant dry mass.
    MassState massState(double time) const {
        MassState s = mass_->at(time);
        if (addMotorPropellant_) s.mass += propulsion_->propellantMass(time);
        return s;
    }

    // Thrust [N] this step (non-const: throttleable engines advance internal
    // spool state -- call once per step).
    double thrust(const PropulsionContext& ctx) { return propulsion_->thrust(ctx); }

    const std::string& typeName() const { return typeName_; }

private:
    std::string typeName_;
    std::unique_ptr<MassModel>       mass_;
    std::unique_ptr<PropulsionModel> propulsion_;
    bool addMotorPropellant_;
};
