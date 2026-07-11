#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "component/ForceComponent.h"
#include "core/Channel.h"
#include "mass/MassModel.h"

// The complete physical craft (what a vehicles/*.json defines): mass
// properties + a flat list of ForceComponents (aero, motors, ...) + the
// channels those components declare. Control/guidance are NOT part of the
// Vehicle -- the GNC stack attaches to the Entity.
class Vehicle {
public:
    // componentNames: one label per component (its config "type", deduped by
    // the factory) -- used for telemetry column names; may be empty.
    Vehicle(std::unique_ptr<MassModel> mass,
            std::vector<std::unique_ptr<ForceComponent>> components,
            bool addMotorPropellant = false,
            std::vector<std::string> componentNames = {})
        : mass_(std::move(mass)),
          components_(std::move(components)),
          componentNames_(std::move(componentNames)),
          addMotorPropellant_(addMotorPropellant) {}

    // Mass, inertia, CG at sim time. The legacy path (scalar dry mass + solid
    // motor) adds the motors' remaining propellant to the constant dry mass.
    MassState massState(double time) const {
        MassState s = mass_->at(time);
        if (addMotorPropellant_)
            for (const auto& c : components_) s.mass += c->propellantMass(time);
        return s;
    }

    std::vector<std::unique_ptr<ForceComponent>>& components() { return components_; }
    const std::vector<std::string>& componentNames() const { return componentNames_; }

    // Forward declaration phase to every component, in list order.
    void declareChannels(ChannelTable& table) {
        for (const auto& c : components_) c->declareChannels(table);
    }

    // Telemetry: total propulsive thrust produced by the last compute pass [N].
    double thrustNewtons() const {
        double t = 0.0;
        for (const auto& c : components_) t += c->thrustNewtons();
        return t;
    }

private:
    std::unique_ptr<MassModel> mass_;
    std::vector<std::unique_ptr<ForceComponent>> components_;
    std::vector<std::string> componentNames_;
    bool addMotorPropellant_;
};
