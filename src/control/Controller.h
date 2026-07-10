#pragma once

#include "control/CommandSet.h"
#include "core/AirData.h"
#include "core/ControlInput.h"
#include "core/State.h"

// Strategy: flight controller. One concrete class per VEHICLE TYPE
// (AircraftController, RocketController, ...). To support a new vehicle type,
// subclass this and register a builder in control::Factory.
//
// Controllers are stateful (integrators, filters) -- one instance per entity.
class Controller {
public:
    virtual ~Controller() = default;

    virtual ControlInput update(const State& state,
                                const AirData& air,
                                const CommandSet& cmd,
                                double dt) = 0;
};
