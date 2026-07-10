#pragma once

#include <vector>

#include "control/CommandSet.h"
#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"

// Strategy: flight controller. One concrete class per VEHICLE TYPE
// (AircraftController, RocketController, ...). To support a new vehicle type,
// subclass this and register a builder in control::Factory.
//
// Controllers are stateful (integrators, filters) -- one instance per entity.
class Controller {
public:
    virtual ~Controller() = default;

    // Resolve the channels this controller writes against the vehicle's
    // declared table and keep the handles. Channels the law cannot function
    // without go through table.require() -- it throws with a listing of what
    // IS declared, so a controller paired with a vehicle that lacks the
    // matching consumer fails at LOAD instead of silently flying open-loop.
    // Secondary channels use table.find() (an invalid handle drops the
    // writes). Returns every handle bound (invalid ones included); the loader
    // warns about declared channels no controller drives. Called once at load.
    virtual std::vector<ChannelHandle> bindChannels(const ChannelTable& table) = 0;

    virtual void update(const State& state,
                        const AirData& air,
                        const CommandSet& cmd,
                        double dt,
                        ChannelValues& out) = 0;
};
