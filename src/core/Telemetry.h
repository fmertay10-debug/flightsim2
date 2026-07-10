#pragma once

#include <string>

#include "core/State.h"
#include "core/Channel.h"
#include "core/AirData.h"
#include "gnc/CommandSet.h"

// One entity's full record at one instant -- what every observer consumes.
// Channel values are indexed by the entity's ChannelTable (`channels`, owned
// by the Entity and stable for its lifetime).
struct Telemetry {
    int          id = -1;
    std::string  name;
    State        state;
    ChannelValues control;      // ACTUAL (post-actuator, what the aero sees)
    ChannelValues controlCmd;   // COMMANDED (controller output, pre-actuator)
    const ChannelTable* channels = nullptr;
    CommandSet   setpoint;      // the guidance/flight-plan setpoints being tracked
    AirData      air;
    double       mass   = 0.0;  // [kg]
    double       thrust = 0.0;  // [N]
};
