#pragma once

#include <string>
#include <vector>

#include "core/State.h"
#include "core/Channel.h"
#include "core/AirData.h"
#include "core/Wrench.h"
#include "gnc/CommandSet.h"

// One entity's full record at one instant -- what every observer consumes.
// Channel values are indexed by the entity's ChannelTable (`channels`, owned
// by the Entity and stable for its lifetime); componentLoads by the Vehicle's
// component list (`componentNames`, likewise Entity/Vehicle-owned).
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
    // Per-force-component wrench this step, CG-referenced, in list order --
    // the aero component's entry IS the aero force/moment; a gimbaled motor's
    // entry shows the TVC contribution (the allocation-blend story).
    std::vector<Wrench> componentLoads;
    const std::vector<std::string>* componentNames = nullptr;
};
