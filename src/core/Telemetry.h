#pragma once

#include <string>

#include "core/State.h"
#include "core/ControlInput.h"
#include "core/AirData.h"
#include "control/CommandSet.h"

// One entity's full record at one instant -- what every observer consumes.
struct Telemetry {
    int          id = -1;
    std::string  name;
    State        state;
    ControlInput control;      // ACTUAL (post-actuator, what the aero sees)
    ControlInput controlCmd;   // COMMANDED (controller output, pre-actuator)
    CommandSet   setpoint;     // the guidance/flight-plan setpoints being tracked
    AirData      air;
    double       mass   = 0.0; // [kg]
    double       thrust = 0.0; // [N]
};
