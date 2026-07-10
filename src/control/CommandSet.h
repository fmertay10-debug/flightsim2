#pragma once

#include <optional>

// The setpoints a controller tracks at one instant. Produced by the flight
// plan (or, later, a guidance law). Every field is optional: a controller
// only closes the loops for which a command is present.
// Units: radians, meters, m/s; throttle is [0..1].
struct CommandSet {
    std::optional<double> pitch;      // theta command [rad]
    std::optional<double> roll;       // phi command [rad]
    std::optional<double> heading;    // psi command [rad]
    std::optional<double> altitude;   // altitude command [m]
    std::optional<double> speed;      // airspeed command [m/s]
    std::optional<double> throttle;   // direct throttle [0..1]
};
