#pragma once

#include <optional>

// The vocabulary level a Command speaks. Attitude-level commands (pitch/
// heading/altitude holds) are simple and human-scriptable; acceleration-level
// commands (missile vocabulary) are nearly vehicle-agnostic. Guidance laws
// declare the level they emit (GuidanceLaw::emits) and control laws declare
// what they accept (ControlLaw::accepts); the pairing is validated when
// guidance is attached to an entity.
enum class CommandLevel {
    Attitude,       // pitch / roll / heading / altitude / speed
    Acceleration,   // body-frame lateral acceleration demands
};

// The setpoints a control law tracks at one instant. Produced by the flight
// plan and/or a guidance law overlay. Every field is optional: a law only
// closes the loops for which a command is present.
// Units: radians, meters, m/s, m/s^2; throttle is [0..1].
struct CommandSet {
    // Attitude-level
    std::optional<double> pitch;      // theta command [rad]
    std::optional<double> roll;       // phi command [rad]
    std::optional<double> heading;    // psi command [rad]
    std::optional<double> altitude;   // altitude command [m]
    std::optional<double> speed;      // airspeed command [m/s]
    std::optional<double> throttle;   // direct throttle [0..1]
    // Acceleration-level (consumed only by laws that accept it; no built-in
    // guidance emits these yet -- the vocabulary seam for accel-native ProNav)
    std::optional<double> accelUp;    // body +up (-z) accel demand [m/s^2]
    std::optional<double> accelRight; // body +right (+y) accel demand [m/s^2]
};
