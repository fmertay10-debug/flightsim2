#pragma once

#include <memory>
#include <string>

#include "io/Json.h"
#include "propulsion/PropulsionModel.h"

// Factory: propulsion strategy from its JSON config block. baseDir resolves
// relative data-file paths (thrust tables, F-16 engine dir).
//   {"type": "none"}
//   {"type": "turbojet", "max_thrust_n": 2200, "density_lapse": 0.7}
//   {"type": "solid_motor", "propellant_kg": 25,
//    "thrust_curve": [[0, 3000], [5.5, 3000], [6, 0]], "ignition_time_s": 0}
//   {"type": "tabulated_thrust", "table": "thrust.csv", "throttle_gated": false}
//   {"type": "f16_engine", "dir": "."}
namespace propulsion {

std::unique_ptr<PropulsionModel> create(const json::Value& config,
                                        const std::string& baseDir = ".");

} // namespace propulsion
