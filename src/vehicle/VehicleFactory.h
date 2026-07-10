#pragma once

#include <memory>
#include <string>

#include "io/Json.h"
#include "vehicle/Vehicle.h"

// Factory: build a Vehicle from a vehicle definition file's top level.
//
// Mass block (Lego): either an explicit "mass" object
//   "mass": {"model": "constant", "mass_kg": 9300,
//            "inertia": {"ixx":.., "iyy":.., "izz":.., "ixz":..}, "xcg_m": 1.2}
//   "mass": {"model": "tabulated", "table": "mass_props.csv", "xcg_m": 0}
// or the legacy flat form ("mass_kg" + "inertia") -- in which case a solid
// motor's propellant is added to the (dry) mass automatically.
//
// baseDir resolves relative data paths (mass table, engine dir).
namespace vehicle {

std::unique_ptr<Vehicle> create(const json::Value& definition,
                                const std::string& baseDir = ".");

} // namespace vehicle
