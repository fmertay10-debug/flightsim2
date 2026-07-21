#pragma once

#include <memory>
#include <string>

#include "io/Json.h"
#include "vehicle/Vehicle.h"

// Factory: build a Vehicle from a vehicle definition file's top level --
// the "mass" block plus the "components" array (each entry built by
// component::Factory, dispatched on its explicit "type").
//
// Mass block:
//   "mass": {"model": "tabulated", "table": "mass_props.csv"}
// The CSV covers every case: constant mass is a two-row table, a burning
// rocket adds mass/inertia/xcg_m rows over the burn. Columns: time_s,
// mass_kg, ixx, iyy, izz [, ixy, ixz, iyz, xcg_m].
//
// baseDir resolves relative data paths (mass table, engine dir).
namespace vehicle {

std::unique_ptr<Vehicle> create(const json::Value& definition,
                                const std::string& baseDir = ".");

} // namespace vehicle
