#pragma once

#include <memory>
#include <vector>

#include "effector/Effector.h"
#include "io/Json.h"

// Builds a vehicle's control-effector list from its definition. Every powered
// vehicle gets exactly one THRUST-applying effector -- axial by default, or a
// TvcEffector when the definition carries a "thrust_vectoring" block:
//
//   "thrust_vectoring": { "nozzle_station_m": 2.7, "max_gimbal_deg": 8 }
//
// EXTENSION POINT: additive effectors (RCS, etc.) would be appended here from
// their own config blocks; the Entity sums whatever the list contains.
namespace effector {

std::vector<std::unique_ptr<Effector>> build(const json::Value& definition);

} // namespace effector
