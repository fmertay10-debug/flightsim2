#pragma once

#include <memory>

#include "guidance/GuidanceLaw.h"
#include "io/Json.h"

// Factory: guidance law from its scenario config block. The target entity id
// is resolved from the "target" name by the scenario loader and passed in.
//
//   {"type": "pro_nav", "target": "target-1", "nav_gain": 3,
//    "max_pitch_deg": 40, "max_yaw_deg": 60, "blind_range_m": 20}
//   {"type": "pure_pursuit", "target": "target-1"}
namespace guidance {

std::unique_ptr<GuidanceLaw> create(const json::Value& config, int targetId);

} // namespace guidance
