#pragma once

#include <functional>
#include <memory>
#include <string>

#include "guidance/GuidanceLaw.h"
#include "io/Json.h"

// Registry for guidance laws, built from a scenario's "guidance" block. The
// target entity id is resolved from the "target" name by the scenario loader
// and passed in.
//
//   {"type": "pro_nav", "target": "target-1", "nav_gain": 3,
//    "max_pitch_deg": 40, "max_yaw_deg": 60, "blind_range_m": 20}
//   {"type": "pure_pursuit", "target": "target-1"}
//
// EXTENSION POINT: Factory::registerLaw("my_law", builder).
namespace guidance {

class Factory {
public:
    using Builder = std::function<std::unique_ptr<GuidanceLaw>(
        const json::Value&, int targetId)>;

    static void registerLaw(const std::string& type, Builder builder);
};

// Reads config["type"]; throws with a listing of registered laws.
std::unique_ptr<GuidanceLaw> create(const json::Value& config, int targetId);

} // namespace guidance
