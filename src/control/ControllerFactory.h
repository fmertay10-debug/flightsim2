#pragma once

#include <functional>
#include <memory>
#include <string>

#include "control/Controller.h"
#include "io/Json.h"

// Factory for controllers -- the Lego "control block". It picks a controller
// TWO ways:
//   1. If the config names a control "method" ("pid" | "lqr"/"scheduled"), the
//      method decides -- so the algorithm is chosen independently of the
//      airframe. "lqr"/"scheduled" -> ScheduledController (reads a gain
//      schedule designed by tools/design_autopilot.py from the vehicle's aero).
//   2. Otherwise it falls back to the per-vehicle-TYPE default
//      ("aircraft" -> AircraftController, "rocket" -> RocketController).
//
// EXTENSION POINT: register a new type default with registerController, or add
// a new method branch.
namespace control {

class Factory {
public:
    using Builder = std::function<std::unique_ptr<Controller>(const json::Value&)>;

    static void registerController(const std::string& type, Builder builder);

    static std::unique_ptr<Controller> create(const std::string& type,
                                              const json::Value& config,
                                              const std::string& baseDir = ".");
};

} // namespace control
