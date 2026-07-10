#pragma once

#include <functional>
#include <memory>
#include <string>

#include "control/Controller.h"
#include "io/Json.h"

// Registry for control laws -- the Lego "control block". The gnc.control_law
// config names its implementation explicitly via "type":
//
//   "aircraft_pid"       cascaded fixed-wing PID (AircraftController)
//   "rocket_pid"         finned-rocket attitude PID (RocketController)
//   "tvc_pid"            thrust-vector-control PID (TvcController)
//   "scheduled" | "lqr"  gain-scheduled state feedback (ScheduledController,
//                        gains designed by tools/design_autopilot.py)
//
// The law is chosen independently of the airframe -- swap PID for LQR by
// editing one line. EXTENSION POINT: registerControlLaw("my_law", builder).
namespace control {

class Factory {
public:
    using Builder = std::function<std::unique_ptr<Controller>(
        const json::Value&, const std::string&)>;

    static void registerControlLaw(const std::string& type, Builder builder);

    // Reads config["type"]; throws with a listing of registered laws.
    static std::unique_ptr<Controller> create(const json::Value& config,
                                              const std::string& baseDir = ".");
};

} // namespace control
