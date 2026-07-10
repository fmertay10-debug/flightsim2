#pragma once

#include <functional>
#include <memory>
#include <string>

#include "gnc/ControlLaw.h"
#include "io/Json.h"

// Registry for control laws -- the Lego "control block". The gnc.control_law
// config names its implementation explicitly via "type":
//
//   "aircraft_pid"       cascaded fixed-wing PID (AircraftPidLaw)
//   "rocket_pid"         finned-rocket attitude PID (RocketPidLaw)
//   "tvc_pid"            thrust-vector-control PID (TvcPidLaw)
//   "scheduled" | "lqr"  gain-scheduled state feedback (ScheduledLaw,
//                        gains designed by tools/design_autopilot.py)
//
// The law is chosen independently of the airframe -- swap PID for LQR by
// editing one line. EXTENSION POINT: registerControlLaw("my_law", builder).
namespace gnc {

class Factory {
public:
    using Builder = std::function<std::unique_ptr<ControlLaw>(
        const json::Value&, const std::string&)>;

    static void registerControlLaw(const std::string& type, Builder builder);

    // Reads config["type"]; throws with a listing of registered laws.
    static std::unique_ptr<ControlLaw> create(const json::Value& config,
                                              const std::string& baseDir = ".");
};

} // namespace gnc
