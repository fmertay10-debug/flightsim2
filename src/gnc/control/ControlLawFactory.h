#pragma once

#include <functional>
#include <memory>
#include <string>

#include "gnc/control/ControlLaw.h"
#include "io/Json.h"

// Registry for control laws -- the Lego "control block". The gnc.control_law
// config names its implementation explicitly via "type". Every law emits a
// WrenchCommand through the Allocator (ADR-0004; the direct-write PIDs were
// retired 2026-07-19 once all vehicles converted):
//
//   "allocated_attitude"  attitude tracking for rockets/missiles
//                         (AllocatedAttitudeLaw)
//   "aircraft_allocated"  fixed-wing cascades (altitude/heading/speed outer
//                         loops) over the same inner loop (AircraftAllocatedLaw)
//   "scheduled" | "lqr"   gain-scheduled state feedback (ScheduledLaw, gains
//                         designed by tools/design_autopilot.py)
//
// The law is chosen independently of the airframe -- swap hand gains for an
// LQR schedule by editing one line. EXTENSION POINT:
// registerControlLaw("my_law", builder).
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
