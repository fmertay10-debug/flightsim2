#include "control/ControllerFactory.h"

#include <map>
#include <stdexcept>

#include "control/AircraftController.h"
#include "control/RocketController.h"
#include "control/ScheduledController.h"
#include "control/TvcController.h"

namespace control {

namespace {

std::map<std::string, Factory::Builder>& registry() {
    static std::map<std::string, Factory::Builder> r = {
        { "aircraft", AircraftController::fromJson },
        { "rocket",   RocketController::fromJson   },
    };
    return r;
}

} // namespace

void Factory::registerController(const std::string& type, Builder builder) {
    registry()[type] = std::move(builder);
}

std::unique_ptr<Controller> Factory::create(const std::string& type,
                                            const json::Value& config,
                                            const std::string& baseDir) {
    // Method-driven selection wins over the type default (Lego control block).
    const std::string method = config.str("method", "");
    if (method == "lqr" || method == "scheduled")
        return ScheduledController::fromJson(config, baseDir);
    if (method == "tvc" || method == "tvc_pid")
        return TvcController::fromJson(config);
    if (!method.empty() && method != "pid")
        throw std::invalid_argument("control::Factory: unknown control method '" +
                                    method + "'");

    const auto it = registry().find(type);
    if (it == registry().end())
        throw std::invalid_argument("control::Factory: no controller registered "
                                    "for vehicle type '" + type + "'");
    return it->second(config);
}

} // namespace control
