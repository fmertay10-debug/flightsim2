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
        { "aircraft_pid",
          [](const json::Value& cfg, const std::string&) {
              return AircraftController::fromJson(cfg);
          } },
        { "rocket_pid",
          [](const json::Value& cfg, const std::string&) {
              return RocketController::fromJson(cfg);
          } },
        { "tvc_pid",
          [](const json::Value& cfg, const std::string&) {
              return TvcController::fromJson(cfg);
          } },
        { "scheduled",
          [](const json::Value& cfg, const std::string& baseDir) {
              return ScheduledController::fromJson(cfg, baseDir);
          } },
        { "lqr",   // alias: the shipped schedules are LQR designs
          [](const json::Value& cfg, const std::string& baseDir) {
              return ScheduledController::fromJson(cfg, baseDir);
          } },
    };
    return r;
}

} // namespace

void Factory::registerControlLaw(const std::string& type, Builder builder) {
    registry()[type] = std::move(builder);
}

std::unique_ptr<Controller> Factory::create(const json::Value& config,
                                            const std::string& baseDir) {
    const std::string type = config.str("type");
    const auto it = registry().find(type);
    if (it == registry().end()) {
        std::string known;
        for (const auto& [k, v] : registry()) {
            if (!known.empty()) known += ", ";
            known += k;
        }
        throw std::invalid_argument("control::Factory: unknown control law '" +
                                    type + "' (registered: " + known + ")");
    }
    return it->second(config, baseDir);
}

} // namespace control
