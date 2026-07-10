#include "aero/AeroFactory.h"

#include <map>
#include <stdexcept>

#include "aero/AircraftAero.h"
#include "aero/F16Aero.h"
#include "aero/RocketAero.h"
#include "aero/RocketTableAero.h"

namespace aero {

namespace {

std::map<std::string, Factory::Builder>& registry() {
    static std::map<std::string, Factory::Builder> r = {
        // "aircraft" dispatches: F-16-style CSV tables when a "dir" is given,
        // point derivatives otherwise.
        { "aircraft",
          [](const json::Value& cfg, const std::string& baseDir) {
              return cfg.has("dir") ? F16Aero::fromJson(cfg, baseDir)
                                    : AircraftAero::fromJson(cfg);
          } },
        // "rocket" dispatches: DATCOM lookup tables when tables_csv is given,
        // point derivatives otherwise.
        { "rocket",
          [](const json::Value& cfg, const std::string& baseDir) {
              return cfg.has("tables_csv") ? RocketTableAero::fromJson(cfg, baseDir)
                                           : RocketAero::fromJson(cfg);
          } },
    };
    return r;
}

} // namespace

void Factory::registerModel(const std::string& type, Builder builder) {
    registry()[type] = std::move(builder);
}

std::unique_ptr<AeroModel> Factory::create(const std::string& type,
                                           const json::Value& config,
                                           const std::string& baseDir) {
    const auto it = registry().find(type);
    if (it == registry().end())
        throw std::invalid_argument("aero::Factory: no aero model registered for "
                                    "vehicle type '" + type + "'");
    return it->second(config, baseDir);
}

} // namespace aero
