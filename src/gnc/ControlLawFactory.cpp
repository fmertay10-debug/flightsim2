#include "gnc/ControlLawFactory.h"

#include <map>
#include <stdexcept>

#include "gnc/AircraftPidLaw.h"
#include "gnc/AllocatedAttitudeLaw.h"
#include "gnc/RocketPidLaw.h"
#include "gnc/ScheduledLaw.h"
#include "gnc/TvcPidLaw.h"

namespace gnc {

namespace {

std::map<std::string, Factory::Builder>& registry() {
    static std::map<std::string, Factory::Builder> r = {
        { "aircraft_pid",
          [](const json::Value& cfg, const std::string&) {
              return AircraftPidLaw::fromJson(cfg);
          } },
        { "rocket_pid",
          [](const json::Value& cfg, const std::string&) {
              return RocketPidLaw::fromJson(cfg);
          } },
        { "tvc_pid",
          [](const json::Value& cfg, const std::string&) {
              return TvcPidLaw::fromJson(cfg);
          } },
        { "scheduled",
          [](const json::Value& cfg, const std::string& baseDir) {
              return ScheduledLaw::fromJson(cfg, baseDir);
          } },
        { "lqr",   // alias: the shipped schedules are LQR designs
          [](const json::Value& cfg, const std::string& baseDir) {
              return ScheduledLaw::fromJson(cfg, baseDir);
          } },
        { "allocated_attitude",
          [](const json::Value& cfg, const std::string&) {
              return AllocatedAttitudeLaw::fromJson(cfg);
          } },
    };
    return r;
}

} // namespace

void Factory::registerControlLaw(const std::string& type, Builder builder) {
    registry()[type] = std::move(builder);
}

std::unique_ptr<ControlLaw> Factory::create(const json::Value& config,
                                            const std::string& baseDir) {
    const std::string type = config.str("type");
    const auto it = registry().find(type);
    if (it == registry().end()) {
        std::string known;
        for (const auto& [k, v] : registry()) {
            if (!known.empty()) known += ", ";
            known += k;
        }
        throw std::invalid_argument("gnc::Factory: unknown control law '" +
                                    type + "' (registered: " + known + ")");
    }
    return it->second(config, baseDir);
}

} // namespace gnc
