#include "guidance/GuidanceFactory.h"

#include <map>
#include <stdexcept>

#include "guidance/ProNav3D.h"
#include "guidance/PurePursuit.h"
#include "math/Units.h"

namespace guidance {

namespace {

std::map<std::string, Factory::Builder>& registry() {
    static std::map<std::string, Factory::Builder> r = {
        { "pro_nav",
          [](const json::Value& config, int targetId) {
              return std::make_unique<ProNav3D>(
                  targetId,
                  config.num("nav_gain", 3.0),
                  units::deg2rad(config.num("max_pitch_deg", 40.0)),
                  units::deg2rad(config.num("max_yaw_deg", 60.0)),
                  config.num("blind_range_m", 20.0));
          } },
        { "pure_pursuit",
          [](const json::Value&, int targetId) {
              return std::make_unique<PurePursuit>(targetId);
          } },
    };
    return r;
}

} // namespace

void Factory::registerLaw(const std::string& type, Builder builder) {
    registry()[type] = std::move(builder);
}

std::unique_ptr<GuidanceLaw> create(const json::Value& config, int targetId) {
    const std::string type = config.str("type");
    const auto it = registry().find(type);
    if (it == registry().end()) {
        std::string known;
        for (const auto& [k, v] : registry()) {
            if (!known.empty()) known += ", ";
            known += k;
        }
        throw std::invalid_argument("guidance::create: unknown guidance law '" +
                                    type + "' (registered: " + known + ")");
    }
    return it->second(config, targetId);
}

} // namespace guidance
