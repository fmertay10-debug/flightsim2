#include "guidance/GuidanceFactory.h"

#include <stdexcept>

#include "guidance/ProNav3D.h"
#include "guidance/PurePursuit.h"
#include "math/Units.h"

namespace guidance {

std::unique_ptr<GuidanceLaw> create(const json::Value& config, int targetId) {
    const std::string type = config.str("type");

    if (type == "pro_nav") {
        return std::make_unique<ProNav3D>(
            targetId,
            config.num("nav_gain", 3.0),
            units::deg2rad(config.num("max_pitch_deg", 40.0)),
            units::deg2rad(config.num("max_yaw_deg", 60.0)),
            config.num("blind_range_m", 20.0));
    }
    if (type == "pure_pursuit") {
        return std::make_unique<PurePursuit>(targetId);
    }
    throw std::invalid_argument("guidance::create: unknown type '" + type + "'");
}

} // namespace guidance
