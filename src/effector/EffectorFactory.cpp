#include "effector/EffectorFactory.h"

#include "effector/ThrustEffector.h"
#include "effector/TvcEffector.h"
#include "math/Units.h"

namespace effector {

std::vector<std::unique_ptr<Effector>> build(const json::Value& def) {
    std::vector<std::unique_ptr<Effector>> effectors;

    // Thrust application: gimbaled (TVC) if configured, else axial.
    if (def.has("thrust_vectoring")) {
        const json::Value& tv = def.at("thrust_vectoring");
        effectors.push_back(std::make_unique<TvcEffector>(
            tv.num("nozzle_station_m"),
            units::deg2rad(tv.num("max_gimbal_deg", 8.0))));
    } else {
        effectors.push_back(std::make_unique<ThrustEffector>());
    }

    return effectors;
}

} // namespace effector
