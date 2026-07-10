#include "propulsion/PropulsionFactory.h"

#include <filesystem>
#include <stdexcept>
#include <utility>
#include <vector>

#include "propulsion/F16Engine.h"
#include "propulsion/SolidMotor.h"
#include "propulsion/TabulatedThrust.h"
#include "propulsion/Turbojet.h"

namespace propulsion {

std::unique_ptr<PropulsionModel> create(const json::Value& config,
                                        const std::string& baseDir) {
    const std::string type = config.str("type");

    const auto resolve = [&](const std::string& p) {
        const std::filesystem::path fp(p);
        return fp.is_absolute() ? p : (std::filesystem::path(baseDir) / fp).string();
    };

    if (type == "none") {
        return std::make_unique<NoPropulsion>();
    }
    if (type == "turbojet") {
        return std::make_unique<Turbojet>(config.num("max_thrust_n"),
                                          config.num("density_lapse", 0.7));
    }
    if (type == "solid_motor") {
        const json::Value& pts = config.at("thrust_curve");
        std::vector<std::pair<double, double>> curve;
        curve.reserve(pts.size());
        for (std::size_t i = 0; i < pts.size(); ++i)
            curve.emplace_back(pts[i][0].asNumber(), pts[i][1].asNumber());
        return std::make_unique<SolidMotor>(std::move(curve),
                                            config.num("propellant_kg"),
                                            config.num("ignition_time_s", 0.0));
    }
    if (type == "tabulated_thrust") {
        return std::make_unique<TabulatedThrust>(
            TabulatedThrust::fromCsv(resolve(config.str("table")),
                                     config.str("time_col", "time_s"),
                                     config.str("thrust_col", "thrust_n"),
                                     config.boolean("throttle_gated", false)));
    }
    if (type == "f16_engine") {
        return std::make_unique<F16Engine>(
            F16Engine::fromCsv(resolve(config.str("dir"))));
    }
    throw std::invalid_argument("propulsion::create: unknown type '" + type + "'");
}

} // namespace propulsion
