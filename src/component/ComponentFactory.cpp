#include "component/ComponentFactory.h"

#include <filesystem>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "models/aircraft/AircraftAero.h"
#include "models/f16/F16Aero.h"
#include "models/rocket/RocketTableAero.h"
#include "component/Propulsor.h"
#include "math/Units.h"
#include "models/f16/F16Engine.h"
#include "propulsion/SolidMotor.h"
#include "propulsion/TabulatedThrust.h"
#include "propulsion/Turbojet.h"

namespace component {

namespace {

std::string resolve(const std::string& p, const std::string& baseDir) {
    const std::filesystem::path fp(p);
    return fp.is_absolute() ? p : (std::filesystem::path(baseDir) / fp).string();
}

// Optional thrust-vectoring mount on any motor component.
std::optional<Propulsor::Gimbal> gimbalFromJson(const json::Value& cfg) {
    if (!cfg.has("gimbal")) return std::nullopt;
    const json::Value& g = cfg.at("gimbal");
    return Propulsor::Gimbal{ g.num("nozzle_station_m"),
                              units::deg2rad(g.num("max_gimbal_deg", 8.0)) };
}

std::unique_ptr<ForceComponent> propulsor(std::unique_ptr<PropulsionModel> model,
                                          const json::Value& cfg) {
    return std::make_unique<Propulsor>(std::move(model), gimbalFromJson(cfg));
}

std::map<std::string, Factory::Builder>& registry() {
    static std::map<std::string, Factory::Builder> r = {
        // --- Aerodynamics (plain ForceComponents; no wrapper) ---
        { "aircraft_aero",
          [](const json::Value& cfg, const std::string&) {
              return AircraftAero::fromJson(cfg);
          } },
        { "f16_aero",
          [](const json::Value& cfg, const std::string& baseDir) {
              return F16Aero::fromJson(cfg, baseDir);
          } },
        { "rocket_table_aero",
          [](const json::Value& cfg, const std::string& baseDir) {
              return RocketTableAero::fromJson(cfg, baseDir);
          } },
        // --- Motors (optional "gimbal" block = thrust-vectoring mount) ---
        { "turbojet",
          [](const json::Value& cfg, const std::string&) {
              return propulsor(std::make_unique<Turbojet>(
                                   cfg.num("max_thrust_n"),
                                   cfg.num("density_lapse", 0.7)),
                               cfg);
          } },
        { "solid_motor",
          [](const json::Value& cfg, const std::string&) {
              const json::Value& pts = cfg.at("thrust_curve");
              std::vector<std::pair<double, double>> curve;
              curve.reserve(pts.size());
              for (std::size_t i = 0; i < pts.size(); ++i)
                  curve.emplace_back(pts[i][0].asNumber(), pts[i][1].asNumber());
              return propulsor(std::make_unique<SolidMotor>(
                                   std::move(curve),
                                   cfg.num("propellant_kg"),
                                   cfg.num("ignition_time_s", 0.0)),
                               cfg);
          } },
        { "tabulated_thrust",
          [](const json::Value& cfg, const std::string& baseDir) {
              return propulsor(std::make_unique<TabulatedThrust>(
                                   TabulatedThrust::fromCsv(
                                       resolve(cfg.str("table"), baseDir),
                                       cfg.str("time_col", "time_s"),
                                       cfg.str("thrust_col", "thrust_n"),
                                       cfg.boolean("throttle_gated", false))),
                               cfg);
          } },
        { "f16_engine",
          [](const json::Value& cfg, const std::string& baseDir) {
              return propulsor(std::make_unique<F16Engine>(
                                   F16Engine::fromCsv(
                                       resolve(cfg.str("dir"), baseDir))),
                               cfg);
          } },
    };
    return r;
}

} // namespace

void Factory::registerComponent(const std::string& type, Builder builder) {
    registry()[type] = std::move(builder);
}

std::unique_ptr<ForceComponent> Factory::create(const json::Value& config,
                                                const std::string& baseDir) {
    const std::string type = config.str("type");
    const auto it = registry().find(type);
    if (it == registry().end()) {
        std::string known;
        for (const auto& [k, v] : registry()) {
            if (!known.empty()) known += ", ";
            known += k;
        }
        throw std::invalid_argument("component::Factory: unknown component type '" +
                                    type + "' (registered: " + known + ")");
    }
    return it->second(config, baseDir);
}

} // namespace component
