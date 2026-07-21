#include "vehicle/VehicleFactory.h"

#include <filesystem>
#include <stdexcept>
#include <utility>

#include "component/ComponentFactory.h"
#include "mass/TabulatedMassModel.h"

namespace vehicle {

namespace {

std::unique_ptr<MassModel> buildMass(const json::Value& def,
                                     const std::string& baseDir) {
    if (def.has("mass")) {
        const json::Value& m = def.at("mass");
        const std::string model = m.str("model", "tabulated");
        if (model == "tabulated") {
            const std::filesystem::path p(m.str("table"));
            const std::string path = p.is_absolute()
                ? p.string() : (std::filesystem::path(baseDir) / p).string();
            return std::make_unique<TabulatedMassModel>(
                       TabulatedMassModel::fromCsv(path));
        }
        // The "constant" and "dry_plus_propellant" models were retired
        // (2026-07-21): both are special cases of a tabulated CSV -- a
        // constant vehicle is a two-row table; a burning motor's drain is
        // sampled into rows by the vehicle generators.
        throw std::invalid_argument(
            "vehicle: unknown mass model '" + model + "' -- the only model is "
            "\"tabulated\" ({\"mass\": {\"model\": \"tabulated\", \"table\": "
            "\"mass_props.csv\"}}; see docs/BUILDING_VEHICLES.md for the CSV "
            "columns).");
    }

    throw std::invalid_argument(
        "vehicle: no \"mass\" block. Use {\"mass\": {\"model\": \"tabulated\", "
        "\"table\": \"mass_props.csv\"}} -- constant mass is a two-row table "
        "(see docs/BUILDING_VEHICLES.md).");
}

} // namespace

std::unique_ptr<Vehicle> create(const json::Value& def, const std::string& baseDir) {
    const json::Value& list = def.at("components");
    if (!list.isArray())
        throw std::invalid_argument("vehicle: 'components' must be an array");

    std::vector<std::unique_ptr<ForceComponent>> components;
    std::vector<std::string> names;
    components.reserve(list.size());
    names.reserve(list.size());
    for (std::size_t i = 0; i < list.size(); ++i) {
        const std::string type = list[i].str("type");
        components.push_back(component::Factory::create(list[i], baseDir));
        // Telemetry label = the config type, suffixed when it repeats.
        std::string label = type;
        int dup = 1;
        for (const std::string& n : names)
            if (n == type || n.rfind(type + "_", 0) == 0) ++dup;
        if (dup > 1) label += "_" + std::to_string(dup);
        names.push_back(std::move(label));
    }

    return std::make_unique<Vehicle>(buildMass(def, baseDir),
                                     std::move(components), std::move(names));
}

} // namespace vehicle
