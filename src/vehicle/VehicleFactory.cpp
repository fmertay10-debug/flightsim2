#include "vehicle/VehicleFactory.h"

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include "component/ComponentFactory.h"
#include "mass/ConstantMassModel.h"
#include "mass/TabulatedMassModel.h"

namespace vehicle {

namespace {

Matrix3x3 inertiaFromJson(const json::Value& in) {
    const double ixy = in.num("ixy", 0.0);
    const double ixz = in.num("ixz", 0.0);
    const double iyz = in.num("iyz", 0.0);
    // Products of inertia enter the tensor as NEGATIVE off-diagonals.
    return Matrix3x3(in.num("ixx"), -ixy,          -ixz,
                     -ixy,          in.num("iyy"), -iyz,
                     -ixz,          -iyz,          in.num("izz"));
}

// Returns {massModel, addMotorPropellant}.
std::pair<std::unique_ptr<MassModel>, bool>
buildMass(const json::Value& def, const std::string& baseDir) {
    if (def.has("mass")) {
        const json::Value& m = def.at("mass");
        const std::string model = m.str("model", "constant");
        if (model == "tabulated") {
            const std::filesystem::path p(m.str("table"));
            const std::string path = p.is_absolute()
                ? p.string() : (std::filesystem::path(baseDir) / p).string();
            return {std::make_unique<TabulatedMassModel>(
                        TabulatedMassModel::fromCsv(path)), false};
        }
        if (model == "constant") {
            MassState s;
            s.mass    = m.num("mass_kg");
            s.inertia = inertiaFromJson(m.at("inertia"));
            s.xcg     = m.has("xcg_m") ? m.num("xcg_m") : std::nan("");
            return {std::make_unique<ConstantMassModel>(s), false};
        }
        if (model == "dry_plus_propellant") {
            // Constant dry mass/inertia; the motors' remaining propellant is
            // added on top each step, so mass drops through the burn. Explicit
            // form of what the legacy flat schema did implicitly.
            MassState s;
            s.mass    = m.num("dry_mass_kg");
            s.inertia = inertiaFromJson(m.at("inertia"));
            s.xcg     = m.has("xcg_m") ? m.num("xcg_m") : std::nan("");
            return {std::make_unique<ConstantMassModel>(s), true};
        }
        throw std::invalid_argument("vehicle: unknown mass model '" + model + "'");
    }

    // The legacy flat mass_kg + inertia form was retired after every config
    // migrated (2026-07-19); its behavior lives on as the explicit
    // "dry_plus_propellant" model.
    throw std::invalid_argument(
        "vehicle: no \"mass\" block. The flat mass_kg + inertia form was "
        "retired: use {\"mass\": {\"model\": \"constant\" | "
        "\"dry_plus_propellant\" | \"tabulated\", ...}} -- "
        "dry_plus_propellant reproduces the old dry-mass + motor-propellant "
        "behavior (see docs/BUILDING_VEHICLES.md).");
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

    auto [mass, addPropellant] = buildMass(def, baseDir);
    return std::make_unique<Vehicle>(std::move(mass), std::move(components),
                                     addPropellant, std::move(names));
}

} // namespace vehicle
