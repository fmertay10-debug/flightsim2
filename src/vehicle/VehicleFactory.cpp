#include "vehicle/VehicleFactory.h"

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include "mass/ConstantMassModel.h"
#include "mass/TabulatedMassModel.h"
#include "propulsion/PropulsionFactory.h"

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
buildMass(const json::Value& def, const std::string& baseDir, bool hasSolidMotor) {
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
        throw std::invalid_argument("vehicle: unknown mass model '" + model + "'");
    }

    // Legacy flat form: dry mass + inertia; a solid motor's propellant is
    // added on top so existing configs keep their burn-time mass drop.
    MassState s;
    s.mass    = def.num("mass_kg");
    s.inertia = inertiaFromJson(def.at("inertia"));
    s.xcg     = def.has("xcg_m") ? def.num("xcg_m") : std::nan("");
    return {std::make_unique<ConstantMassModel>(s), hasSolidMotor};
}

} // namespace

std::unique_ptr<Vehicle> create(const json::Value& def, const std::string& baseDir) {
    std::unique_ptr<PropulsionModel> prop;
    bool hasSolidMotor = false;
    if (def.has("propulsion")) {
        const json::Value& p = def.at("propulsion");
        hasSolidMotor = (p.str("type", "none") == "solid_motor");
        prop = propulsion::create(p, baseDir);
    }

    auto [mass, addPropellant] = buildMass(def, baseDir, hasSolidMotor);

    return std::make_unique<Vehicle>(def.str("type"), std::move(mass),
                                     std::move(prop), addPropellant);
}

} // namespace vehicle
