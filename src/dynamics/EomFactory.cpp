#include "dynamics/EomFactory.h"

#include <stdexcept>

#include "dynamics/SixDofEom.h"
#include "dynamics/PointMassEom.h"
#include "dynamics/KinematicEom.h"

namespace eom {

std::unique_ptr<EquationsOfMotion> create(const std::string& name, const Options& opts) {
    if (name == "six_dof")    return std::make_unique<SixDofEom>();
    if (name == "point_mass") return std::make_unique<PointMassEom>();
    if (name == "kinematic")  return std::make_unique<KinematicEom>(opts.turnRate);
    throw std::invalid_argument("eom::create: unknown dynamics type '" + name +
                                "' (expected six_dof, point_mass, or kinematic)");
}

} // namespace eom
