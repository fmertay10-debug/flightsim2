#pragma once

#include <memory>
#include <string>

#include "dynamics/EquationsOfMotion.h"

// Factory: EOM strategy from its config name.
//   "six_dof"    -> SixDofEom
//   "kinematic"  -> KinematicEom (opts.turnRate applies)
namespace eom {

struct Options {
    double turnRate = 0.0;   // kinematic only [rad/s]
};

std::unique_ptr<EquationsOfMotion> create(const std::string& name,
                                          const Options& opts = {});

} // namespace eom
