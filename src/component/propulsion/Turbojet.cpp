#include "component/propulsion/Turbojet.h"

#include <algorithm>
#include <cmath>

double Turbojet::thrustFromState(const PropulsionContext& ctx,
                                 const double* /*x: stateless*/) const {
    constexpr double RHO0 = 1.225;   // ISA sea-level density [kg/m^3]
    const double demand = std::clamp(ctx.throttle, 0.0, 1.0);
    const double lapse  = std::pow(ctx.density / RHO0, lapse_);
    return maxThrust_ * demand * lapse;
}
