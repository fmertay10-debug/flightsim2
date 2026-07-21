#include "component/propulsion/SolidMotor.h"

#include <stdexcept>

SolidMotor::SolidMotor(std::vector<std::pair<double, double>> curve,
                       double ignitionTime)
    : ignitionTime_(ignitionTime)
{
    if (curve.size() < 2)
        throw std::invalid_argument("SolidMotor: thrust curve needs >= 2 points");

    std::vector<double> t, f;
    t.reserve(curve.size());
    f.reserve(curve.size());
    for (const auto& p : curve) {
        t.push_back(p.first);
        f.push_back(p.second);
    }
    burnTime_ = t.back();
    thrustCurve_ = LookupTable1D(t, f);
}

double SolidMotor::thrustFromState(const PropulsionContext& ctx,
                                   const double* /*x: stateless*/) const {
    const double tb = ctx.time - ignitionTime_;
    if (tb < 0.0 || tb > burnTime_) return 0.0;
    return thrustCurve_.eval(tb);
}
