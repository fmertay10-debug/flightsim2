#include "propulsion/SolidMotor.h"

#include <stdexcept>

SolidMotor::SolidMotor(std::vector<std::pair<double, double>> curve,
                       double propellantMass,
                       double ignitionTime)
    : propellant_(propellantMass), ignitionTime_(ignitionTime)
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

    // Cumulative impulse by trapezoid rule -> normalized fraction curve.
    std::vector<double> impulse(t.size(), 0.0);
    for (std::size_t i = 1; i < t.size(); ++i)
        impulse[i] = impulse[i - 1] + 0.5 * (f[i] + f[i - 1]) * (t[i] - t[i - 1]);
    const double total = impulse.back();
    if (total <= 0.0)
        throw std::invalid_argument("SolidMotor: thrust curve has zero total impulse");
    for (double& v : impulse) v /= total;
    impulseFraction_ = LookupTable1D(t, impulse);
}

double SolidMotor::thrust(const PropulsionContext& ctx) {
    const double tb = ctx.time - ignitionTime_;
    if (tb < 0.0 || tb > burnTime_) return 0.0;
    return thrustCurve_.eval(tb);
}

double SolidMotor::propellantMass(double time) const {
    const double tb = time - ignitionTime_;
    if (tb <= 0.0)       return propellant_;
    if (tb >= burnTime_) return 0.0;
    return propellant_ * (1.0 - impulseFraction_.eval(tb));
}
