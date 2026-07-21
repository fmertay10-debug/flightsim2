#pragma once

#include <vector>

#include "math/LookupTable1D.h"
#include "component/propulsion/PropulsionModel.h"

// Solid rocket motor: tabulated thrust-time curve, ignores throttle. The
// propellant drain lives in the vehicle's tabulated mass CSV (the generators
// sample it impulse-proportionally from this same curve).
class SolidMotor : public PropulsionModel {
public:
    // curve: (time [s], thrust [N]) points, strictly ascending in time.
    SolidMotor(std::vector<std::pair<double, double>> curve,
               double ignitionTime = 0.0);

    double thrustFromState(const PropulsionContext& ctx, const double* x) const override;

    double burnTime() const { return burnTime_; }

private:
    LookupTable1D thrustCurve_;      // thrust vs time-since-ignition
    double ignitionTime_;            // sim time the motor lights [s]
    double burnTime_;                // curve duration [s]
};
