#pragma once

#include <vector>

#include "math/LookupTable1D.h"
#include "propulsion/PropulsionModel.h"

// Solid rocket motor: tabulated thrust-time curve, ignores throttle.
// Propellant depletes in proportion to delivered impulse, so mass tracks
// the burn profile rather than draining linearly.
class SolidMotor : public PropulsionModel {
public:
    // curve: (time [s], thrust [N]) points, strictly ascending in time.
    SolidMotor(std::vector<std::pair<double, double>> curve,
               double propellantMass,
               double ignitionTime = 0.0);

    double thrustFromState(const PropulsionContext& ctx, const double* x) const override;
    double propellantMass(double time) const override;

    double burnTime() const { return burnTime_; }

private:
    LookupTable1D thrustCurve_;      // thrust vs time-since-ignition
    LookupTable1D impulseFraction_;  // delivered impulse fraction vs time
    double propellant_;              // total propellant [kg]
    double ignitionTime_;            // sim time the motor lights [s]
    double burnTime_;                // curve duration [s]
};
