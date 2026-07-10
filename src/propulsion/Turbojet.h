#pragma once

#include "propulsion/PropulsionModel.h"

// Throttleable air-breathing engine: thrust = maxThrust * throttle * (rho/rho0)^lapse.
// Fuel mass is not modeled (short-duration sims); extend propellantMass() if needed.
class Turbojet : public PropulsionModel {
public:
    Turbojet(double maxThrust, double densityLapseExponent = 0.7)
        : maxThrust_(maxThrust), lapse_(densityLapseExponent) {}

    double thrust(const PropulsionContext& ctx) override;

private:
    double maxThrust_;   // sea-level static thrust [N]
    double lapse_;       // altitude lapse exponent on density ratio
};
