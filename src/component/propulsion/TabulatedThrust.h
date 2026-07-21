#pragma once

#include <string>
#include <vector>

#include "math/LookupTable1D.h"
#include "component/propulsion/PropulsionModel.h"

// Variable thrust as a raw thrust-vs-time lookup table, independent of the
// mass model (mass/inertia/CG are supplied separately, e.g. TabulatedMassModel).
// Throttle-gated: the tabulated profile is scaled by the throttle command, so
// a controller can still throttle back. Ignores flight condition.
class TabulatedThrust : public PropulsionModel {
public:
    // curve: (time [s], thrust [N]) points, strictly ascending in time.
    // Outside the table the endpoint value holds (clamped by LookupTable1D).
    explicit TabulatedThrust(std::vector<std::pair<double, double>> curve,
                             bool throttleGated = false);

    static TabulatedThrust fromCsv(const std::string& path,
                                   const std::string& timeCol = "time_s",
                                   const std::string& thrustCol = "thrust_n",
                                   bool throttleGated = false);

    double thrustFromState(const PropulsionContext& ctx, const double* x) const override;

private:
    LookupTable1D curve_;
    double        endTime_;
    bool          gated_;
};
