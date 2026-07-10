#pragma once

#include <vector>

// 1-D linear interpolation over ascending breakpoints x with values y.
// Clamps to the endpoint value outside [x.front(), x.back()].
// Used for thrust(t) curves and gain schedules.
class LookupTable1D {
public:
    LookupTable1D() = default;
    LookupTable1D(std::vector<double> x,    // breakpoints (strictly ascending)
                  std::vector<double> y);   // values, same size as x

    double eval(double x) const;

    bool empty() const { return x_.empty(); }

private:
    std::vector<double> x_;
    std::vector<double> y_;
};
