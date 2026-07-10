#pragma once

#include <vector>

// Bilinear interpolation over a rectilinear (x, y) grid.
// Values are stored row-major: z[i*ny + j] is the value at (x[i], y[j]).
// Intended for aero coefficient tables C(alpha, Mach).
class LookupTable2D {
public:
    LookupTable2D() = default;
    LookupTable2D(std::vector<double> x,    // breakpoints along dim 0 (ascending)
                  std::vector<double> y,    // breakpoints along dim 1 (ascending)
                  std::vector<double> z);   // size must equal x.size() * y.size()

    // Bilinear interpolation. Clamps to the nearest edge value outside the grid.
    double eval(double x, double y) const;

    bool empty() const { return x_.empty(); }

private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> z_;   // row-major: z_[i*ny + j]
};
