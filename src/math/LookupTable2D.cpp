#include "math/LookupTable2D.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace {
    // Locate the interval [grid[i], grid[i+1]] containing v and the fraction f
    // across it. Clamps: below range -> (0, 0.0); above -> (n-2, 1.0).
    void locate(const std::vector<double>& grid, double v, int& i, double& f) {
        const int n = static_cast<int>(grid.size());
        if (v <= grid.front()) { i = 0;     f = 0.0; return; }
        if (v >= grid.back())  { i = n - 2; f = 1.0; return; }
        const int hi = static_cast<int>(
            std::upper_bound(grid.begin(), grid.end(), v) - grid.begin());
        i = hi - 1;
        f = (v - grid[i]) / (grid[i + 1] - grid[i]);
    }
}

LookupTable2D::LookupTable2D(std::vector<double> x,
                             std::vector<double> y,
                             std::vector<double> z)
    : x_(std::move(x)), y_(std::move(y)), z_(std::move(z))
{
    if (x_.size() < 2 || y_.size() < 2)
        throw std::invalid_argument("LookupTable2D: need >= 2 breakpoints per axis");
    if (z_.size() != x_.size() * y_.size())
        throw std::invalid_argument("LookupTable2D: z size must equal x.size()*y.size()");
    if (!std::is_sorted(x_.begin(), x_.end()) || !std::is_sorted(y_.begin(), y_.end()))
        throw std::invalid_argument("LookupTable2D: breakpoints must be ascending");
}

double LookupTable2D::eval(double x, double y) const {
    int i, j;
    double fx, fy;
    locate(x_, x, i, fx);
    locate(y_, y, j, fy);

    const int ny = static_cast<int>(y_.size());
    const double z00 = z_[i       * ny + j];
    const double z10 = z_[(i + 1) * ny + j];
    const double z01 = z_[i       * ny + (j + 1)];
    const double z11 = z_[(i + 1) * ny + (j + 1)];

    return (1.0 - fx) * (1.0 - fy) * z00
         +        fx  * (1.0 - fy) * z10
         + (1.0 - fx) *        fy  * z01
         +        fx  *        fy  * z11;
}
