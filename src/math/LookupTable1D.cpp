#include "math/LookupTable1D.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

LookupTable1D::LookupTable1D(std::vector<double> x, std::vector<double> y)
    : x_(std::move(x)), y_(std::move(y))
{
    if (x_.size() != y_.size())
        throw std::invalid_argument("LookupTable1D: x and y must be the same size");
    if (x_.size() < 2)
        throw std::invalid_argument("LookupTable1D: need >= 2 breakpoints");
    for (std::size_t i = 1; i < x_.size(); ++i)
        if (!(x_[i] > x_[i - 1]))
            throw std::invalid_argument("LookupTable1D: breakpoints must be strictly ascending");
}

double LookupTable1D::eval(double x) const {
    if (x <= x_.front()) return y_.front();
    if (x >= x_.back())  return y_.back();

    const auto it = std::upper_bound(x_.begin(), x_.end(), x);
    const std::size_t hi = static_cast<std::size_t>(it - x_.begin());
    const std::size_t lo = hi - 1;

    const double t = (x - x_[lo]) / (x_[hi] - x_[lo]);
    return y_[lo] + t * (y_[hi] - y_[lo]);
}
