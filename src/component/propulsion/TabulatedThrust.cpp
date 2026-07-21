#include "component/propulsion/TabulatedThrust.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "io/CsvReader.h"

TabulatedThrust::TabulatedThrust(std::vector<std::pair<double, double>> curve,
                                 bool throttleGated)
    : gated_(throttleGated)
{
    if (curve.size() < 2)
        throw std::invalid_argument("TabulatedThrust: need >= 2 curve points");
    std::vector<double> t, f;
    t.reserve(curve.size());
    f.reserve(curve.size());
    for (const auto& p : curve) { t.push_back(p.first); f.push_back(p.second); }
    endTime_ = t.back();
    curve_ = LookupTable1D(std::move(t), std::move(f));
}

TabulatedThrust TabulatedThrust::fromCsv(const std::string& path,
                                         const std::string& timeCol,
                                         const std::string& thrustCol,
                                         bool throttleGated) {
    const csv::Table tbl = csv::read(path);
    const std::size_t ti = tbl.col(timeCol);
    const std::size_t fi = tbl.col(thrustCol);
    std::vector<std::pair<double, double>> curve;
    curve.reserve(tbl.rows.size());
    for (const auto& r : tbl.rows) curve.emplace_back(r[ti], r[fi]);
    return TabulatedThrust(std::move(curve), throttleGated);
}

double TabulatedThrust::thrustFromState(const PropulsionContext& ctx,
                                        const double* /*x: stateless*/) const {
    if (ctx.time > endTime_) return 0.0;              // burnout
    const double raw = curve_.eval(ctx.time);
    return gated_ ? raw * std::clamp(ctx.throttle, 0.0, 1.0) : raw;
}
