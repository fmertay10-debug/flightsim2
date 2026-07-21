#include "mass/TabulatedMassModel.h"

#include <cmath>
#include <vector>

#include "io/CsvReader.h"

TabulatedMassModel TabulatedMassModel::fromCsv(const std::string& path) {
    const csv::Table t = csv::read(path);
    const std::size_t ti = t.col("time_s");

    const auto column = [&](const std::string& name) {
        const std::size_t ci = t.col(name);
        std::vector<double> time, val;
        time.reserve(t.rows.size());
        val.reserve(t.rows.size());
        for (const auto& r : t.rows) { time.push_back(r[ti]); val.push_back(r[ci]); }
        return LookupTable1D(std::move(time), std::move(val));
    };
    // Absent optional column -> a constant over the table's time span.
    const auto optionalColumn = [&](const std::string& name, double fallback) {
        if (t.hasCol(name)) return column(name);
        const double t0 = t.rows.front()[ti];
        return LookupTable1D({t0, t0 + 1.0}, {fallback, fallback});
    };

    return TabulatedMassModel(column("mass_kg"), column("ixx"), column("iyy"),
                              column("izz"),
                              optionalColumn("ixy", 0.0),
                              optionalColumn("ixz", 0.0),
                              optionalColumn("iyz", 0.0),
                              optionalColumn("xcg_m", std::nan("")));
}
