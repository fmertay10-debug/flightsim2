#include "mass/TabulatedMassModel.h"

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

    return TabulatedMassModel(column("mass_kg"), column("ixx"), column("iyy"),
                              column("izz"), column("xcg_m"));
}
