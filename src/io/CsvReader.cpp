#include "io/CsvReader.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace csv {

std::size_t Table::col(const std::string& name) const {
    for (std::size_t i = 0; i < columns.size(); ++i)
        if (columns[i] == name) return i;
    throw std::runtime_error("csv: no column '" + name + "'");
}

bool Table::hasCol(const std::string& name) const {
    return std::find(columns.begin(), columns.end(), name) != columns.end();
}

Table read(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("csv: cannot open '" + path + "'");

    Table t;
    std::string line;

    if (!std::getline(in, line))
        throw std::runtime_error("csv: empty file '" + path + "'");
    {
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ','))
            t.columns.push_back(cell);
    }

    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<double> row;
        row.reserve(t.columns.size());
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ','))
            row.push_back(std::stod(cell));
        if (row.size() != t.columns.size())
            throw std::runtime_error("csv: ragged row in '" + path + "'");
        t.rows.push_back(std::move(row));
    }
    return t;
}

std::map<std::string, double> readKeyValue(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("csv: cannot open '" + path + "'");

    std::map<std::string, double> out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const std::size_t comma = line.find(',');
        if (comma == std::string::npos) continue;
        const std::string key = line.substr(0, comma);
        out[key] = std::stod(line.substr(comma + 1));
    }
    return out;
}

LookupTable2D buildTable2D(const Table& table,
                           const std::string& xCol,
                           const std::string& yCol,
                           const std::string& zCol) {
    const std::size_t xi = table.col(xCol);
    const std::size_t yi = table.col(yCol);
    const std::size_t zi = table.col(zCol);

    // Unique sorted breakpoints. Values arrive with fixed precision, so
    // exact comparison is safe.
    std::vector<double> xs, ys;
    for (const auto& r : table.rows) { xs.push_back(r[xi]); ys.push_back(r[yi]); }
    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    std::sort(ys.begin(), ys.end());
    ys.erase(std::unique(ys.begin(), ys.end()), ys.end());

    const std::size_t nx = xs.size(), ny = ys.size();
    if (table.rows.size() != nx * ny)
        throw std::runtime_error("csv: '" + zCol + "' rows do not form a complete " +
                                 std::to_string(nx) + "x" + std::to_string(ny) + " grid");

    const auto index = [](const std::vector<double>& grid, double v) {
        const auto it = std::lower_bound(grid.begin(), grid.end(), v);
        return static_cast<std::size_t>(it - grid.begin());
    };

    std::vector<double> z(nx * ny, std::nan(""));
    for (const auto& r : table.rows)
        z[index(xs, r[xi]) * ny + index(ys, r[yi])] = r[zi];

    for (double v : z)
        if (std::isnan(v))
            throw std::runtime_error("csv: grid for '" + zCol + "' has holes");

    return LookupTable2D(std::move(xs), std::move(ys), std::move(z));
}

} // namespace csv
