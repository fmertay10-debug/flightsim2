#pragma once

#include <map>
#include <string>
#include <vector>

#include "math/LookupTable2D.h"

// Tidy-CSV reading for aero table files (header row + numeric rows).
namespace csv {

struct Table {
    std::vector<std::string>         columns;
    std::vector<std::vector<double>> rows;      // rows[i][col]

    // Column index by header name; throws std::runtime_error if absent.
    std::size_t col(const std::string& name) const;
};

Table read(const std::string& path);

// Read a two-column "key,value" file (no header) into a map. Used for the
// scalar reference files (reference.csv: sref_m2, cbar_m, ...).
std::map<std::string, double> readKeyValue(const std::string& path);

// Build a bilinear lookup from tidy (x, y, z) columns. The rows must cover a
// complete rectilinear grid (every x paired with every y, any order).
LookupTable2D buildTable2D(const Table& table,
                           const std::string& xCol,
                           const std::string& yCol,
                           const std::string& zCol);

} // namespace csv
