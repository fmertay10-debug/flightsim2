#pragma once

#include <memory>
#include <string>

#include "sim/Simulation.h"

// Builds a ready-to-run Simulation from a scenario JSON file. This is the
// single place where config text meets the factories -- no scenario logic
// is compiled into the sim core.
//
// Scenario schema (angles in degrees; sim core is SI radians):
// {
//   "name": "my_scenario",
//   "simulation":  {"dt_s": 0.002, "duration_s": 60, "ground_level_m": 0},
//   "environment": {"gravity": "flat" | "spherical",
//                   "wind": {"type": "none" | "constant", "ned_ms": [n, e, d]}},
//   "vehicles": [
//     {
//       "name": "chase-1",
//       "vehicle": "../vehicles/trainer.json",  // path relative to the scenario file
//       // ... or "definition": { inline vehicle definition }
//       "dynamics": "six_dof" | "kinematic",
//       "turn_rate_dps": 0,                     // kinematic only
//       "initial": {"position_ned_m": [n, e, d], "velocity_ned_ms": [n, e, d],
//                   "euler_deg": [roll, pitch, yaw], "rates_dps": [p, q, r]},
//       "flight_plan": [ {"time_s": 0, "altitude_m": 1000, ...}, ... ],
//       "log": "output/chase-1.csv",            // optional; relative to CWD
//       "log_decimation": 10                    // optional; log every Nth step
//     }
//   ]
// }
//
// Vehicle definition schema (vehicles/*.json, see docs/BUILDING_VEHICLES.md):
// {
//   "mass":       {"model": "tabulated", "table": "mass_props.csv"},
//   "components": [ {"type": <component::Factory name>, ...}, ... ],
//   "gnc": {
//     "control_law": {"type": <gnc::Factory name>, ...},
//     "actuator":    {"tau_s": ..., "rate_dps": ..., "limit_deg": ...}  // optional
//   }
// }
namespace scenario {

struct LoadResult {
    std::string                 name;
    std::unique_ptr<Simulation> simulation;
};

// verbose = narrate the build to stdout as it happens (the `flightsim
// --describe` dry run): definition sources, components, declared channels,
// law binding, validation results, guidance wiring.
LoadResult load(const std::string& scenarioPath, bool verbose = false);

} // namespace scenario
