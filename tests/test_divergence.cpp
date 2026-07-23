#include "test_util.h"

#include <filesystem>
#include <fstream>
#include <string>

#include "scenario/ScenarioLoader.h"

// The divergence tripwire: a run whose physics goes non-finite must die
// loudly and early, not fly NaNs to a clean exit. (A corrupted mass table
// once produced a 55-second all-NaN flight with exit code 0.)

namespace {

void writeScenario(const std::string& path, const std::string& massCsv) {
    std::ofstream(path) << R"({
      "name": "divergence_probe",
      "simulation": { "dt_s": 0.005, "duration_s": 2, "ground_level_m": -10000 },
      "environment": { "gravity": "flat", "wind": { "type": "none" } },
      "vehicles": [{
        "name": "probe",
        "definition": {
          "mass": { "model": "tabulated", "table": ")" + massCsv + R"(" },
          "components": [{
            "type": "aircraft_aero", "sref_m2": 1.0, "cbar_m": 1.0,
            "bspan_m": 3.0, "cd0": 0.05, "cla": 5.0, "cma": -1.0, "cmq": -10.0
          }]
        },
        "dynamics": "six_dof",
        "initial": { "position_ned_m": [0, 0, -1000],
                     "velocity_ned_ms": [50, 0, 0] }
      }]
    })";
}

} // namespace

int main() {
    std::filesystem::create_directories("data/output");

    // Mass goes NaN at t = 0.5 s: the run must end early with the entity dead.
    std::ofstream("data/output/_test_div_mass_bad.csv")
        << "time_s,mass_kg,ixx,iyy,izz\n0,100,10,50,50\n0.5,nan,10,50,50\n";
    writeScenario("data/output/_test_div_bad.json", "_test_div_mass_bad.csv");
    {
        auto r = scenario::load("data/output/_test_div_bad.json");
        r.simulation->run();
        CHECK(!r.simulation->entity(0).alive());     // tripwire killed it
        CHECK(r.simulation->time() < 1.0);           // and early, not at t=2
    }

    // Control: the same vehicle with a healthy table flies to the end alive.
    std::ofstream("data/output/_test_div_mass_ok.csv")
        << "time_s,mass_kg,ixx,iyy,izz\n0,100,10,50,50\n2,100,10,50,50\n";
    writeScenario("data/output/_test_div_ok.json", "_test_div_mass_ok.csv");
    {
        auto r = scenario::load("data/output/_test_div_ok.json");
        r.simulation->run();
        CHECK(r.simulation->entity(0).alive());
        CHECK(r.simulation->time() >= 2.0 - 1e-9);
    }

    std::printf("test_divergence: all checks passed\n");
    return 0;
}
