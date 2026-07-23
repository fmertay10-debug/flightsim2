#include "test_util.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include "scenario/ScenarioLoader.h"

// Solution verification: forward Euler is first-order, so halving dt must
// halve the error against a fine-dt reference. Run the same boost-climb
// scenario end-to-end (loader -> entity -> components -> EOM) at dt = 4, 2,
// 1 ms against a 0.5 ms reference and check the observed convergence order.
// This is what proves the integrator is converging to the ODE's solution --
// and quantifies the Euler tax per halving.

namespace {

std::string scenarioAt(double dt, const std::string& tag) {
    const std::string path = "data/output/_test_conv_" + tag + ".json";
    std::ofstream(path) << R"({
      "name": "convergence_probe",
      "simulation": { "dt_s": )" + std::to_string(dt) + R"(,
                      "duration_s": 6, "ground_level_m": 0 },
      "environment": { "gravity": "flat", "wind": { "type": "none" } },
      "vehicles": [{
        "name": "probe",
        "definition": {
          "mass": { "model": "tabulated", "table": "_test_conv_mass.csv" },
          "components": [
            { "type": "aircraft_aero", "sref_m2": 0.2, "cbar_m": 2.0,
              "bspan_m": 0.5, "cd0": 0.3, "cla": 8.0, "cma": -2.0,
              "cmq": -40.0, "clp": -4.0 },
            { "type": "solid_motor", "propellant_kg": 10,
              "thrust_curve": [[0, 3000], [3, 3000], [3.5, 0]] }
          ]
        },
        "dynamics": "six_dof",
        "initial": { "position_ned_m": [0, 0, -10],
                     "velocity_ned_ms": [5.2, 0, -29.5],
                     "euler_deg": [0, 80, 0] }
      }]
    })";
    return path;
}

struct Endpoint { Vector3 pos, vel; };

Endpoint fly(double dt, const std::string& tag) {
    auto r = scenario::load(scenarioAt(dt, tag));
    r.simulation->run();
    const State& s = r.simulation->entity(0).state();
    CHECK(r.simulation->entity(0).alive());         // whole flight airborne
    return { s.position, s.velocity };
}

double errorVs(const Endpoint& e, const Endpoint& ref) {
    return (e.pos - ref.pos).norm() + 5.0 * (e.vel - ref.vel).norm();
}

} // namespace

int main() {
    std::filesystem::create_directories("data/output");
    std::ofstream("data/output/_test_conv_mass.csv")
        << "time_s,mass_kg,ixx,iyy,izz\n0,100,5,60,60\n6,100,5,60,60\n";

    const Endpoint ref = fly(0.0005, "ref");
    const double e4 = errorVs(fly(0.004, "dt4"), ref);
    const double e2 = errorVs(fly(0.002, "dt2"), ref);
    const double e1 = errorVs(fly(0.001, "dt1"), ref);

    const double p42 = std::log2(e4 / e2);
    const double p21 = std::log2(e2 / e1);
    std::printf("test_convergence: err(4ms) %.4g  err(2ms) %.4g  err(1ms) "
                "%.4g  order %.2f, %.2f\n", e4, e2, e1, p42, p21);

    CHECK(e4 > e2 && e2 > e1);          // error shrinks monotonically with dt
    CHECK(p42 > 0.7 && p42 < 1.4);      // ...at first order, as Euler must
    CHECK(p21 > 0.7 && p21 < 1.4);
    CHECK(e4 > 1e-6);                   // the measurement is not degenerate

    std::printf("test_convergence: all checks passed\n");
    return 0;
}
