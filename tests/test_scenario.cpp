#include "test_util.h"

#include <cmath>

#include "scenario/ScenarioLoader.h"

// End-to-end smoke tests over the shipped scenarios. Run with the repo root
// as working directory (set by CMake).

static void testAircraftCruise() {
    scenario::LoadResult loaded = scenario::load("scenarios/aircraft_cruise.json");
    CHECK(loaded.name == "aircraft_cruise");
    Simulation& sim = *loaded.simulation;
    CHECK(sim.entities().size() == 1);
    sim.run();

    const State& s = sim.entities()[0]->state();
    CHECK(sim.entities()[0]->alive());
    CHECK(std::isfinite(s.position.norm()));
    // Commanded: 1000 m, 60 m/s, heading east after t=60.
    CHECK_NEAR(s.altitude(), 1000.0, 30.0);
    CHECK_NEAR(s.velocity.norm(), 60.0, 6.0);
    CHECK_NEAR(s.eulerAngles().z, 1.5708, 0.1);
}

static void testInterceptHits() {
    scenario::LoadResult loaded = scenario::load("scenarios/intercept.json");
    Simulation& sim = *loaded.simulation;
    sim.run();
    const InterceptResult& ic = sim.interceptResult();
    CHECK(ic.watching);
    CHECK(ic.hit);                         // ProNav + agile missile must connect
    CHECK(ic.missDistance < 8.0);
}

static void testF16Stable() {
    // Real tabular F-16 (statically unstable): the SAS-style autopilot must
    // hold the commanded climb to 3500 m and 150 m/s without departing.
    scenario::LoadResult loaded = scenario::load("scenarios/f16_cruise.json");
    Simulation& sim = *loaded.simulation;
    sim.run();
    const State& s = sim.entities()[0]->state();
    CHECK(sim.entities()[0]->alive());
    CHECK(std::isfinite(s.velocity.norm()));
    CHECK_NEAR(s.velocity.norm(), 150.0, 15.0);
    CHECK_NEAR(s.altitude(), 3500.0, 60.0);
}

static void testDatcomRocket() {
    // Table-aero rocket must reach a sensible apogee and come back intact.
    scenario::LoadResult loaded = scenario::load("scenarios/datcom_rocket_launch.json");
    Simulation& sim = *loaded.simulation;
    double apogee = 0.0;
    while (sim.step())
        apogee = std::max(apogee, sim.entities()[0]->state().altitude());
    CHECK(apogee > 1500.0);
    CHECK(std::isfinite(sim.entities()[0]->state().position.norm()));
}

static void testVariableRocket() {
    // Variable thrust + mass + inertia + CG rocket: mass must drop through the
    // burn (tabulated mass model + tabulated thrust) and it must reach apogee.
    scenario::LoadResult loaded = scenario::load("scenarios/advanced_rocket_launch.json");
    Simulation& sim = *loaded.simulation;
    double apogee = 0.0, massStart = 0.0, massEnd = 0.0;
    bool first = true;
    while (sim.step()) {
        const Telemetry& t = sim.entities()[0]->telemetry();
        if (first && t.state.time > 0.0) { massStart = t.mass; first = false; }
        massEnd = t.mass;
        apogee = std::max(apogee, sim.entities()[0]->state().altitude());
    }
    CHECK(apogee > 3000.0);
    CHECK_NEAR(massStart, 85.0, 3.0);       // full at liftoff
    CHECK(massEnd < 62.0);                   // burned down to ~dry
    CHECK(massStart - massEnd > 20.0);       // real propellant consumed
}

static void testLqrRocket() {
    // The LQR gain-scheduled autopilot (auto-designed gains) must fly the same
    // airframe and pitch program as the PID version to a comparable apogee.
    scenario::LoadResult loaded = scenario::load("scenarios/lqr_rocket_launch.json");
    Simulation& sim = *loaded.simulation;
    double apogee = 0.0;
    while (sim.step())
        apogee = std::max(apogee, sim.entities()[0]->state().altitude());
    CHECK(apogee > 2500.0);
    CHECK(std::isfinite(sim.entities()[0]->state().position.norm()));
}

static void testMissileIntercept() {
    // Full pipeline vehicle: DATCOM aero + auto-designed LQR autopilot + ProNav.
    // The AAM must intercept its maneuvering target.
    scenario::LoadResult loaded = scenario::load("scenarios/aam_intercept.json");
    Simulation& sim = *loaded.simulation;
    sim.run();
    const InterceptResult& ic = sim.interceptResult();
    CHECK(ic.watching);
    CHECK(ic.hit);
    CHECK(ic.missDistance < 10.0);
}

static void testTvcRocket() {
    // Thrust-vectored rocket (no aero control surfaces): the gimbal must tip it
    // over, tracking the pitch program DURING the burn. Capture near burnout.
    scenario::LoadResult loaded = scenario::load("scenarios/tvc_launch.json");
    Simulation& sim = *loaded.simulation;
    double thetaAtBurnout = 90.0, maxRollRate = 0.0;
    while (sim.step()) {
        const State& s = sim.entities()[0]->state();
        maxRollRate = std::max(maxRollRate, std::abs(s.angularRate.x));
        if (s.time <= 7.6) thetaAtBurnout = s.eulerAngles().y;   // last boost sample
    }
    // Started at 88 deg; TVC must have tipped it well over toward the ~55 deg cmd.
    CHECK(thetaAtBurnout < 1.30);            // < ~74 deg (radians)
    CHECK(thetaAtBurnout > 0.70);            // > ~40 deg (tracked, not tumbled)
    CHECK(maxRollRate < 0.2);                // roll passively bounded (no roll ctrl)
}

int main() {
    testAircraftCruise();
    testInterceptHits();
    testF16Stable();
    testDatcomRocket();
    testVariableRocket();
    testLqrRocket();
    testMissileIntercept();
    testTvcRocket();
    std::printf("test_scenario: all checks passed\n");
    return 0;
}
