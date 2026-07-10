#include <cstdio>
#include <exception>

#include "math/Units.h"
#include "scenario/ScenarioLoader.h"

// flightsim <scenario.json> [--json]
// Loads the scenario, runs it, prints a per-entity summary. With --json prints
// one machine-readable line (for the Monte Carlo tool) instead.
int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: flightsim <scenario.json> [--json]\n"
                     "example scenarios live in scenarios/\n");
        return 2;
    }
    bool jsonOut = false;
    for (int i = 2; i < argc; ++i)
        if (std::string(argv[i]) == "--json") jsonOut = true;

    try {
        scenario::LoadResult loaded = scenario::load(argv[1]);
        Simulation& sim = *loaded.simulation;
        if (!jsonOut) std::printf("scenario: %s\n", loaded.name.c_str());
        sim.run();

        const InterceptResult& ic = sim.interceptResult();

        if (jsonOut) {
            // Flat JSON: intercept outcome + miss vector (target-pursuer, NED).
            std::printf("{\"finished_s\":%.4f,\"intercept\":%s,\"hit\":%s,"
                        "\"miss_m\":%.4f,\"time_s\":%.4f,"
                        "\"miss_ned\":[%.4f,%.4f,%.4f]}\n",
                        sim.time(),
                        ic.watching ? "true" : "false",
                        ic.hit ? "true" : "false",
                        ic.missDistance, ic.time,
                        ic.relPos.x, ic.relPos.y, ic.relPos.z);
            return 0;
        }

        std::printf("finished at t = %.3f s\n", sim.time());
        if (ic.watching)
            std::printf("intercept: %s -- closest approach %.2f m at t = %.3f s\n",
                        ic.hit ? "HIT" : "miss", ic.missDistance, ic.time);
        std::printf("\n");

        std::printf("%-16s %-6s %10s %10s %10s %10s\n",
                    "entity", "alive", "north[m]", "east[m]", "alt[m]", "speed[m/s]");
        for (const auto& e : sim.entities()) {
            const State& s = e->state();
            std::printf("%-16s %-6s %10.1f %10.1f %10.1f %10.1f\n",
                        e->name().c_str(), e->alive() ? "yes" : "no",
                        s.position.x, s.position.y, s.altitude(), s.velocity.norm());
        }
        return 0;
    } catch (const std::exception& ex) {
        if (jsonOut) std::printf("{\"error\":\"%s\"}\n", ex.what());
        else std::fprintf(stderr, "error: %s\n", ex.what());
        return 1;
    }
}
