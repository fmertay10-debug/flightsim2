#include <cstdio>
#include <exception>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "design/Linearizer.h"
#include "math/Units.h"
#include "scenario/ScenarioLoader.h"
#include "vehicle/VehicleFactory.h"

// flightsim --linearize <vehicle.json> [--altitude A] [--machs 0.3,0.6,...]
//   [--mass M] [--iyy I] [--xcg X] [--time T] [--trim] [--out plant.csv]
// Trim/linearize the REAL vehicle (its actual force components, ADR-0003's
// pure f(x,u)) into the short-period plant tools/design_autopilot.py consumes
// via --plant. Works for any airframe with an elevator channel.
static int runLinearize(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: flightsim --linearize <vehicle.json> "
                             "[--altitude A] [--machs a,b,c] [--mass M] "
                             "[--iyy I] [--xcg X] [--time T] [--trim] "
                             "[--out plant.csv]\n");
        return 2;
    }
    const std::string vpath = argv[2];
    design::LinearizerOptions opt;
    std::vector<double> machs = {0.3, 0.5, 0.7, 0.9, 1.5, 2.0, 2.5, 3.0};
    std::string outPath;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument(a + " needs a value");
            return argv[++i];
        };
        if      (a == "--altitude") opt.altitude = std::stod(next());
        else if (a == "--mass")     opt.mass = std::stod(next());
        else if (a == "--iyy")      opt.iyy = std::stod(next());
        else if (a == "--xcg")      opt.xcg = std::stod(next());
        else if (a == "--time")     opt.time = std::stod(next());
        else if (a == "--trim")     opt.trim = true;
        else if (a == "--out")      outPath = next();
        else if (a == "--machs") {
            machs.clear();
            std::stringstream ss(next());
            for (std::string tok; std::getline(ss, tok, ',');)
                machs.push_back(std::stod(tok));
        } else {
            throw std::invalid_argument("unknown option " + a);
        }
    }
    const std::filesystem::path vp(vpath);
    if (outPath.empty()) outPath = (vp.parent_path() / "plant.csv").string();

    const json::Value def = json::Value::parseFile(vpath);
    auto veh = vehicle::create(def, vp.parent_path().string());
    ChannelTable channels;
    veh->declareChannels(channels);
    const design::Linearizer lin(*veh, channels, opt);

    std::FILE* f = std::fopen(outPath.c_str(), "w");
    if (!f) throw std::runtime_error("cannot write " + outPath);
    std::fprintf(f, "mach,V_ms,qbar_pa,alpha_trim_rad,de_trim_rad,"
                    "Za,Zde,Ma,Mq,Mde\n");
    std::printf("linearized %s at %.0f m%s\n", vpath.c_str(), opt.altitude,
                opt.trim ? " (trimmed)" : "");
    std::printf("  %5s %7s %9s %8s %8s %8s %8s %9s %8s %9s\n", "mach", "V",
                "qbar", "a_trim", "de_trim", "Za", "Zde", "Ma", "Mq", "Mde");
    for (double m : machs) {
        const design::PlantPoint p = lin.at(m);
        if (opt.trim && !p.trimmed)
            std::fprintf(stderr, "warning: mach %.2f: trim did not converge; "
                                 "linearized about (%.4f, %.4f)\n",
                         m, p.alphaTrim, p.deTrim);
        std::fprintf(f, "%.3f,%.6g,%.6g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g\n",
                     p.mach, p.V, p.qbar, p.alphaTrim, p.deTrim,
                     p.Za, p.Zde, p.Ma, p.Mq, p.Mde);
        std::printf("  %5.2f %7.1f %9.0f %8.4f %8.4f %8.3f %8.3f %9.3f "
                    "%8.3f %9.3f\n", p.mach, p.V, p.qbar, p.alphaTrim,
                    p.deTrim, p.Za, p.Zde, p.Ma, p.Mq, p.Mde);
    }
    std::fclose(f);
    std::printf("  %zu points -> %s\n", machs.size(), outPath.c_str());
    return 0;
}

// flightsim <scenario.json> [--json]
// Loads the scenario, runs it, prints a per-entity summary. With --json prints
// one machine-readable line (for the Monte Carlo tool) instead.
int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: flightsim <scenario.json> [--json]\n"
                     "       flightsim --describe <scenario.json>   (build + validate, don't fly)\n"
                     "       flightsim --linearize <vehicle.json> [...]\n"
                     "example scenarios live in data/scenarios/\n");
        return 2;
    }
    if (std::string(argv[1]) == "--linearize") {
        try {
            return runLinearize(argc, argv);
        } catch (const std::exception& ex) {
            std::fprintf(stderr, "error: %s\n", ex.what());
            return 1;
        }
    }
    // Dry run: build the whole scenario, narrating every stage (definition
    // sources, components, channels, law binding, validation, guidance
    // wiring), then exit WITHOUT flying. The loader's validation still runs,
    // so this is also "check my config" without a 60 s simulation.
    if (std::string(argv[1]) == "--describe") {
        if (argc < 3) {
            std::fprintf(stderr, "usage: flightsim --describe <scenario.json>\n");
            return 2;
        }
        try {
            scenario::load(argv[2], /*verbose=*/true);
            std::printf("\nload OK -- everything built and validated "
                        "(run without --describe to fly)\n");
            return 0;
        } catch (const std::exception& ex) {
            std::fprintf(stderr, "\nload FAILED: %s\n", ex.what());
            return 1;
        }
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
