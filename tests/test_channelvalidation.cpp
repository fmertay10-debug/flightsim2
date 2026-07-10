#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>

#include "aero/RocketAero.h"
#include "control/RocketController.h"
#include "control/TvcController.h"
#include "effector/ThrustEffector.h"
#include "effector/TvcEffector.h"
#include "io/Json.h"
#include "scenario/ScenarioLoader.h"
#include "test_util.h"

// The channel write/read graph is validated at LOAD: a controller whose
// required channels no component declares must throw, not fly open-loop.

static bool throwsMentioning(const std::function<void()>& f,
                             const std::string& needle) {
    try {
        f();
    } catch (const std::exception& e) {
        return std::string(e.what()).find(needle) != std::string::npos;
    }
    return false;
}

int main() {
    // A TVC airframe: stability derivatives only, no fin authority.
    const json::Value tvcAero = json::Value::parse(R"({
        "sref_m2": 0.2, "lref_m": 8.0, "dref_m": 0.5,
        "ca0": 0.3, "cna": 20.0, "cma": -1.5, "cmq": -60.0, "clp": -4.0
    })");
    // A finned airframe: same but with control derivatives.
    const json::Value finAero = json::Value::parse(R"({
        "sref_m2": 0.2, "lref_m": 8.0, "dref_m": 0.5,
        "ca0": 0.3, "cna": 20.0, "cma": -1.5, "cmq": -60.0, "clp": -4.0,
        "cnde": 1.5, "cmde": -8.0, "clda": 3.0
    })");

    // --- TVC controller on a fin-only vehicle: fails naming the gimbal ---
    {
        ChannelTable t;
        auto aero = RocketAero::fromJson(finAero);
        aero->declareChannels(t);
        ThrustEffector().declareChannels(t);
        auto ctl = TvcController::fromJson(json::Value::parse("{}"));
        CHECK(throwsMentioning([&] { ctl->bindChannels(t); }, "tvc_pitch"));
    }

    // --- Fin controller on a TVC-only vehicle: fails naming the fin ---
    {
        ChannelTable t;
        auto aero = RocketAero::fromJson(tvcAero);
        aero->declareChannels(t);                       // declares NOTHING
        TvcEffector(6.0, 0.1).declareChannels(t);
        auto ctl = RocketController::fromJson(json::Value::parse("{}"));
        CHECK(throwsMentioning([&] { ctl->bindChannels(t); }, "elevator"));
    }

    // --- Matched pairings bind cleanly ---
    {
        ChannelTable t;
        auto aero = RocketAero::fromJson(finAero);
        aero->declareChannels(t);
        auto ctl = RocketController::fromJson(json::Value::parse("{}"));
        for (const ChannelHandle h : ctl->bindChannels(t))
            (void)h;                                    // no throw is the check
        CHECK(t.find("elevator").valid());
        CHECK(t.find("aileron").valid());
        CHECK(!t.find("tvc_pitch").valid());
    }

    // --- End-to-end through the scenario loader (inline definition) ---
    {
        const std::string path = "output/_test_bad_pairing_scenario.json";
        std::ofstream(path) << R"({
            "simulation": {"dt_s": 0.01, "duration_s": 1},
            "vehicles": [{
                "name": "bad",
                "definition": {
                    "type": "rocket",
                    "mass_kg": 40, "inertia": {"ixx": 1, "iyy": 60, "izz": 60},
                    "aero": {"sref_m2": 0.2, "lref_m": 8.0, "dref_m": 0.5,
                             "ca0": 0.3, "cna": 20.0, "cma": -1.5,
                             "cmq": -60.0, "clp": -4.0},
                    "controller": {"method": "tvc"}
                }
            }]
        })";
        CHECK(throwsMentioning([&] { scenario::load(path); }, "tvc_pitch"));
    }

    std::printf("test_channelvalidation: all checks passed\n");
    return 0;
}
