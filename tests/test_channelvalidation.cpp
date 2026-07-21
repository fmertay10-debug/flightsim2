#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "component/aero/AircraftAero.h"
#include "component/ComponentFactory.h"
#include "component/propulsion/Propulsor.h"
#include "gnc/control/laws/AllocatedAttitudeLaw.h"
#include "gnc/control/laws/ScheduledLaw.h"
#include "io/Json.h"
#include "component/propulsion/SolidMotor.h"
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

static std::unique_ptr<Propulsor> motor(std::optional<Propulsor::Gimbal> gimbal) {
    return std::make_unique<Propulsor>(
        std::make_unique<SolidMotor>(
            std::vector<std::pair<double, double>>{{0.0, 5000.0}, {5.0, 0.0}},
            20.0, 0.0),
        gimbal);
}

// A broken-by-design component: declares an elevator but reports no control
// effectiveness -- the situation the loader's authority probe must catch when
// an allocating law is attached (pre-probe, this loaded fine and flew
// open-loop; the F-16 had exactly this failure mode before its aero model
// implemented controlEffectiveness).
struct DeadSurface : ForceComponent {
    ChannelHandle elevator_;
    void declareChannels(ChannelTable& t) override {
        elevator_ = t.add({channels::kElevator, ChannelKind::Surface, -0.4, 0.4});
    }
    Wrench computeWrench(const ComponentContext&, const ChannelValues&,
                         const double*) const override {
        return {};
    }
};

int main() {
    // A surface-less airframe: stability derivatives only, no control authority.
    const json::Value bareAero = json::Value::parse(R"({
        "sref_m2": 0.2, "cbar_m": 8.0, "bspan_m": 0.5,
        "cd0": 0.3, "cla": 20.0, "cma": -1.5, "cmq": -60.0, "clp": -4.0
    })");
    // A surfaced airframe: same but with control derivatives.
    const json::Value finAero = json::Value::parse(R"({
        "sref_m2": 0.2, "cbar_m": 8.0, "bspan_m": 0.5,
        "cd0": 0.3, "cla": 20.0, "cma": -1.5, "cmq": -60.0, "clp": -4.0,
        "clde": 1.5, "cmde": -8.0, "clda": 3.0, "cndr": -8.0
    })");

    // --- A fin-requiring law on a surface-less vehicle: fails naming the fin ---
    // (ScheduledLaw requires elevator+rudder; the bare aero declares nothing.)
    {
        ChannelTable t;
        auto aero = AircraftAero::fromJson(bareAero);
        aero->declareChannels(t);                      // declares NOTHING
        motor(Propulsor::Gimbal{6.0, 0.1})->declareChannels(t);
        ScheduledLaw ctl(ScheduledLaw::Config{});
        CHECK(throwsMentioning([&] { ctl.bindChannels(t); }, "elevator"));
    }

    // --- require() lists what IS declared when a channel is missing ---
    {
        ChannelTable t;
        auto aero = AircraftAero::fromJson(finAero);
        aero->declareChannels(t);
        motor(std::nullopt)->declareChannels(t);       // axial mount: no gimbal
        CHECK(throwsMentioning([&] { t.require(channels::kTvcPitch); },
                               "tvc_pitch"));
        CHECK(throwsMentioning([&] { t.require(channels::kTvcPitch); },
                               "elevator"));           // the listing
    }

    // --- Matched pairings bind cleanly ---
    {
        ChannelTable t;
        auto aero = AircraftAero::fromJson(finAero);
        aero->declareChannels(t);
        ScheduledLaw ctl(ScheduledLaw::Config{});
        for (const ChannelHandle h : ctl.bindChannels(t))
            (void)h;                                    // no throw is the check
        CHECK(t.find("elevator").valid());
        CHECK(t.find("aileron").valid());
        CHECK(!t.find("tvc_pitch").valid());
    }

    // --- End-to-end through the scenario loader (inline definition): an
    //     allocating law on an airframe with no control channels at all ---
    {
        const std::string path = "output/_test_bad_pairing_scenario.json";
        std::ofstream(path) << R"({
            "simulation": {"dt_s": 0.01, "duration_s": 1},
            "vehicles": [{
                "name": "bad",
                "definition": {
                    "mass": {"model": "constant", "mass_kg": 40,
                             "inertia": {"ixx": 1, "iyy": 60, "izz": 60}},
                    "components": [
                        {"type": "aircraft_aero",
                         "sref_m2": 0.2, "cbar_m": 8.0, "bspan_m": 0.5,
                         "cd0": 0.3, "cla": 20.0, "cma": -1.5,
                         "cmq": -60.0, "clp": -4.0}
                    ],
                    "gnc": { "control_law": {"type": "allocated_attitude"} }
                }
            }]
        })";
        CHECK(throwsMentioning([&] { scenario::load(path); }, "no control"));
    }

    // --- Authority probe: an allocating law over a surface channel with no
    //     effectiveness column fails at LOAD, naming the channel ---
    {
        component::Factory::registerComponent(
            "test_dead_surface",
            [](const json::Value&, const std::string&) {
                return std::make_unique<DeadSurface>();
            });
        const std::string path = "output/_test_dead_surface_scenario.json";
        std::ofstream(path) << R"({
            "simulation": {"dt_s": 0.01, "duration_s": 1},
            "vehicles": [{
                "name": "dead",
                "definition": {
                    "mass": {"model": "constant", "mass_kg": 40,
                             "inertia": {"ixx": 1, "iyy": 60, "izz": 60}},
                    "components": [ {"type": "test_dead_surface"} ],
                    "gnc": { "control_law": {"type": "allocated_attitude"} }
                }
            }]
        })";
        CHECK(throwsMentioning([&] { scenario::load(path); },
                               "control effectiveness"));
        CHECK(throwsMentioning([&] { scenario::load(path); }, "elevator"));
    }

    // --- The retired flat mass schema is rejected, pointing at the block ---
    {
        const std::string path = "output/_test_flat_mass_scenario.json";
        std::ofstream(path) << R"({
            "simulation": {"dt_s": 0.01, "duration_s": 1},
            "vehicles": [{
                "name": "legacy_mass",
                "definition": {
                    "mass_kg": 40, "inertia": {"ixx": 1, "iyy": 60, "izz": 60},
                    "components": [
                        {"type": "aircraft_aero",
                         "sref_m2": 0.2, "cbar_m": 8.0, "bspan_m": 0.5,
                         "cd0": 0.3, "cla": 20.0, "cma": -1.5,
                         "cmq": -60.0, "clp": -4.0}
                    ]
                }
            }]
        })";
        CHECK(throwsMentioning([&] { scenario::load(path); }, "mass"));
        CHECK(throwsMentioning([&] { scenario::load(path); },
                               "dry_plus_propellant"));
    }

    // --- The old schema is rejected with a pointer to the new one ---
    {
        const std::string path = "output/_test_old_schema_scenario.json";
        std::ofstream(path) << R"({
            "simulation": {"dt_s": 0.01, "duration_s": 1},
            "vehicles": [{
                "name": "legacy",
                "definition": {
                    "type": "rocket",
                    "mass_kg": 40, "inertia": {"ixx": 1, "iyy": 60, "izz": 60},
                    "aero": {"sref_m2": 0.2, "lref_m": 8.0, "dref_m": 0.5,
                             "ca0": 0.3, "cna": 20.0, "cma": -1.5}
                }
            }]
        })";
        CHECK(throwsMentioning([&] { scenario::load(path); }, "components"));
    }

    std::printf("test_channelvalidation: all checks passed\n");
    return 0;
}
