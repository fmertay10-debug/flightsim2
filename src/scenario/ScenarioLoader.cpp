#include "scenario/ScenarioLoader.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <stdexcept>

#include "gnc/ActuatorBank.h"
#include "gnc/control/ControlLawFactory.h"
#include "dynamics/EomFactory.h"
#include "gnc/guidance/GuidanceFactory.h"
#include "io/Json.h"
#include "math/Units.h"
#include "sim/CsvLogger.h"
#include "vehicle/VehicleFactory.h"

namespace scenario {

namespace {

// --describe narration (set once by load(); loading is single-threaded).
bool gVerbose = false;

const char* kindName(ChannelKind k) {
    switch (k) {
        case ChannelKind::Surface:  return "surface";
        case ChannelKind::Gimbal:   return "gimbal";
        case ChannelKind::Throttle: return "throttle";
    }
    return "?";
}

std::unique_ptr<Environment> buildEnvironment(const json::Value& root) {
    std::unique_ptr<GravityModel> gravity = std::make_unique<FlatEarthGravity>();
    std::unique_ptr<WindModel>    wind    = std::make_unique<NoWind>();

    if (root.has("environment")) {
        const json::Value& env = root.at("environment");

        // "gravity" is either a name ("flat" | "spherical") or a parameterized
        // model: {"type": "inverse_square", "mu_m3_s2": ..., "radius_m": ...,
        // "j2": 0} (gravitation about a non-rotating sphere; NESC check-cases).
        if (env.has("gravity") && env.at("gravity").isObject()) {
            const json::Value& g = env.at("gravity");
            const std::string type = g.str("type");
            if (type == "inverse_square")
                gravity = std::make_unique<InverseSquareGravity>(
                    g.num("mu_m3_s2"), g.num("radius_m"), g.num("j2", 0.0));
            else
                throw std::invalid_argument(
                    "scenario: unknown gravity model type '" + type + "'");
        } else {
            const std::string g = env.str("gravity", "flat");
            if (g == "spherical")   gravity = std::make_unique<SphericalEarthGravity>();
            else if (g != "flat")
                throw std::invalid_argument("scenario: unknown gravity model '" + g + "'");
        }

        if (env.has("wind")) {
            const json::Value& w = env.at("wind");
            const std::string type = w.str("type", "none");
            if (type == "constant")
                wind = std::make_unique<ConstantWind>(w.vec3("ned_ms"));
            else if (type != "none")
                throw std::invalid_argument("scenario: unknown wind model '" + type + "'");
        }
    }
    return std::make_unique<Environment>(std::move(gravity), std::move(wind));
}

State buildInitialState(const json::Value& entry) {
    State s;
    if (!entry.has("initial")) return s;
    const json::Value& init = entry.at("initial");

    s.position = init.vec3("position_ned_m", {0, 0, 0});
    s.velocity = init.vec3("velocity_ned_ms", {0, 0, 0});

    const Vector3 eulerDeg = init.vec3("euler_deg", {0, 0, 0});
    s.attitude = Quaternion::fromEuler(units::deg2rad(eulerDeg.x),
                                       units::deg2rad(eulerDeg.y),
                                       units::deg2rad(eulerDeg.z));

    const Vector3 ratesDps = init.vec3("rates_dps", {0, 0, 0});
    s.angularRate = { units::deg2rad(ratesDps.x),
                      units::deg2rad(ratesDps.y),
                      units::deg2rad(ratesDps.z) };
    return s;
}

std::unique_ptr<Entity> buildEntity(const json::Value& entry,
                                    const std::filesystem::path& scenarioDir) {
    const std::string name = entry.str("name");
    const std::string dynamics = entry.str("dynamics", "six_dof");
    if (gVerbose)
        std::printf("entity '%s'  (dynamics: %s)\n", name.c_str(), dynamics.c_str());

    // Kinematic movers need no vehicle definition at all.
    std::unique_ptr<Vehicle>      veh;
    std::unique_ptr<ControlLaw>   controller;
    std::unique_ptr<ActuatorBank> actuators;
    ChannelTable channels;

    if (dynamics != "kinematic") {
        // Vehicle definition: referenced file or inline object. Relative data
        // paths inside the definition resolve against the file's directory
        // (the scenario's for inline definitions).
        json::Value definition;
        std::filesystem::path baseDir = scenarioDir;
        if (entry.has("definition")) {
            definition = entry.at("definition");
        } else {
            const std::filesystem::path ref = entry.str("vehicle");
            const std::filesystem::path path =
                ref.is_absolute() ? ref : scenarioDir / ref;
            definition = json::Value::parseFile(path.string());
            baseDir = path.parent_path();
            if (gVerbose)
                std::printf("  definition: %s\n  (relative data paths resolve in %s)\n",
                            path.string().c_str(), baseDir.string().c_str());
        }
        if (gVerbose && entry.has("definition"))
            std::printf("  definition: inline (relative data paths resolve in %s)\n",
                        baseDir.string().c_str());

        // Clean break: the pre-components schema is not supported.
        if (!definition.has("components"))
            throw std::invalid_argument(
                "vehicle definition for '" + name + "' has no \"components\" "
                "array. The old schema (top-level \"aero\"/\"propulsion\"/"
                "\"thrust_vectoring\"/\"controller\"/\"actuator\" blocks) was "
                "replaced: list aero and motors under \"components\" (each with "
                "an explicit \"type\") and put control_law/actuator under "
                "\"gnc\" -- see docs/BUILDING_VEHICLES.md.");

        veh = vehicle::create(definition, baseDir.string());

        // Channel phase A: force components DECLARE the channels they consume...
        veh->declareChannels(channels);

        if (gVerbose) {
            if (definition.has("mass"))
                std::printf("  mass model: %s\n",
                            definition.at("mass").str("model", "constant").c_str());
            std::printf("  components:");
            for (const std::string& n : veh->componentNames())
                std::printf(" %s", n.c_str());
            std::printf("\n  channels declared:");
            for (int i = 0; i < channels.size(); ++i) {
                const ChannelDef& d = channels.def(i);
                std::printf(" %s(%s %.3g..%.3g)", d.name.c_str(),
                            kindName(d.kind), d.minValue, d.maxValue);
            }
            std::printf("\n");
        }

        if (definition.has("gnc")) {
            const json::Value& gnc = definition.at("gnc");
            if (gnc.has("control_law")) {
                controller = gnc::Factory::create(gnc.at("control_law"),
                                                  baseDir.string());
                // Allocation-based laws query the components' effectiveness.
                controller->bindComponents(veh->components());
                if (gVerbose)
                    std::printf("  control law: %s%s\n",
                                gnc.at("control_law").str("type").c_str(),
                                controller->allocates()
                                    ? " (emits WrenchCommand via the Allocator)"
                                    : "");
            }
            if (gnc.has("actuator")) {
                actuators = ActuatorBank::fromJson(gnc.at("actuator"), channels);
                if (gVerbose)
                    std::printf("  actuators: per-channel servo dynamics attached\n");
            } else if (gVerbose && controller) {
                std::printf("  actuators: none (ideal: actual == commanded)\n");
            }
        }

        // ...phase B: the controller BINDS the channels it writes. A required
        // channel nothing declared throws here (misconfigured pairing);
        // declared channels no controller drives are only warned about --
        // they hold their default (zero).
        if (controller) {
            std::vector<bool> driven(static_cast<std::size_t>(channels.size()), false);
            for (const ChannelHandle h : controller->bindChannels(channels))
                if (h.valid()) driven[h.index] = true;
            if (gVerbose) {
                std::printf("  controller drives:");
                for (int i = 0; i < channels.size(); ++i)
                    if (driven[i])
                        std::printf(" %s", channels.def(i).name.c_str());
                std::printf("\n");
            }
            for (int i = 0; i < channels.size(); ++i)
                if (!driven[i])
                    std::fprintf(stderr,
                                 "warning: %s: channel '%s' is declared but the "
                                 "controller never writes it\n",
                                 name.c_str(), channels.def(i).name.c_str());
        }

        // Authority probe: a law that ALLOCATES depends on the components'
        // controlEffectiveness -- a declared Surface channel with no column
        // behind it would load fine and then fly open-loop (the allocator's
        // no-authority guard commands nothing). Probe once at a synthetic
        // healthy condition and fail LOUDLY at load instead. Gimbal channels
        // are exempt (authority is legitimately zero until the motor lights);
        // Throttle is not allocated.
        if (controller && controller->allocates() && veh) {
            State probeState;
            AirData probeAir;
            probeAir.airspeed = 100.0;
            probeAir.mach = 0.3;
            probeAir.qbar = 6000.0;
            probeAir.velocityBody = Vector3(100.0, 0.0, 0.0);
            const ComponentContext cctx{ probeState, probeAir, 1000.0, 0.0,
                                         std::nan("") };
            ControlEffect fx[ChannelTable::kMaxChannels];
            int n = 0;
            for (const auto& c : veh->components())
                n += c->controlEffectiveness(cctx, fx + n,
                                             ChannelTable::kMaxChannels - n);
            std::vector<bool> has(static_cast<std::size_t>(channels.size()), false);
            for (int k = 0; k < n; ++k) {
                const ControlEffect& e = fx[k];
                const bool nonzero =
                    e.dMoment.x != 0.0 || e.dMoment.y != 0.0 || e.dMoment.z != 0.0 ||
                    e.dForce.x  != 0.0 || e.dForce.y  != 0.0 || e.dForce.z  != 0.0;
                if (e.channel.valid() && nonzero) has[e.channel.index] = true;
            }
            for (int i = 0; i < channels.size(); ++i)
                if (channels.def(i).kind == ChannelKind::Surface && !has[i])
                    throw std::invalid_argument(
                        "vehicle '" + name + "': control law allocates, but no "
                        "component reports control effectiveness for surface "
                        "channel '" + channels.def(i).name + "' -- the aero "
                        "model must implement controlEffectiveness (see "
                        "ForceComponent.h) or not declare the channel");
            if (gVerbose)
                std::printf("  authority probe: OK (every surface channel has "
                            "an effectiveness column)\n");
        }
    }

    eom::Options opts;
    opts.turnRate = units::deg2rad(entry.num("turn_rate_dps", 0.0));

    FlightPlan plan;
    if (entry.has("flight_plan"))
        plan = FlightPlan::fromJson(entry.at("flight_plan"));

    return std::make_unique<Entity>(name,
                                    std::move(veh),
                                    std::move(controller),
                                    std::move(actuators),
                                    eom::create(dynamics, opts),
                                    std::move(plan),
                                    buildInitialState(entry),
                                    std::move(channels));
}

} // namespace

LoadResult load(const std::string& scenarioPath, bool verbose) {
    gVerbose = verbose;
    const json::Value root = json::Value::parseFile(scenarioPath);
    const std::filesystem::path scenarioDir =
        std::filesystem::path(scenarioPath).parent_path();

    SimConfig config;
    if (root.has("simulation")) {
        const json::Value& sim = root.at("simulation");
        config.dt          = sim.num("dt_s", config.dt);
        config.duration    = sim.num("duration_s", config.duration);
        config.groundLevel = sim.num("ground_level_m", config.groundLevel);
    }

    LoadResult result;
    result.name = root.str("name", std::filesystem::path(scenarioPath).stem().string());
    if (gVerbose)
        std::printf("scenario '%s'  (dt %g s, duration %g s)\n",
                    result.name.c_str(), config.dt, config.duration);
    result.simulation = std::make_unique<Simulation>(config, buildEnvironment(root));

    const json::Value& vehicles = root.at("vehicles");
    if (!vehicles.isArray() || vehicles.size() == 0)
        throw std::invalid_argument("scenario: 'vehicles' must be a non-empty array");

    // Pass 1: build every entity so names can be referenced regardless of order.
    std::map<std::string, int> idByName;
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        const json::Value& entry = vehicles[i];
        const int id = result.simulation->addEntity(buildEntity(entry, scenarioDir));
        idByName[entry.str("name")] = id;

        if (entry.has("log")) {
            const int decimation = static_cast<int>(entry.num("log_decimation", 1));
            result.simulation->addObserver(std::make_unique<CsvLogger>(
                id, entry.str("log"), decimation));
            if (gVerbose)
                std::printf("  log: %s (every %d steps)\n",
                            entry.str("log").c_str(), decimation);
        }
    }

    const auto resolveName = [&](const std::string& name) {
        const auto it = idByName.find(name);
        if (it == idByName.end())
            throw std::invalid_argument("scenario: unknown entity name '" + name + "'");
        return it->second;
    };

    // Pass 2: guidance (needs target ids).
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        const json::Value& entry = vehicles[i];
        if (!entry.has("guidance")) continue;
        const json::Value& g = entry.at("guidance");
        const int selfId   = resolveName(entry.str("name"));
        const int targetId = resolveName(g.str("target"));
        result.simulation->entity(selfId).setGuidance(guidance::create(g, targetId));
        if (gVerbose)
            std::printf("guidance: '%s' %s -> '%s'\n", entry.str("name").c_str(),
                        g.str("type", "?").c_str(), g.str("target").c_str());
    }

    // End conditions: intercept watch.
    if (root.has("end_conditions")) {
        const json::Value& ec = root.at("end_conditions");
        if (ec.has("intercept")) {
            const json::Value& ic = ec.at("intercept");
            result.simulation->watchIntercept(resolveName(ic.str("pursuer")),
                                              resolveName(ic.str("target")),
                                              ic.num("hit_radius_m", 5.0));
            if (gVerbose)
                std::printf("intercept watch: '%s' vs '%s', hit radius %g m\n",
                            ic.str("pursuer").c_str(), ic.str("target").c_str(),
                            ic.num("hit_radius_m", 5.0));
        }
    }
    return result;
}

} // namespace scenario
