#include "scenario/ScenarioLoader.h"

#include <cstdio>
#include <filesystem>
#include <map>
#include <stdexcept>

#include "gnc/ActuatorBank.h"
#include "gnc/ControlLawFactory.h"
#include "dynamics/EomFactory.h"
#include "gnc/GuidanceFactory.h"
#include "io/Json.h"
#include "math/Units.h"
#include "sim/CsvLogger.h"
#include "vehicle/VehicleFactory.h"

namespace scenario {

namespace {

std::unique_ptr<Environment> buildEnvironment(const json::Value& root) {
    std::unique_ptr<GravityModel> gravity = std::make_unique<FlatEarthGravity>();
    std::unique_ptr<WindModel>    wind    = std::make_unique<NoWind>();

    if (root.has("environment")) {
        const json::Value& env = root.at("environment");

        const std::string g = env.str("gravity", "flat");
        if (g == "spherical")   gravity = std::make_unique<SphericalEarthGravity>();
        else if (g != "flat")
            throw std::invalid_argument("scenario: unknown gravity model '" + g + "'");

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
        }

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

        if (definition.has("gnc")) {
            const json::Value& gnc = definition.at("gnc");
            if (gnc.has("control_law")) {
                controller = gnc::Factory::create(gnc.at("control_law"),
                                                  baseDir.string());
                // Allocation-based laws query the components' effectiveness.
                controller->bindComponents(veh->components());
            }
            if (gnc.has("actuator"))
                actuators = ActuatorBank::fromJson(gnc.at("actuator"), channels);
        }

        // ...phase B: the controller BINDS the channels it writes. A required
        // channel nothing declared throws here (misconfigured pairing);
        // declared channels no controller drives are only warned about --
        // they hold their default (zero).
        if (controller) {
            std::vector<bool> driven(static_cast<std::size_t>(channels.size()), false);
            for (const ChannelHandle h : controller->bindChannels(channels))
                if (h.valid()) driven[h.index] = true;
            for (int i = 0; i < channels.size(); ++i)
                if (!driven[i])
                    std::fprintf(stderr,
                                 "warning: %s: channel '%s' is declared but the "
                                 "controller never writes it\n",
                                 name.c_str(), channels.def(i).name.c_str());
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

LoadResult load(const std::string& scenarioPath) {
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
    }

    // End conditions: intercept watch.
    if (root.has("end_conditions")) {
        const json::Value& ec = root.at("end_conditions");
        if (ec.has("intercept")) {
            const json::Value& ic = ec.at("intercept");
            result.simulation->watchIntercept(resolveName(ic.str("pursuer")),
                                              resolveName(ic.str("target")),
                                              ic.num("hit_radius_m", 5.0));
        }
    }
    return result;
}

} // namespace scenario
