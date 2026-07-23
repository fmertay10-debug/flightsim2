#include "test_util.h"

#include <filesystem>
#include <fstream>
#include <string>

#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "io/Json.h"
#include "math/Units.h"
#include "vehicle/Vehicle.h"
#include "vehicle/VehicleFactory.h"

// Metamorphic invariants: physics symmetries the component stack must respect
// regardless of the numbers involved. These need no ground truth -- the same
// vehicle expressed two equivalent ways must produce the same wrench. The
// datum test is the one that catches station-handling bugs: stations share an
// arbitrary datum and only DIFFERENCES may matter (the retired NaN-xcg
// fallback in Propulsor violated exactly this: shifting the datum changed
// the gimbal arm).

namespace {

// A gimbaled solid-motor vehicle with every station shifted by `datum` and
// every arm scaled by `scale` (nozzle at 6*scale aft of datum, CG at
// 3.5*scale): arm = 2.5*scale regardless of datum.
std::unique_ptr<Vehicle> makeVehicle(double datum, double scale,
                                     const std::string& tag) {
    const double xcg    = datum + 3.5 * scale;
    const double nozzle = datum + 6.0 * scale;

    const std::string massCsv = "_test_inv_mass_" + tag + ".csv";
    std::ofstream("data/output/" + massCsv)
        << "time_s,mass_kg,ixx,iyy,izz,xcg_m\n"
        << "0,100,10,500,500," << xcg << "\n"
        << "6,100,10,500,500," << xcg << "\n";

    const json::Value def = json::Value::parse(R"({
        "mass": {"model": "tabulated", "table": ")" + massCsv + R"("},
        "components": [{
            "type": "solid_motor", "propellant_kg": 10,
            "thrust_curve": [[0, 10000], [4, 10000], [5, 0]],
            "gimbal": {"nozzle_station_m": )" + std::to_string(nozzle) + R"(,
                       "max_gimbal_deg": 8}
        }]
    })");
    return vehicle::create(def, "data/output");
}

// Total component wrench at mid-burn with a fixed gimbal command.
Wrench wrenchOf(Vehicle& v, ChannelTable& table) {
    v.declareChannels(table);
    ChannelValues u(table);
    u.set(table.find("tvc_pitch"), units::deg2rad(4.0));
    u.set(table.find("tvc_yaw"),   units::deg2rad(-3.0));

    State s;
    s.time = 2.0;                        // mid-burn: thrust = 10 kN exactly
    AirData air;                         // vacuum pad: no aero terms
    const ComponentContext ctx{ s, air, 0.0, 0.002, v.massState(2.0).xcg };

    Wrench total;
    for (const auto& c : v.components()) {
        const Wrench w = c->computeWrench(ctx, u, nullptr);
        total.force  = total.force + w.force;
        total.moment = total.moment + w.moment;
    }
    return total;
}

} // namespace

int main() {
    std::filesystem::create_directories("data/output");

    // --- Datum invariance: shift every station by +10 m -> same physics ---
    {
        auto base    = makeVehicle(0.0, 1.0, "base");
        auto shifted = makeVehicle(10.0, 1.0, "shift");
        ChannelTable tb, ts;
        const Wrench wb = wrenchOf(*base, tb);
        const Wrench ws = wrenchOf(*shifted, ts);
        CHECK_NEAR(wb.force.x,  ws.force.x,  1e-9);
        CHECK_NEAR(wb.force.y,  ws.force.y,  1e-9);
        CHECK_NEAR(wb.force.z,  ws.force.z,  1e-9);
        CHECK_NEAR(wb.moment.x, ws.moment.x, 1e-9);
        CHECK_NEAR(wb.moment.y, ws.moment.y, 1e-9);
        CHECK_NEAR(wb.moment.z, ws.moment.z, 1e-9);

        // Effectiveness columns must be datum-invariant too (the allocator's
        // view of the same physics). lastThrust_ is primed by wrenchOf above.
        State s; s.time = 2.0;
        AirData air;
        ControlEffect eb[8], es[8];
        const int nb = base->components()[0]->controlEffectiveness(
            { s, air, 0.0, 0.002, base->massState(2.0).xcg }, eb, 8);
        const int ns = shifted->components()[0]->controlEffectiveness(
            { s, air, 0.0, 0.002, shifted->massState(2.0).xcg }, es, 8);
        CHECK(nb == ns);
        for (int i = 0; i < nb; ++i) {
            CHECK_NEAR(eb[i].dMoment.x, es[i].dMoment.x, 1e-9);
            CHECK_NEAR(eb[i].dMoment.y, es[i].dMoment.y, 1e-9);
            CHECK_NEAR(eb[i].dMoment.z, es[i].dMoment.z, 1e-9);
        }
    }

    // --- Arm scaling: double every arm -> moments double, forces identical ---
    {
        auto base   = makeVehicle(0.0, 1.0, "s1");
        auto scaled = makeVehicle(0.0, 2.0, "s2");
        ChannelTable tb, ts;
        const Wrench w1 = wrenchOf(*base, tb);
        const Wrench w2 = wrenchOf(*scaled, ts);
        CHECK_NEAR(w2.force.x,  w1.force.x,        1e-9);
        CHECK_NEAR(w2.force.z,  w1.force.z,        1e-9);
        CHECK_NEAR(w2.moment.y, 2.0 * w1.moment.y, 1e-9);
        CHECK_NEAR(w2.moment.z, 2.0 * w1.moment.z, 1e-9);
        CHECK(std::abs(w1.moment.y) > 100.0);   // the moments are real, not 0==0
    }

    std::printf("test_physics_invariants: all checks passed\n");
    return 0;
}
