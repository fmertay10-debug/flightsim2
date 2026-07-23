#include "test_util.h"

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "component/propulsion/Propulsor.h"
#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "io/Json.h"
#include "math/Units.h"
#include "vehicle/VehicleFactory.h"

// Fixed-thrust stub so the mount math can be tested at chosen thrust levels.
struct FixedThrust : PropulsionModel {
    double T = 0.0;
    double thrustFromState(const PropulsionContext&, const double*) const override { return T; }
};

int main() {
    State s;
    AirData air;

    // Gimbaled propulsor: nozzle at station 6 m, CG at 3.5 m -> arm 2.5 m;
    // 8 deg gimbal limit.
    auto tvcModel = std::make_unique<FixedThrust>();
    FixedThrust* tvcThrust = tvcModel.get();
    Propulsor tvc(std::move(tvcModel),
                  Propulsor::Gimbal{ 6.0, units::deg2rad(8.0) });
    ChannelTable table;
    tvc.declareChannels(table);
    const ChannelHandle pitch = table.find("tvc_pitch");
    const ChannelHandle yaw   = table.find("tvc_yaw");
    CHECK(table.find("throttle").valid());   // motors own the demand channel
    CHECK(pitch.valid());
    CHECK(yaw.valid());
    const double T = 10000.0;
    const double arm = 6.0 - 3.5;
    const auto ctxAt = [&](double xcg) {
        return ComponentContext{ s, air, 0.0, 0.01, xcg };
    };

    // --- Axial baseline: ungimbaled propulsor is pure +x force, no moment ---
    {
        auto axModel = std::make_unique<FixedThrust>();
        axModel->T = T;
        Propulsor axial(std::move(axModel), std::nullopt);
        ChannelTable t2;
        axial.declareChannels(t2);
        const Wrench w = axial.computeWrench(ctxAt(3.5), ChannelValues(t2), nullptr);
        CHECK_NEAR(w.force.x, T, 1e-9);
        CHECK_NEAR(w.moment.x, 0.0, 1e-12);
        CHECK_NEAR(w.moment.y, 0.0, 1e-12);
        CHECK_NEAR(w.moment.z, 0.0, 1e-12);
        CHECK_NEAR(axial.thrustNewtons(), T, 1e-12);   // telemetry hook
    }

    // --- +tvc_pitch -> nose-UP moment (My>0), thrust deflected +z ---
    {
        tvcThrust->T = T;
        ChannelValues u(table);
        u.set(pitch, units::deg2rad(4.0));
        const Wrench w = tvc.computeWrench(ctxAt(3.5), u, nullptr);
        CHECK(w.moment.y > 0.0);
        CHECK(w.force.z > 0.0);
        CHECK_NEAR(w.moment.y, arm * T * std::sin(units::deg2rad(4.0)), 1e-6);
        CHECK_NEAR(w.moment.z, 0.0, 1e-9);
    }

    // --- +tvc_yaw -> nose-RIGHT moment (Mz>0) ---
    {
        ChannelValues u(table);
        u.set(yaw, units::deg2rad(4.0));
        const Wrench w = tvc.computeWrench(ctxAt(3.5), u, nullptr);
        CHECK(w.moment.z > 0.0);
        CHECK(w.force.y < 0.0);
    }

    // --- Gimbal is clamped to the limit ---
    {
        ChannelValues u(table);
        u.set(pitch, units::deg2rad(30.0));   // beyond 8 deg
        const Wrench w = tvc.computeWrench(ctxAt(3.5), u, nullptr);
        CHECK_NEAR(w.moment.y, arm * T * std::sin(units::deg2rad(8.0)), 1e-6);
    }

    // --- Moment scales with thrust; ZERO thrust -> no authority (burnout) ---
    {
        ChannelValues u(table);
        u.set(pitch, units::deg2rad(4.0));
        tvcThrust->T = 20000.0;
        const Wrench big = tvc.computeWrench(ctxAt(3.5), u, nullptr);
        tvcThrust->T = 10000.0;
        const Wrench sml = tvc.computeWrench(ctxAt(3.5), u, nullptr);
        CHECK_NEAR(big.moment.y, 2.0 * sml.moment.y, 1e-6);
        tvcThrust->T = 0.0;
        const Wrench none = tvc.computeWrench(ctxAt(3.5), u, nullptr);
        CHECK_NEAR(none.moment.y, 0.0, 1e-12);
        CHECK_NEAR(none.force.x, 0.0, 1e-12);
    }

    // --- Moment arm grows as the CG moves FORWARD (toward the nose) ---
    {
        tvcThrust->T = T;
        ChannelValues u(table);
        u.set(pitch, units::deg2rad(4.0));
        const Wrench aft = tvc.computeWrench(ctxAt(4.0), u, nullptr);   // arm 2.0
        const Wrench fwd = tvc.computeWrench(ctxAt(3.0), u, nullptr);   // arm 3.0
        CHECK(fwd.moment.y > aft.moment.y);   // longer arm -> more moment
    }

    // --- Load gate: a gimbaled vehicle whose mass table has no xcg_m column
    //     must be REJECTED (the arm nozzle_station - xcg cannot be formed;
    //     the retired NaN fallback silently used the nose as the CG) ---
    {
        std::filesystem::create_directories("data/output");
        std::ofstream("data/output/_test_tvc_mass_nocg.csv")
            << "time_s,mass_kg,ixx,iyy,izz\n0,100,10,500,500\n1,100,10,500,500\n";
        std::ofstream("data/output/_test_tvc_mass_cg.csv")
            << "time_s,mass_kg,ixx,iyy,izz,xcg_m\n"
               "0,100,10,500,500,2.5\n1,100,10,500,500,2.5\n";

        const auto gimbaledDef = [](const std::string& massCsv) {
            return json::Value::parse(R"({
                "mass": {"model": "tabulated", "table": ")" + massCsv + R"("},
                "components": [{
                    "type": "solid_motor", "propellant_kg": 10,
                    "thrust_curve": [[0, 5000], [5, 0]],
                    "gimbal": {"nozzle_station_m": 4.0, "max_gimbal_deg": 8}
                }]
            })");
        };

        bool threw = false;
        try {
            vehicle::create(gimbaledDef("_test_tvc_mass_nocg.csv"), "data/output");
        } catch (const std::exception& e) {
            threw = std::string(e.what()).find("xcg_m") != std::string::npos;
        }
        CHECK(threw);   // rejected, and the message names the missing column

        // Same vehicle WITH a CG column loads, and the CG reads back.
        const auto ok = vehicle::create(gimbaledDef("_test_tvc_mass_cg.csv"),
                                        "data/output");
        CHECK_NEAR(ok->massState(0.0).xcg, 2.5, 1e-12);

        // A present xcg_m column must be finite in EVERY row -- the load gate
        // samples t=0, so the table parser rejects mid-table NaN itself.
        std::ofstream("data/output/_test_tvc_mass_nancg.csv")
            << "time_s,mass_kg,ixx,iyy,izz,xcg_m\n"
               "0,100,10,500,500,2.5\n1,100,10,500,500,nan\n";
        bool threwNan = false;
        try {
            vehicle::create(gimbaledDef("_test_tvc_mass_nancg.csv"),
                            "data/output");
        } catch (const std::exception& e) {
            threwNan = std::string(e.what()).find("xcg_m") != std::string::npos;
        }
        CHECK(threwNan);

        // An AXIAL motor without xcg_m is still fine (thrust through the CG).
        const json::Value axialDef = json::Value::parse(R"({
            "mass": {"model": "tabulated", "table": "_test_tvc_mass_nocg.csv"},
            "components": [{
                "type": "solid_motor", "propellant_kg": 10,
                "thrust_curve": [[0, 5000], [5, 0]]
            }]
        })");
        CHECK(vehicle::create(axialDef, "data/output") != nullptr);
    }

    std::printf("test_tvc: all checks passed\n");
    return 0;
}
