#include "test_util.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "io/CsvReader.h"
#include "io/Json.h"
#include "vehicle/Vehicle.h"
#include "vehicle/VehicleFactory.h"

// Pre-flight checkout of every SHIPPED vehicle, the software analogue of a
// hardware polarity test: load each data/vehicles definition and check its
// physics against the DATA FILES read independently -- not against anything
// the components expose. This is the correctness net the 2.25x gimbal-arm
// bug slipped through: code and controller shared the same wrong arm, so
// only an oracle derived straight from the JSON/CSV numbers could disagree.

namespace {

namespace fs = std::filesystem;

double lerp(const std::vector<std::pair<double, double>>& pts, double t) {
    if (t <= pts.front().first) return pts.front().second;
    for (std::size_t i = 1; i < pts.size(); ++i)
        if (t <= pts[i].first) {
            const double f = (t - pts[i - 1].first) /
                             (pts[i].first - pts[i - 1].first);
            return pts[i - 1].second +
                   f * (pts[i].second - pts[i - 1].second);
        }
    return pts.back().second;
}

struct Checked { int vehicles = 0, gimbals = 0, surfaces = 0, lints = 0; };

void checkVehicle(const fs::path& jsonPath, Checked& n) {
    const std::string dir = jsonPath.parent_path().string();
    const json::Value def = json::Value::parseFile(jsonPath.string());
    ++n.vehicles;

    // Every shipped vehicle must load under the current code.
    auto veh = vehicle::create(def, dir);
    ChannelTable table;
    veh->declareChannels(table);

    // Mass CSV, read independently of the mass model.
    const fs::path massCsv = fs::path(dir) / def.at("mass").str("table");
    const csv::Table mt = csv::read(massCsv.string());
    const bool hasXcg = mt.hasCol("xcg_m");
    double xcg0 = std::nan(""), xcgMin = 0.0, xcgMax = 0.0;
    if (hasXcg) {
        const std::size_t ci = mt.col("xcg_m");
        xcg0 = xcgMin = xcgMax = mt.rows.front()[ci];
        for (const auto& r : mt.rows) {
            xcgMin = std::min(xcgMin, r[ci]);
            xcgMax = std::max(xcgMax, r[ci]);
        }
    }
    const bool xcgConstant = hasXcg && (xcgMax - xcgMin) < 1e-9;

    // --- Lint: a constant CSV CG must agree with the geometry block's CG.
    // (A varying column is deliberate CG travel; geometry.xcg_m is then just
    // the reference, so equality is not required.)
    if (xcgConstant && def.has("geometry") && def.at("geometry").has("xcg_m")) {
        const double gx = def.at("geometry").num("xcg_m");
        if (std::abs(gx - xcg0) > 1e-6) {
            std::fprintf(stderr,
                "%s: geometry xcg_m %.6g != mass-table xcg_m %.6g\n",
                jsonPath.string().c_str(), gx, xcg0);
            CHECK(false);
        }
        ++n.lints;
    }

    const json::Value& comps = def.at("components");
    for (std::size_t i = 0; i < comps.size(); ++i) {
        const json::Value& c = comps[i];
        if (!c.has("gimbal")) continue;

        // --- Gimbal arm oracle: moment from a 4-deg pitch deflection at a
        // thrust-curve breakpoint must equal (nozzle - xcg) * T * sin(d),
        // every number taken from the DATA, none from the component. ---
        CHECK(hasXcg);   // the load gate guarantees this; re-assert on data
        const double nozzle = c.at("gimbal").num("nozzle_station_m");
        const double ign    = c.num("ignition_time_s", 0.0);
        std::vector<std::pair<double, double>> curve;
        const json::Value& pts = c.at("thrust_curve");
        for (std::size_t k = 0; k < pts.size(); ++k)
            curve.emplace_back(pts[k][0].asNumber(), pts[k][1].asNumber());
        const double tStar = ign + curve[1].first;      // exact breakpoint
        const double T     = lerp(curve, curve[1].first);

        const double xcgAtT = veh->massState(tStar).xcg;
        const double dp = 4.0 * M_PI / 180.0;
        const double expectMy = (nozzle - xcgAtT) * T * std::sin(dp);

        State s;
        s.time = tStar;
        AirData air;                       // qbar = 0: aero contributes nothing
        air.airspeed = 1.0;                // finite V for any damping division
        air.velocityBody = Vector3(1.0, 0.0, 0.0);
        const ComponentContext ctx{ s, air, 0.0, 0.002, xcgAtT };

        ChannelValues u(table);
        u.set(table.find("tvc_pitch"), dp);
        Wrench total;
        std::vector<double> x;             // externalized states, if any
        for (const auto& comp : veh->components()) {
            x.assign(std::max(comp->numStates(), 1), 0.0);
            comp->initializeState(x.data());
            const Wrench w = comp->computeWrench(ctx, u, x.data());
            total.force  = total.force + w.force;
            total.moment = total.moment + w.moment;
        }
        if (std::abs(total.moment.y - expectMy) > 1e-6 * std::abs(expectMy)) {
            std::fprintf(stderr,
                "%s: gimbal My %.6g != data-derived %.6g "
                "(nozzle %.4g, xcg %.4g, T %.6g)\n",
                jsonPath.string().c_str(), total.moment.y, expectMy,
                nozzle, xcgAtT, T);
            CHECK(false);
        }
        ++n.gimbals;
    }

    // --- Control polarity at a healthy flight condition (locked
    // conventions: +elevator nose DOWN, +rudder nose LEFT, +aileron right
    // roll -- except the F-16's documented inverted aileron; gimbals:
    // +tvc_pitch nose UP, +tvc_yaw nose RIGHT). ---
    const bool isF16 = jsonPath.filename() == "f16.json";
    State s;
    s.time = 0.1;
    AirData air;
    air.atmosphere.density = 0.9;
    air.airspeed = 280.0;
    air.mach     = 0.85;
    air.alpha    = 0.02;
    air.qbar     = 0.5 * 0.9 * 280.0 * 280.0;
    air.velocityBody = Vector3(280.0 * std::cos(0.02), 0.0,
                               280.0 * std::sin(0.02));
    const double xcgNow = veh->massState(0.1).xcg;
    const ComponentContext ctx{ s, air, 1000.0, 0.002, xcgNow };

    ChannelValues u(table);
    for (const auto& comp : veh->components()) {
        std::vector<double> x(std::max(comp->numStates(), 1), 0.0);
        comp->initializeState(x.data());
        comp->computeWrench(ctx, u, x.data());   // primes gimbal lastThrust
        ControlEffect eff[ChannelTable::kMaxChannels];
        const int ne = comp->controlEffectiveness(ctx, eff,
                                                  ChannelTable::kMaxChannels);
        for (int k = 0; k < ne; ++k) {
            const std::string& name = table.def(eff[k].channel.index).name;
            bool ok = true;
            if (name == "elevator")       ok = eff[k].dMoment.y < 0.0;
            else if (name == "rudder")    ok = eff[k].dMoment.z < 0.0;
            else if (name == "aileron")   ok = isF16 ? eff[k].dMoment.x < 0.0
                                                     : eff[k].dMoment.x > 0.0;
            else if (name == "tvc_pitch") ok = eff[k].dMoment.y > 0.0;
            else if (name == "tvc_yaw")   ok = eff[k].dMoment.z > 0.0;
            else continue;
            if (!ok) {
                std::fprintf(stderr, "%s: polarity of '%s' violates the "
                    "locked convention (dM = %.4g, %.4g, %.4g)\n",
                    jsonPath.string().c_str(), name.c_str(),
                    eff[k].dMoment.x, eff[k].dMoment.y, eff[k].dMoment.z);
                CHECK(false);
            }
            ++n.surfaces;
        }
    }
}

} // namespace

int main() {
    Checked n;
    std::vector<fs::path> defs;
    for (const auto& e : fs::directory_iterator("data/vehicles"))
        if (e.path().extension() == ".json") defs.push_back(e.path());
    for (const char* sub : { "data/vehicles/fleet", "data/vehicles/generated" })
        for (const auto& e : fs::directory_iterator(sub))
            if (fs::exists(e.path() / "vehicle.json"))
                defs.push_back(e.path() / "vehicle.json");

    for (const auto& p : defs) checkVehicle(p, n);

    // The sweep must actually be sweeping: silent skips would hollow it out.
    CHECK(n.vehicles >= 20);   // 2 top-level + 13 fleet + 7 generated today
    CHECK(n.gimbals  >= 3);    // atlas_tvc, triax_probe, vulcan_hybrid
    CHECK(n.surfaces >= 20);   // fins across the fleet + f16 + trainer
    CHECK(n.lints    >= 3);    // every constant-CG vehicle with a geometry CG

    std::printf("test_vehicle_checkout: %d vehicles, %d gimbal oracles, "
                "%d surface polarities, %d CG lints -- all checks passed\n",
                n.vehicles, n.gimbals, n.surfaces, n.lints);
    return 0;
}
