#include "test_util.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "design/Linearizer.h"
#include "environment/Environment.h"
#include "io/CsvReader.h"
#include "io/Json.h"
#include "math/Quaternion.h"
#include "vehicle/Vehicle.h"
#include "vehicle/VehicleFactory.h"

// External validation against Stevens, Lewis & Johnson, "Aircraft Control
// and Simulation" 3rd ed.: level-flight trim and the short-period
// linearization at the book's reference condition (V_T = 502 ft/s, sea
// level, qbar = 300 psf). Oracles frozen from the book (2026-07-23):
//   - Table 3.6-3 (p.195): trimmed throttle / elevator / alpha at
//     xcg = 0.30, 0.35 (nominal), 0.38 cbar.
//   - Example 3.8-1 (p.206): longitudinal Jacobian at xcg = 0.3 cbar ->
//     short-period poles -1.2039 +/- j1.4922 (zeta 0.628).
// alpha/elevator come from the Linearizer's Newton trim; throttle from an
// Fx = 0 solve here (engine spool at its per-candidate equilibrium via the
// generic derivatives() seam). Our poles are the 2-state (alpha, q) short
// period INCLUDING the d(alpha_dot)/dq = 1 + Zq/V column (PlantPoint::Aq);
// the book's are the full 4-state longitudinal set -- residual difference
// is the V_T/theta coupling, well under the tolerances used here.

namespace {

constexpr double FT  = 0.3048;
constexpr double R2D = 180.0 / M_PI;
constexpr double V502 = 502.0 * FT;               // the S&L trim condition

// The same flight-condition construction the Linearizer uses, so every
// component sees exactly what it sees during trim.
struct TrimPoint {
    State s;
    AirData air;
};

TrimPoint pointAt(const Environment& env, double V, double alpha) {
    TrimPoint p;
    p.s.velocity = Vector3(V, 0.0, 0.0);
    p.s.attitude = Quaternion::fromEuler(0.0, alpha, 0.0);
    p.air.atmosphere = env.atmosphere(0.0);
    p.air.velocityBody = p.s.attitude.rotate(p.s.velocity);
    p.air.airspeed = p.air.velocityBody.norm();
    p.air.alpha = std::atan2(p.air.velocityBody.z, p.air.velocityBody.x);
    p.air.beta  = std::atan2(p.air.velocityBody.y, p.air.velocityBody.x);
    p.air.mach = p.air.airspeed / p.air.atmosphere.soundSpeed;
    p.air.qbar = 0.5 * p.air.atmosphere.density
               * p.air.airspeed * p.air.airspeed;
    return p;
}

// Equilibrium internal state of one component at fixed inputs: bisect each
// state on xdot = 0 (the F-16 spool's power rate is monotone through its
// fixed point; stateless components return an empty vector).
std::vector<double> equilibriumState(const ForceComponent& c,
                                     const ComponentContext& ctx,
                                     const ChannelValues& u) {
    const int ns = c.numStates();
    std::vector<double> x(static_cast<std::size_t>(std::max(ns, 1)), 0.0);
    c.initializeState(x.data());
    std::vector<double> xdot(x.size(), 0.0);
    for (int i = 0; i < ns; ++i) {
        double lo = 0.0, hi = 100.0;
        for (int it = 0; it < 60; ++it) {
            x[i] = 0.5 * (lo + hi);
            c.derivatives(ctx, u, x.data(), xdot.data());
            (xdot[i] > 0.0 ? lo : hi) = x[i];
        }
    }
    x.resize(static_cast<std::size_t>(ns));
    return x;
}

// Total body-x force from all components with the engine at its equilibrium
// spool for this throttle.
double bodyFx(Vehicle& veh, const ComponentContext& ctx,
              const ChannelValues& u) {
    double fx = 0.0;
    for (const auto& c : veh.components()) {
        const std::vector<double> x = equilibriumState(*c, ctx, u);
        fx += c->computeWrench(ctx, u, x.empty() ? nullptr : x.data()).force.x;
    }
    return fx;
}

} // namespace

int main() {
    const json::Value def = json::Value::parseFile("data/vehicles/f16.json");
    auto veh = vehicle::create(def, "data/vehicles");
    ChannelTable table;
    veh->declareChannels(table);
    const ChannelHandle elevator = table.find("elevator");
    const ChannelHandle throttle = table.find("throttle");
    CHECK(elevator.valid());
    CHECK(throttle.valid());

    const auto ref  = csv::readKeyValue("data/vehicles/f16/reference.csv");
    const double cbar = ref.at("cbar_m");
    const double mass = veh->massState(0.0).mass;

    const Environment env(std::make_unique<FlatEarthGravity>(),
                          std::make_unique<NoWind>());
    const double g  = env.gravity(0.0);
    const double a0 = env.atmosphere(0.0).soundSpeed;

    std::printf(
        "S&L level-flight trim + short period, V_T = 502 ft/s (%.4f m/s, "
        "M %.4f), sea level\n(signs: +el = S&L convention, trailing-edge "
        "down / nose-down)\n\n"
        "%-9s %8s %8s %10s %9s %9s %9s | %s\n",
        V502, V502 / a0, "xcg/cbar", "thtl", "el_deg", "alpha_deg",
        "Za_1/s", "Ma_1/s2", "Mq_1/s", "short-period poles [1/s]");

    // S&L 3rd ed. Table 3.6-3 (p.195): THTL (0-1), EL (deg), alpha (rad).
    struct BookCase {
        double frac, thtl, elDeg, alphaRad;
    };
    const BookCase cases[] = {
        { 0.30, 0.1485, -1.931,   0.03936 },
        { 0.35, 0.1385, -0.7588,  0.03691 },
        { 0.38, 0.1325, -0.05590, 0.03544 },
    };

    double prevMa = -1e9;
    for (const BookCase& bc : cases) {
        const double frac = bc.frac;
        design::LinearizerOptions opt;
        opt.altitude = 0.0;
        opt.trim     = true;
        opt.xcg      = frac * cbar;
        design::Linearizer lin(*veh, table, opt);
        const design::PlantPoint pp = lin.at(V502 / a0);
        CHECK(pp.trimmed);

        // Throttle from Fx = 0 at the trimmed attitude: aero + thrust
        // balance the gravity component m g sin(theta), theta = alpha.
        const TrimPoint tp = pointAt(env, V502, pp.alphaTrim);
        const ComponentContext ctx{ tp.s, tp.air, 0.0, 0.0, frac * cbar };
        ChannelValues u(table);
        u.set(elevator, pp.deTrim);
        double lo = 0.0, hi = 1.0, thtl = 0.0;
        for (int it = 0; it < 50; ++it) {
            thtl = 0.5 * (lo + hi);
            u.set(throttle, thtl);
            const double fx = bodyFx(*veh, ctx, u)
                            - mass * g * std::sin(pp.alphaTrim);
            (fx < 0.0 ? lo : hi) = thtl;     // more throttle -> more +x force
        }

        // Short-period poles of A = [[Za, Aq], [Ma, Mq]] -- Aq = d(alpha_dot)
        // /dq = 1 + Zq/V (about 0.905 for the F-16; assuming 1 inflates the
        // imaginary part by ~5%, exactly the gap vs the book's Example 3.8-1).
        const double tr = pp.Za + pp.Mq, det = pp.Za * pp.Mq - pp.Ma * pp.Aq;
        const double disc = 0.25 * tr * tr - det;
        char poles[96];
        if (disc < 0.0)
            std::snprintf(poles, sizeof poles,
                          "%.4f +/- %.4fi  (wn %.3f rad/s, zeta %.3f)",
                          0.5 * tr, std::sqrt(-disc), std::sqrt(det),
                          -0.5 * tr / std::sqrt(det));
        else
            std::snprintf(poles, sizeof poles, "%.4f, %.4f  (real%s)",
                          0.5 * tr + std::sqrt(disc), 0.5 * tr - std::sqrt(disc),
                          0.5 * tr + std::sqrt(disc) > 0.0 ? ", UNSTABLE" : "");
        std::printf("%-9.2f %8.4f %8.3f %10.3f %9.4f %9.4f %9.4f | %s\n",
                    frac, thtl, pp.deTrim * R2D, pp.alphaTrim * R2D,
                    pp.Za, pp.Ma, pp.Mq, poles);

        // --- The book's trim table, Table 3.6-3 (p.195) ---
        CHECK_NEAR(thtl,        bc.thtl,           1e-3);
        CHECK_NEAR(pp.deTrim * R2D, bc.elDeg,      1e-2);
        CHECK_NEAR(pp.alphaTrim,    bc.alphaRad,   4e-4);

        // --- The book's short-period poles, Example 3.8-1 (p.206): the
        // longitudinal Jacobian at xcg = 0.3 cbar -> -1.2039 +/- j1.4922 ---
        if (frac == 0.30) {
            CHECK(disc < 0.0);                       // oscillatory, as printed
            CHECK_NEAR(0.5 * tr,           -1.2039,  0.02);
            CHECK_NEAR(std::sqrt(-disc),    1.4922,  0.02);
        }

        CHECK(pp.Za < 0.0);
        CHECK(pp.Mq < 0.0);
        CHECK(pp.Ma > prevMa);        // aft cg must destabilize monotonically
        prevMa = pp.Ma;
    }

    std::printf("\ntest_f16_trim: all checks passed -- trim vs S&L 3rd ed. "
                "Table 3.6-3, short period vs Example 3.8-1\n");
    return 0;
}
