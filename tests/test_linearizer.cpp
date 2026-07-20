#include <cmath>
#include <memory>

#include "design/Linearizer.h"
#include "environment/Environment.h"
#include "io/Json.h"
#include "vehicle/VehicleFactory.h"
#include "test_util.h"

// The Linearizer must reproduce the ANALYTIC short-period derivatives when
// the airframe is the linear RocketAero model -- ground truth the numerical
// differencing of the full component stack has to match. This is the
// end-to-end check of the ADR-0003 premise: the pure f(x,u) the sim flies is
// the same function the offline design linearizes.

int main() {
    const json::Value def = json::Value::parse(R"({
        "mass": { "model": "constant", "mass_kg": 85,
                  "inertia": { "ixx": 2, "iyy": 320, "izz": 320 } },
        "components": [
            { "type": "rocket_aero",
              "sref_m2": 0.049, "lref_m": 3.0, "dref_m": 0.25,
              "ca0": 0.35,
              "cna": 10.0,  "cnde": 1.5,
              "cma": -12.0, "cmq": -120.0, "cmde": -8.0,
              "clp": -6.0,  "clda": 3.0 }
        ]
    })");
    auto veh = vehicle::create(def, ".");
    ChannelTable channels;
    veh->declareChannels(channels);

    const double alt = 1000.0, mach = 0.5;
    const double S = 0.049, l = 3.0, m = 85.0, Iyy = 320.0;
    const double cna = 10.0, cnde = 1.5, cma = -12.0, cmq = -120.0, cmde = -8.0;

    const Environment env(std::make_unique<FlatEarthGravity>(),
                          std::make_unique<NoWind>());
    const AtmosphereState atm = env.atmosphere(alt);
    const double V = mach * atm.soundSpeed;
    const double qS = 0.5 * atm.density * V * V * S;

    // --- Untrimmed point matches the closed forms exactly (linear model) ---
    {
        design::LinearizerOptions opt;
        opt.altitude = alt;
        const design::Linearizer lin(*veh, channels, opt);
        const design::PlantPoint p = lin.at(mach);

        CHECK_NEAR(p.V, V, 1e-9);
        CHECK_NEAR(p.Za,  -qS * cna / (m * V),            1e-8 * std::abs(qS * cna / (m * V)) + 1e-12);
        CHECK_NEAR(p.Zde, -qS * cnde / (m * V),           1e-8 * std::abs(qS * cnde / (m * V)) + 1e-12);
        CHECK_NEAR(p.Ma,   qS * l * cma / Iyy,            1e-8 * std::abs(qS * l * cma / Iyy));
        CHECK_NEAR(p.Mq,   qS * l * l * cmq / (2.0 * V * Iyy),
                   1e-8 * std::abs(qS * l * l * cmq / (2.0 * V * Iyy)));
        CHECK_NEAR(p.Mde,  qS * l * cmde / Iyy,           1e-8 * std::abs(qS * l * cmde / Iyy));
    }

    // --- Trim: lift carries the weight, moment balances; both closed-form
    //     for the linear airframe: alpha = m g cos(a)/(qS cna) (approx),
    //     de = -(cma/cmde) * alpha ---
    {
        design::LinearizerOptions opt;
        opt.altitude = alt;
        opt.trim = true;
        const design::Linearizer lin(*veh, channels, opt);
        const design::PlantPoint p = lin.at(mach);
        CHECK(p.trimmed);

        // Exact residual re-check from the closed forms: CN(alpha,de)*qS =
        // m g cos(alpha) and cma*alpha + cmde*de = 0.
        const double g = env.gravity(alt);
        const double lhs = qS * (cna * p.alphaTrim + cnde * p.deTrim);
        CHECK_NEAR(lhs, m * g * std::cos(p.alphaTrim), 1e-6 * m * g);
        CHECK_NEAR(cma * p.alphaTrim + cmde * p.deTrim, 0.0, 1e-8);
        CHECK(p.alphaTrim > 0.0);            // lifting up
        CHECK(p.deTrim < 0.0);               // stable airframe: fin holds nose up
    }

    std::printf("test_linearizer: all checks passed\n");
    return 0;
}
