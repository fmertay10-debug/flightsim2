#include <cmath>
#include <memory>

#include "design/Linearizer.h"
#include "environment/Environment.h"
#include "io/Json.h"
#include "vehicle/VehicleFactory.h"
#include "test_util.h"

// The Linearizer must reproduce the ANALYTIC short-period derivatives when
// the airframe is the linear AircraftAero model -- ground truth the numerical
// differencing of the full component stack has to match. This is the
// end-to-end check of the ADR-0003 premise: the pure f(x,u) the sim flies is
// the same function the offline design linearizes.
//
// With cl0 = cd0 = k_induced = 0 the model reduces to Fz = -CL cos(alpha) qS,
// CL = cla*alpha + clde*de, Cm = cma*alpha + cmq*qhat + cmde*de -- every
// closed form below is exact. The Linearizer's +/-ha central difference of
// the alpha*cos(alpha) product is cos(ha) times the true slope EXACTLY
// ([h cos h + h cos h]/2h), so Za carries that factor; everything else is
// linear in the differenced variable and matches the slope directly.

int main() {
    const json::Value def = json::Value::parse(R"({
        "mass": { "model": "tabulated",
                  "table": "tests/fixtures/linearizer_mass.csv" },
        "components": [
            { "type": "aircraft_aero",
              "sref_m2": 0.049, "cbar_m": 3.0, "bspan_m": 0.25,
              "cd0": 0.0,
              "cla": 10.0,  "clde": 1.5,
              "cma": -12.0, "cmq": -120.0, "cmde": -8.0,
              "clp": -6.0,  "clda": 3.0 }
        ]
    })");
    auto veh = vehicle::create(def, ".");
    ChannelTable channels;
    veh->declareChannels(channels);

    const double alt = 1000.0, mach = 0.5;
    const double S = 0.049, c = 3.0, m = 85.0, Iyy = 320.0;
    const double cla = 10.0, clde = 1.5, cma = -12.0, cmq = -120.0, cmde = -8.0;
    const double ha = 0.0349;   // the Linearizer's alpha/deflection half-step

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

        const double Za = -qS * cla * std::cos(ha) / (m * V);   // see header note
        CHECK_NEAR(p.V, V, 1e-9);
        CHECK_NEAR(p.Za,  Za,                             1e-8 * std::abs(Za) + 1e-12);
        CHECK_NEAR(p.Zde, -qS * clde / (m * V),           1e-8 * std::abs(qS * clde / (m * V)) + 1e-12);
        CHECK_NEAR(p.Ma,   qS * c * cma / Iyy,            1e-8 * std::abs(qS * c * cma / Iyy));
        CHECK_NEAR(p.Mq,   qS * c * c * cmq / (2.0 * V * Iyy),
                   1e-8 * std::abs(qS * c * c * cmq / (2.0 * V * Iyy)));
        CHECK_NEAR(p.Mde,  qS * c * cmde / Iyy,           1e-8 * std::abs(qS * c * cmde / Iyy));
    }

    // --- Trim: lift carries the weight, moment balances. The body-z balance
    //     Fz + m g cos(a) = 0 with Fz = -CL cos(a) qS cancels the cos(a):
    //     CL qS = m g exactly, and cma*alpha + cmde*de = 0 ---
    {
        design::LinearizerOptions opt;
        opt.altitude = alt;
        opt.trim = true;
        const design::Linearizer lin(*veh, channels, opt);
        const design::PlantPoint p = lin.at(mach);
        CHECK(p.trimmed);

        const double g = env.gravity(alt);
        const double lhs = qS * (cla * p.alphaTrim + clde * p.deTrim);
        CHECK_NEAR(lhs, m * g, 1e-6 * m * g);
        CHECK_NEAR(cma * p.alphaTrim + cmde * p.deTrim, 0.0, 1e-8);
        CHECK(p.alphaTrim > 0.0);            // lifting up
        CHECK(p.deTrim < 0.0);               // stable airframe: fin holds nose up
    }

    std::printf("test_linearizer: all checks passed\n");
    return 0;
}
