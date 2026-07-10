#include "test_util.h"

#include <cmath>

#include "aero/F16Aero.h"
#include "io/CsvReader.h"
#include "io/Json.h"

// Validate the ported F-16 buildup against the reference fixture
// (tests/fixtures/f16_coeff_checks.csv), which lists the full 6-coefficient
// output (CXT..CNT) at sample flight conditions. Runs with the repo root as
// CWD. Recovers coefficients from the returned dimensional loads.
int main() {
    const json::Value cfg = json::Value::parse(R"({"dir": "vehicles/f16"})");
    const auto aero = F16Aero::fromJson(cfg, ".");

    ChannelTable table;
    aero->declareChannels(table);
    const ChannelHandle elevator = table.find("elevator");
    const ChannelHandle aileron  = table.find("aileron");
    const ChannelHandle rudder   = table.find("rudder");
    CHECK(elevator.valid());
    CHECK(aileron.valid());
    CHECK(rudder.valid());

    const auto ref = csv::readKeyValue("vehicles/f16/reference.csv");
    const double S = ref.at("sref_m2"), b = ref.at("bref_m"), c = ref.at("cbar_m");

    const csv::Table fx = csv::read("tests/fixtures/f16_coeff_checks.csv");
    const std::size_t ai = fx.col("alpha_rad"), bi = fx.col("beta_rad"),
        ei = fx.col("el_rad"), li = fx.col("ail_rad"), ri = fx.col("rdr_rad"),
        pi = fx.col("p_radps"), qi = fx.col("q_radps"), rri = fx.col("r_radps"),
        vi = fx.col("vt_mps"),
        cx = fx.col("CXT"), cy = fx.col("CYT"), cz = fx.col("CZT"),
        cl = fx.col("CLT"), cm = fx.col("CMT"), cn = fx.col("CNT");

    int rows = 0;
    for (const auto& row : fx.rows) {
        const double alpha = row[ai], beta = row[bi], V = row[vi];

        // Wind-to-body velocity consistent with (alpha, beta, V).
        const double u = V * std::cos(alpha) * std::cos(beta);
        const double v = V * std::sin(beta);
        const double w = V * std::sin(alpha) * std::cos(beta);

        State s;
        s.angularRate = { row[pi], row[qi], row[rri] };

        AirData air;
        air.velocityBody = { u, v, w };
        air.airspeed = V;
        air.alpha = std::atan2(w, u);
        air.mach = 0.0;
        const double rho = 1.0;                    // any rho: coefficients cancel it
        air.qbar = 0.5 * rho * V * V;
        air.atmosphere.density = rho;
        air.atmosphere.soundSpeed = 340.0;

        ChannelValues ctl(table);
        ctl.set(elevator, row[ei]);
        ctl.set(aileron,  row[li]);
        ctl.set(rudder,   row[ri]);

        const AeroForces f = aero->compute(s, air, ctl);
        const double qS = air.qbar * S;

        // Recover coefficients and compare to the fixture.
        CHECK_NEAR(f.force.x / qS,       row[cx], 2e-4);
        CHECK_NEAR(f.force.y / qS,       row[cy], 2e-4);
        CHECK_NEAR(f.force.z / qS,       row[cz], 2e-4);
        CHECK_NEAR(f.moment.x / (qS * b), row[cl], 2e-4);
        CHECK_NEAR(f.moment.y / (qS * c), row[cm], 2e-4);
        CHECK_NEAR(f.moment.z / (qS * b), row[cn], 2e-4);
        ++rows;
    }
    CHECK(rows >= 3);

    std::printf("test_f16aero: all %d fixture rows matched\n", rows);
    return 0;
}
