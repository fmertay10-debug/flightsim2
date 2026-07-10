#include "test_util.h"

#include "models/rocket/RocketTableAero.h"
#include "io/CsvReader.h"
#include "io/Json.h"

// Loads the DATCOM-generated rocket tables (vehicles/generated/datcom_rocket,
// produced by tools/datcom_export.py) and checks the physical sign
// conventions of the assembled body-frame loads.
int main() {
    const std::string dir = "vehicles/generated/datcom_rocket";
    const json::Value cfg = json::Value::parse(R"({
        "sref_m2": 0.129693, "cbar_m": 8.2296, "bref_m": 0.5182,
        "tables_csv": "aero_tables.csv", "control_csv": "control_tables.csv"
    })");
    const auto aero = RocketTableAero::fromJson(cfg, dir);

    ChannelTable table;
    aero->declareChannels(table);
    const ChannelHandle elevator = table.find("elevator");
    const ChannelHandle rudder   = table.find("rudder");
    CHECK(elevator.valid());
    CHECK(rudder.valid());
    CHECK(table.find("aileron").valid());

    State s;
    AirData air;
    air.atmosphere.density = 1.225;
    air.atmosphere.soundSpeed = 340.0;
    air.airspeed = 272.0;                  // Mach 0.8
    air.mach = 0.8;
    air.qbar = 0.5 * 1.225 * 272.0 * 272.0;

    // --- Positive alpha: normal force UP (-z), pitch moment restoring (<0) ---
    air.alpha = 0.05;
    air.beta = 0.0;
    ChannelValues u(table);
    AeroForces f = aero->compute(s, air, u);
    const double qS = air.qbar * 0.129693;
    CHECK(f.force.z < 0.0);                // lift opposes alpha
    CHECK(f.force.x < 0.0);                // drag opposes motion
    CHECK(f.moment.y < 0.0);               // statically stable: nose-down
    // DATCOM's control tables carry ~1e-3 coefficient noise at delta = 0, so
    // the off-plane loads are only zero to that level, not exactly.
    CHECK_NEAR(f.force.y, 0.0, 0.005 * qS);
    CHECK_NEAR(f.moment.z, 0.0, 0.005 * qS * 8.2296);

    // --- Positive beta mirrors: side force LEFT, weathercock yaw RIGHT ---
    air.alpha = 0.0;
    air.beta = 0.05;
    f = aero->compute(s, air, u);
    CHECK(f.force.y < 0.0);                // crossflow pushes -y
    CHECK(f.moment.z > 0.0);               // restoring: nose toward velocity

    // --- Positive elevator: fin lift joins CN, moment pitches DOWN ---
    air.beta = 0.0;
    ChannelValues uDown = u;
    uDown.set(elevator, 0.15);
    const AeroForces f0 = aero->compute(s, air, u);
    const AeroForces fE = aero->compute(s, air, uDown);
    CHECK(fE.moment.y < f0.moment.y);      // dCM < 0 for +deflection
    CHECK(fE.force.z < f0.force.z);        // fin lift adds upward force

    // --- Rudder mirrors elevator through the cbar/bref conversion ---
    ChannelValues uR = u;
    uR.set(rudder, 0.15);
    const AeroForces fR = aero->compute(s, air, uR);
    CHECK(fR.moment.z < f0.moment.z);      // +rudder -> nose-left moment
    const double c2b = 8.2296 / 0.5182;
    CHECK_NEAR(fR.moment.z * 8.2296,       // same physical moment as pitch:
               fE.moment.y * 0.5182 * c2b, // Mz*cbar == My*bref*(cbar/bref)
               std::abs(fE.moment.y) * 0.05);

    // --- Pitch damping: positive q resists (more negative My) ---
    State sq = s;
    sq.angularRate.y = 0.5;
    const AeroForces fQ = aero->compute(sq, air, u);
    CHECK(fQ.moment.y < f0.moment.y);

    std::printf("test_tableaero: all checks passed\n");
    return 0;
}
