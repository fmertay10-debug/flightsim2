#include "test_util.h"

#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "effector/ThrustEffector.h"
#include "effector/TvcEffector.h"
#include "math/Units.h"

int main() {
    State s;
    AirData air;

    // Nozzle at station 6 m, CG at 3.5 m -> moment arm 2.5 m; 8 deg gimbal limit.
    TvcEffector tvc(6.0, units::deg2rad(8.0));
    ChannelTable table;
    tvc.declareChannels(table);
    const ChannelHandle pitch = table.find("tvc_pitch");
    const ChannelHandle yaw   = table.find("tvc_yaw");
    CHECK(pitch.valid());
    CHECK(yaw.valid());
    const double T = 10000.0;
    const double arm = 6.0 - 3.5;

    // --- Axial baseline: ThrustEffector is pure +x force, no moment ---
    {
        ThrustEffector axial;
        EffectorContext ctx{ s, air, T, 3.5 };
        const Wrench w = axial.compute(ctx, ChannelValues(table));
        CHECK_NEAR(w.force.x, T, 1e-9);
        CHECK_NEAR(w.moment.x, 0.0, 1e-12);
        CHECK_NEAR(w.moment.y, 0.0, 1e-12);
        CHECK_NEAR(w.moment.z, 0.0, 1e-12);
    }

    // --- +tvc_pitch -> nose-UP moment (My>0), thrust deflected +z ---
    {
        EffectorContext ctx{ s, air, T, 3.5 };
        ChannelValues u(table);
        u.set(pitch, units::deg2rad(4.0));
        const Wrench w = tvc.compute(ctx, u);
        CHECK(w.moment.y > 0.0);
        CHECK(w.force.z > 0.0);
        CHECK_NEAR(w.moment.y, arm * T * std::sin(units::deg2rad(4.0)), 1e-6);
        CHECK_NEAR(w.moment.z, 0.0, 1e-9);
    }

    // --- +tvc_yaw -> nose-RIGHT moment (Mz>0) ---
    {
        EffectorContext ctx{ s, air, T, 3.5 };
        ChannelValues u(table);
        u.set(yaw, units::deg2rad(4.0));
        const Wrench w = tvc.compute(ctx, u);
        CHECK(w.moment.z > 0.0);
        CHECK(w.force.y < 0.0);
    }

    // --- Gimbal is clamped to the limit ---
    {
        EffectorContext ctx{ s, air, T, 3.5 };
        ChannelValues u(table);
        u.set(pitch, units::deg2rad(30.0));   // beyond 8 deg
        const Wrench w = tvc.compute(ctx, u);
        CHECK_NEAR(w.moment.y, arm * T * std::sin(units::deg2rad(8.0)), 1e-6);
    }

    // --- Moment scales with thrust; ZERO thrust -> no authority (burnout) ---
    {
        ChannelValues u(table);
        u.set(pitch, units::deg2rad(4.0));
        const Wrench big = tvc.compute(EffectorContext{ s, air, 20000.0, 3.5 }, u);
        const Wrench sml = tvc.compute(EffectorContext{ s, air, 10000.0, 3.5 }, u);
        CHECK_NEAR(big.moment.y, 2.0 * sml.moment.y, 1e-6);
        const Wrench none = tvc.compute(EffectorContext{ s, air, 0.0, 3.5 }, u);
        CHECK_NEAR(none.moment.y, 0.0, 1e-12);
        CHECK_NEAR(none.force.x, 0.0, 1e-12);
    }

    // --- Moment arm grows as the CG moves FORWARD (toward the nose) ---
    {
        ChannelValues u(table);
        u.set(pitch, units::deg2rad(4.0));
        const Wrench aft = tvc.compute(EffectorContext{ s, air, T, 4.0 }, u);  // arm 2.0
        const Wrench fwd = tvc.compute(EffectorContext{ s, air, T, 3.0 }, u);  // arm 3.0
        CHECK(fwd.moment.y > aft.moment.y);   // longer arm -> more moment
    }

    std::printf("test_tvc: all checks passed\n");
    return 0;
}
