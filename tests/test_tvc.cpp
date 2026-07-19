#include "test_util.h"

#include <memory>

#include "component/Propulsor.h"
#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "math/Units.h"

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

    std::printf("test_tvc: all checks passed\n");
    return 0;
}
