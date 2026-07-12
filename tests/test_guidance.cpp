#include "test_util.h"

#include <cmath>

#include "gnc/guidance/ProNav3D.h"
#include "gnc/guidance/PurePursuit.h"
#include "math/Units.h"

namespace {

WorldView makeWorld(const State& target) {
    WorldView w;
    w.add({0, "pursuer", State{}, true});
    w.add({1, "target", target, true});
    return w;
}

} // namespace

int main() {
    // --- Pure pursuit points at the target ---
    {
        PurePursuit pp(1);
        State self;
        self.position = {0, 0, -1000};
        State tgt;
        tgt.position = {1000, 1000, -2000};      // NE of us and higher

        const CommandSet cmd = pp.update(self, makeWorld(tgt), 0.01);
        CHECK(cmd.pitch && cmd.heading);
        CHECK_NEAR(*cmd.heading, units::deg2rad(45), 1e-9);
        CHECK_NEAR(*cmd.pitch, std::atan2(1000.0, std::hypot(1000.0, 1000.0)), 1e-9);
    }

    // --- ProNav: collision course -> zero LOS rate -> commands hold seed ---
    {
        ProNav3D pn(1, 3.0, units::deg2rad(40), units::deg2rad(60), 0.0);
        State self;
        self.position = {0, 0, -1000};
        self.velocity = {200, 0, 0};
        State tgt;
        tgt.position = {2000, 0, -1000};
        tgt.velocity = {-100, 0, 0};             // pure head-on

        CommandSet cmd;
        for (int i = 0; i < 100; ++i)
            cmd = pn.update(self, makeWorld(tgt), 0.01);
        CHECK_NEAR(*cmd.heading, 0.0, 1e-6);     // no correction needed
        CHECK_NEAR(*cmd.pitch, 0.0, 1e-6);
    }

    // --- ProNav: target crossing left-to-right -> heading command leads right ---
    {
        ProNav3D pn(1, 3.0, units::deg2rad(40), units::deg2rad(60), 0.0);
        State self;
        self.position = {0, 0, -1000};
        self.velocity = {200, 0, 0};
        State tgt;
        tgt.position = {2000, 0, -1000};
        tgt.velocity = {0, 80, 0};               // crossing toward +y (east)

        CommandSet cmd;
        for (int i = 0; i < 100; ++i)
            cmd = pn.update(self, makeWorld(tgt), 0.01);
        CHECK(*cmd.heading > units::deg2rad(1)); // turned toward the crossing
    }

    // --- ProNav: target climbing -> pitch command rises ---
    {
        ProNav3D pn(1, 3.0, units::deg2rad(40), units::deg2rad(60), 0.0);
        State self;
        self.position = {0, 0, -1000};
        self.velocity = {200, 0, 0};
        State tgt;
        tgt.position = {2000, 0, -1000};
        tgt.velocity = {0, 0, -40};              // climbing

        CommandSet cmd;
        for (int i = 0; i < 100; ++i)
            cmd = pn.update(self, makeWorld(tgt), 0.01);
        CHECK(*cmd.pitch > units::deg2rad(1));
    }

    std::printf("test_guidance: all checks passed\n");
    return 0;
}
