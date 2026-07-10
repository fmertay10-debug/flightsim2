#include <algorithm>
#include <cmath>

#include "gnc/ActuatorBank.h"
#include "core/Channel.h"
#include "test_util.h"

// Reference: the old FirstOrderActuator per-channel step, kept inline here so
// the bank stays locked to the historical behavior even though that class is
// gone. (Lag toward command, slew-rate limited, hard stop.)
static double refStep(double current, double commanded, double dt,
                      double tau, double rateLimit, double limit) {
    double rate = (commanded - current) / tau;
    rate = std::clamp(rate, -rateLimit, rateLimit);
    return std::clamp(current + rate * dt, -limit, limit);
}

int main() {
    const double tau = 0.05, rateLimit = 2.0, posLimit = 0.4;
    const double throttleTau = 0.5, gimbalLimit = 0.1;
    const double dt = 0.01;

    ChannelTable t;
    const ChannelHandle elev = t.add({"elevator", ChannelKind::Surface, -0.3, 0.3});
    const ChannelHandle gim  = t.add({"tvc_pitch", ChannelKind::Gimbal, -0.2, 0.2});
    const ChannelHandle thr  = t.add({"throttle", ChannelKind::Throttle, 0.0, 1.0});

    ActuatorBank bank(t, tau, rateLimit, posLimit, throttleTau, gimbalLimit);

    // Drive with a saturating step then a reversal; compare every substep
    // against the reference dynamics for each channel kind.
    double refElev = 0.0, refGim = 0.0, refThr = 0.0;
    for (int i = 0; i < 200; ++i) {
        const double cmd = (i < 100) ? 1.0 : -1.0;   // way past all limits
        ChannelValues u(t);
        u.set(elev, cmd);
        u.set(gim, cmd);
        u.set(thr, cmd * 0.8);

        const ChannelValues actual = bank.apply(u, dt);

        refElev = refStep(refElev, cmd, dt, tau, rateLimit, posLimit);
        refGim  = refStep(refGim,  cmd, dt, tau, rateLimit, gimbalLimit);
        // Throttle: command clamped to [0,1], no rate limit, clamped position.
        const double tCmd  = std::clamp(cmd * 0.8, 0.0, 1.0);
        const double tRate = (tCmd - refThr) / throttleTau;
        refThr = std::clamp(refThr + tRate * dt, 0.0, 1.0);

        CHECK_NEAR(actual.get(elev), refElev, 1e-15);
        CHECK_NEAR(actual.get(gim),  refGim,  1e-15);
        CHECK_NEAR(actual.get(thr),  refThr,  1e-15);
    }

    // The surface must have saturated at the CONFIG stop (0.4), not the
    // tighter declared channel limit (0.3) -- matching the old actuator.
    ChannelValues hold(t);
    hold.set(elev, 1.0);
    for (int i = 0; i < 500; ++i) bank.apply(hold, dt);
    CHECK_NEAR(bank.apply(hold, dt).get(elev), posLimit, 1e-12);

    std::printf("test_actuatorbank: all checks passed\n");
    return 0;
}
