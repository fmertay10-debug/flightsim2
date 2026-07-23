#pragma once

#include <memory>

#include "component/ForceComponent.h"
#include "io/Json.h"

// Attitude-independent drag for bluff bodies (spheres, debris, the NESC
// check-case spheroid): F = -qbar * S * CD * v_hat, antiparallel to the
// relative wind in ANY orientation. Derivative aero builds drag in stability
// axes and is only meaningful near small alpha/beta; a tumbling body needs
// the direction to come from the wind vector itself. No moments (the force
// acts through the CM), no channels.
class BallisticDrag : public ForceComponent {
public:
    BallisticDrag(double sref, double cd) : sref_(sref), cd_(cd) {}

    static std::unique_ptr<BallisticDrag> fromJson(const json::Value& cfg) {
        return std::make_unique<BallisticDrag>(cfg.num("sref_m2"),
                                               cfg.num("cd"));
    }

    Wrench computeWrench(const ComponentContext& ctx, const ChannelValues&,
                         const double*) const override {
        Wrench w;
        const double V = ctx.air.airspeed;
        if (V < 1e-9) return w;
        // -qbar*S*CD * (v/V) = -(1/2) rho V S CD * v
        const double k = -0.5 * ctx.air.atmosphere.density * V * sref_ * cd_;
        w.force = ctx.air.velocityBody * k;
        return w;
    }

private:
    double sref_, cd_;
};
