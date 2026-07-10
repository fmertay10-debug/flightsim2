#include "effector/TvcEffector.h"

#include <algorithm>
#include <cmath>

Wrench TvcEffector::compute(const EffectorContext& ctx, const ControlInput& u) const {
    Wrench w;
    if (ctx.thrust <= 0.0) return w;   // no thrust -> no TVC authority

    const double dp = std::clamp(u.tvcPitch, -maxGimbal_, maxGimbal_);
    const double dy = std::clamp(u.tvcYaw,   -maxGimbal_, maxGimbal_);
    const double T  = ctx.thrust;

    // Gimbaled thrust vector in the body frame:
    //   Fx forward, Fz = T sin(dp) (pitch), Fy = -T cos(dp) sin(dy) (yaw).
    const double Fx = T * std::cos(dp) * std::cos(dy);
    const double Fy = -T * std::cos(dp) * std::sin(dy);
    const double Fz = T * std::sin(dp);
    w.force = Vector3(Fx, Fy, Fz);

    // Nozzle is aft of the CG by the moment arm L (body -x). Moment about CG
    // = r x F with r = (-L, 0, 0):  My = L*Fz (nose up), Mz = -L*Fy (nose right).
    const double L = std::isfinite(ctx.xcg) ? (nozzleStation_ - ctx.xcg)
                                            : nozzleStation_;
    w.moment = Vector3(0.0, L * Fz, -L * Fy);
    return w;
}
