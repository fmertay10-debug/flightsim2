#include "component/Propulsor.h"

#include <algorithm>
#include <cmath>

void Propulsor::declareChannels(ChannelTable& table) {
    if (model_->hasThrottleChannel())
        throttle_ = table.add({channels::kThrottle, ChannelKind::Throttle, 0.0, 1.0});
    if (gimbal_) {
        tvcPitch_ = table.add({channels::kTvcPitch, ChannelKind::Gimbal,
                               -gimbal_->maxGimbal, gimbal_->maxGimbal});
        tvcYaw_   = table.add({channels::kTvcYaw, ChannelKind::Gimbal,
                               -gimbal_->maxGimbal, gimbal_->maxGimbal});
    }
}

Wrench Propulsor::compute(const ComponentContext& ctx, const ChannelValues& u) {
    PropulsionContext pc;
    pc.time     = ctx.state.time;
    pc.throttle = u.get(throttle_);
    pc.mach     = ctx.air.mach;
    pc.density  = ctx.air.atmosphere.density;
    pc.altitude = ctx.altitude;
    pc.dt       = ctx.dt;
    lastThrust_ = model_->thrust(pc);

    Wrench w;
    if (!gimbal_) {
        // Axial mount: thrust along body +x through the CG, no moment.
        w.force = Vector3(lastThrust_, 0.0, 0.0);
        return w;
    }

    if (lastThrust_ <= 0.0) return w;   // no thrust -> no TVC authority

    const double dp = std::clamp(u.get(tvcPitch_), -gimbal_->maxGimbal, gimbal_->maxGimbal);
    const double dy = std::clamp(u.get(tvcYaw_),   -gimbal_->maxGimbal, gimbal_->maxGimbal);
    const double T  = lastThrust_;

    // Gimbaled thrust vector in the body frame:
    //   Fx forward, Fz = T sin(dp) (pitch), Fy = -T cos(dp) sin(dy) (yaw).
    const double Fx = T * std::cos(dp) * std::cos(dy);
    const double Fy = -T * std::cos(dp) * std::sin(dy);
    const double Fz = T * std::sin(dp);
    w.force = Vector3(Fx, Fy, Fz);

    // Nozzle is aft of the CG by the moment arm L (body -x). Moment about CG
    // = r x F with r = (-L, 0, 0):  My = L*Fz (nose up), Mz = -L*Fy (nose right).
    const double L = std::isfinite(ctx.xcg) ? (gimbal_->nozzleStation - ctx.xcg)
                                            : gimbal_->nozzleStation;
    w.moment = Vector3(0.0, L * Fz, -L * Fy);
    return w;
}

int Propulsor::controlEffectiveness(const ComponentContext& ctx,
                                    ControlEffect* out, int maxOut) const {
    if (!gimbal_ || lastThrust_ <= 0.0) return 0;
    const double L = std::isfinite(ctx.xcg) ? (gimbal_->nozzleStation - ctx.xcg)
                                            : gimbal_->nozzleStation;
    const double LT = L * lastThrust_;
    int n = 0;
    if (tvcPitch_.valid() && n < maxOut)
        out[n++] = { tvcPitch_, Vector3(0.0, LT, 0.0) };
    if (tvcYaw_.valid() && n < maxOut)
        out[n++] = { tvcYaw_, Vector3(0.0, 0.0, LT) };
    return n;
}
