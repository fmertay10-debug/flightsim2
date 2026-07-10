#include "gnc/AllocatedAttitudeLaw.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "component/ForceComponent.h"
#include "math/Units.h"

std::unique_ptr<ControlLaw> AllocatedAttitudeLaw::fromJson(const json::Value& cfg) {
    Gains g;
    if (cfg.has("gains")) {
        const json::Value& j = cfg.at("gains");
        g.pitchKp = j.num("pitch_kp", g.pitchKp);
        g.pitchKd = j.num("pitch_kd", g.pitchKd);
        g.pitchKi = j.num("pitch_ki", g.pitchKi);
        g.yawKp   = j.num("yaw_kp", g.yawKp);
        g.yawKd   = j.num("yaw_kd", g.yawKd);
        g.yawKi   = j.num("yaw_ki", g.yawKi);
        g.rollKp  = j.num("roll_kp", g.rollKp);
        g.rollKd  = j.num("roll_kd", g.rollKd);
    }
    double damping = 1e-6;
    if (cfg.has("limits")) {
        const json::Value& j = cfg.at("limits");
        g.maxAngAccel = units::deg2rad(
            j.num("max_ang_accel_dps2", units::rad2deg(g.maxAngAccel)));
        g.intLimit      = j.num("int_limit", g.intLimit);
        g.verticalGuard = units::deg2rad(j.num("vertical_guard_deg",
                                               units::rad2deg(g.verticalGuard)));
        damping = j.num("alloc_damping", damping);
    }
    return std::make_unique<AllocatedAttitudeLaw>(g, damping);
}

std::vector<ChannelHandle> AllocatedAttitudeLaw::bindChannels(const ChannelTable& t) {
    table_ = t;   // allocation needs the declared channel limits per step
    throttle_ = t.find(channels::kThrottle);

    // The allocator writes whichever control channels exist; validate that at
    // least one pitch/yaw moment producer is declared at all.
    std::vector<ChannelHandle> bound{ throttle_ };
    bool any = false;
    for (const char* name : { channels::kElevator, channels::kAileron,
                              channels::kRudder, channels::kTvcPitch,
                              channels::kTvcYaw }) {
        const ChannelHandle h = t.find(name);
        if (h.valid()) any = true;
        bound.push_back(h);
    }
    if (!any)
        throw std::invalid_argument(
            "allocated_attitude: the vehicle declares no control channels to "
            "allocate over (no fins with control derivatives, no gimbal)");
    return bound;
}

void AllocatedAttitudeLaw::bindComponents(
    const std::vector<std::unique_ptr<ForceComponent>>& components) {
    components_.clear();
    for (const auto& c : components) components_.push_back(c.get());
}

void AllocatedAttitudeLaw::update(const GncContext& gc, const CommandSet& cmd,
                                  ChannelValues& out) {
    out.set(throttle_, cmd.throttle ? std::clamp(*cmd.throttle, 0.0, 1.0) : 1.0);

    const State& s = gc.state;
    const Vector3 euler = s.eulerAngles();
    const double phi = euler.x, theta = euler.y, psi = euler.z;
    const double p = s.angularRate.x, q = s.angularRate.y, r = s.angularRate.z;
    const auto clampA = [&](double a) {
        return std::clamp(a, -g_.maxAngAccel, g_.maxAngAccel);
    };

    // ---- Attitude errors -> desired angular accelerations [rad/s^2] ----
    const double thetaCmd = cmd.pitch ? *cmd.pitch : theta;
    const double ePitch = thetaCmd - theta;
    ziPitch_ = std::clamp(ziPitch_ + ePitch * gc.dt, -g_.intLimit, g_.intLimit);
    const double aPitch =
        clampA(g_.pitchKp * ePitch - g_.pitchKd * q + g_.pitchKi * ziPitch_);

    // Near vertical, heading/roll Euler angles are ill-conditioned: rate-damp.
    double aYaw, aRoll;
    if (std::abs(theta) > g_.verticalGuard) {
        aYaw  = clampA(-g_.yawKd * r);
        aRoll = clampA(-g_.rollKd * p);
    } else {
        const double psiCmd = cmd.heading ? *cmd.heading : psi;
        const double eYaw = units::wrapAngle(psiCmd - psi);
        ziYaw_ = std::clamp(ziYaw_ + eYaw * gc.dt, -g_.intLimit, g_.intLimit);
        aYaw = clampA(g_.yawKp * eYaw - g_.yawKd * r + g_.yawKi * ziYaw_);

        const double phiCmd = cmd.roll ? *cmd.roll : 0.0;
        aRoll = clampA(g_.rollKp * units::wrapAngle(phiCmd - phi) - g_.rollKd * p);
    }

    // ---- Pseudo-controls: nu = I * alpha_desired (diagonal terms) ----
    const Matrix3x3& I = gc.mass.inertia;
    const Vector3 nu(I(0, 0) * aRoll, I(1, 1) * aPitch, I(2, 2) * aYaw);

    // ---- Allocation over the components' current effectiveness ----
    ControlEffect effects[ChannelTable::kMaxChannels];
    int n = 0;
    const ComponentContext cctx{ s, gc.air, s.altitude(), gc.dt, gc.mass.xcg };
    for (const ForceComponent* c : components_)
        n += c->controlEffectiveness(cctx, effects + n,
                                     ChannelTable::kMaxChannels - n);
    allocator_.allocate(nu, effects, n, table_, out);
}
