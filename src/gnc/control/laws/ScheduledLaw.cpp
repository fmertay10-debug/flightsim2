#include "gnc/control/laws/ScheduledLaw.h"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include "io/CsvReader.h"
#include "math/Units.h"

std::unique_ptr<ControlLaw> ScheduledLaw::fromJson(const json::Value& cfg,
                                                          const std::string& baseDir) {
    const std::filesystem::path sp(cfg.str("schedule"));
    const std::string path = sp.is_absolute()
        ? sp.string() : (std::filesystem::path(baseDir) / sp).string();

    const csv::Table t = csv::read(path);
    // Accel-domain schedule (ADR-0004 C1). An old fin-domain file (k_alpha,
    // k_q, ...) must be regenerated, not misread as accelerations.
    bool hasAcc = false;
    for (const std::string& name : t.columns)
        if (name == "k_alpha_acc") hasAcc = true;
    if (!hasAcc)
        throw std::invalid_argument(
            "scheduled law: '" + path + "' is a retired fin-domain schedule "
            "(k_alpha,...). Regenerate the acceleration-domain schedule "
            "(k_alpha_acc,...) with tools/design_autopilot.py.");
    const std::size_t mi = t.col("mach");
    const auto col = [&](const std::string& name) {
        const std::size_t ci = t.col(name);
        std::vector<double> mach, val;
        mach.reserve(t.rows.size());
        val.reserve(t.rows.size());
        for (const auto& r : t.rows) { mach.push_back(r[mi]); val.push_back(r[ci]); }
        return LookupTable1D(std::move(mach), std::move(val));
    };

    Config c;
    c.kAlpha = col("k_alpha_acc");
    c.kQ     = col("k_q_acc");
    c.kTheta = col("k_theta_acc");
    c.kI     = col("k_i_acc");

    if (cfg.has("limits")) {
        const json::Value& j = cfg.at("limits");
        c.maxAngAccel   = units::deg2rad(
            j.num("max_ang_accel_dps2", units::rad2deg(c.maxAngAccel)));
        c.minAirspeed   = j.num("min_airspeed_ms", c.minAirspeed);
        c.verticalGuard = units::deg2rad(j.num("vertical_guard_deg",
                                               units::rad2deg(c.verticalGuard)));
    }
    if (cfg.has("roll")) {
        const json::Value& j = cfg.at("roll");
        c.rollKp = j.num("kp", c.rollKp);
        c.rollKd = j.num("kd", c.rollKd);
    }
    return std::make_unique<ScheduledLaw>(std::move(c));
}

std::vector<ChannelHandle> ScheduledLaw::bindChannels(const ChannelTable& t) {
    table_    = t;   // allocation needs the declared channel limits per step
    elevator_ = t.require(channels::kElevator);
    rudder_   = t.require(channels::kRudder);
    aileron_  = t.find(channels::kAileron);
    throttle_ = t.find(channels::kThrottle);
    return {elevator_, rudder_, aileron_, throttle_};
}

void ScheduledLaw::update(const GncContext& gc, const CommandSet& cmd,
                    ChannelValues& out) {
    const State& state = gc.state;
    const AirData& air = gc.air;
    const double dt = gc.dt;
    out.set(throttle_, cmd.throttle ? std::clamp(*cmd.throttle, 0.0, 1.0) : 1.0);

    if (air.airspeed < c_.minAirspeed) return;   // fins ineffective on the rail

    const double mach = air.mach;
    const double kA = c_.kAlpha.eval(mach);
    const double kQ = c_.kQ.eval(mach);
    const double kT = c_.kTheta.eval(mach);
    const double kI = c_.kI.eval(mach);

    const Vector3 euler = state.eulerAngles();
    const double phi = euler.x, theta = euler.y, psi = euler.z;
    const double p = state.angularRate.x, q = state.angularRate.y, r = state.angularRate.z;

    const auto clampA = [&](double a) {
        return std::clamp(a, -c_.maxAngAccel, c_.maxAngAccel);
    };

    // ---- Pitch: state feedback with integral tracking [rad/s^2] ----
    const double thetaCmd = cmd.pitch ? *cmd.pitch : theta;
    const double eTheta = theta - thetaCmd;
    const double aPitchRaw = -(kA * air.alpha + kQ * q + kT * eTheta + kI * ziTheta_.value());
    const double aPitch = clampA(aPitchRaw);
    // Conditional integration: freeze while the demand is clamped (anti-windup).
    ziTheta_.accumulate(eTheta, dt, /*hold=*/std::abs(aPitchRaw) >= c_.maxAngAccel);

    double aYaw, aRoll;
    if (std::abs(theta) > c_.verticalGuard) {
        // Near vertical, heading/roll Euler angles are ill-conditioned:
        // rate-damp. (-kQ is the positive damping magnitude: the pitch law
        // damps q via -(kQ*q).)
        aYaw  = clampA(-kQ * r);
        aRoll = clampA(-c_.rollKd * p);
    } else {
        // ---- Yaw: mirror the pitch feedback by axisymmetry ----
        // Sideslip beta plays alpha's role; heading error plays theta's; the
        // demanded accelerations carry no channel signs (the Allocator gets
        // those from the effectiveness columns).
        const double psiCmd = cmd.heading ? *cmd.heading : psi;
        const double ePsi = units::wrapAngle(psi - psiCmd);
        const double aYawRaw = -(kA * air.beta + kQ * r + kT * ePsi + kI * ziPsi_.value());
        aYaw = clampA(aYawRaw);
        ziPsi_.accumulate(ePsi, dt, /*hold=*/std::abs(aYawRaw) >= c_.maxAngAccel);

        // ---- Roll: PD hold wings level. No qbar attenuation needed: the
        // allocator divides the fixed accel demand by the qbar-growing roll
        // effectiveness, so the deflection shrinks by construction (the old
        // fin-domain law needed an explicit qbarRef hack here). ----
        const double phiCmd = cmd.roll ? *cmd.roll : 0.0;
        aRoll = clampA(c_.rollKp * units::wrapAngle(phiCmd - phi) - c_.rollKd * p);
    }

    commandAngularAccel(Vector3(aRoll, aPitch, aYaw), gc, out);
}
