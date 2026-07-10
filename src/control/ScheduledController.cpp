#include "control/ScheduledController.h"

#include <algorithm>
#include <filesystem>
#include <vector>

#include "io/CsvReader.h"
#include "math/Units.h"

std::unique_ptr<Controller> ScheduledController::fromJson(const json::Value& cfg,
                                                          const std::string& baseDir) {
    const std::filesystem::path sp(cfg.str("schedule"));
    const std::string path = sp.is_absolute()
        ? sp.string() : (std::filesystem::path(baseDir) / sp).string();

    const csv::Table t = csv::read(path);
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
    c.kAlpha = col("k_alpha");
    c.kQ     = col("k_q");
    c.kTheta = col("k_theta");
    c.kI     = col("k_i");

    if (cfg.has("limits")) {
        const json::Value& j = cfg.at("limits");
        c.maxFin        = units::deg2rad(j.num("max_fin_deg", units::rad2deg(c.maxFin)));
        c.minAirspeed   = j.num("min_airspeed_ms", c.minAirspeed);
        c.verticalGuard = units::deg2rad(j.num("vertical_guard_deg",
                                               units::rad2deg(c.verticalGuard)));
    }
    if (cfg.has("roll")) {
        const json::Value& j = cfg.at("roll");
        c.rollKp = j.num("kp", c.rollKp);
        c.rollKd = j.num("kd", c.rollKd);
    }
    return std::make_unique<ScheduledController>(std::move(c));
}

ControlInput ScheduledController::update(const State& state, const AirData& air,
                                         const CommandSet& cmd, double dt) {
    ControlInput out;
    out.throttle = cmd.throttle ? std::clamp(*cmd.throttle, 0.0, 1.0) : 1.0;

    if (air.airspeed < c_.minAirspeed) return out;   // fins ineffective on the rail

    const double mach = air.mach;
    const double kA = c_.kAlpha.eval(mach);
    const double kQ = c_.kQ.eval(mach);
    const double kT = c_.kTheta.eval(mach);
    const double kI = c_.kI.eval(mach);

    const Vector3 euler = state.eulerAngles();
    const double phi = euler.x, theta = euler.y, psi = euler.z;
    const double p = state.angularRate.x, q = state.angularRate.y, r = state.angularRate.z;

    // The roll axis has tiny inertia and huge fin authority, so its control
    // power scales with dynamic pressure. Attenuate the roll command above a
    // reference qbar (never amplify below it) to keep the roll-loop bandwidth
    // bounded -- otherwise a fixed-gain roll loop limit-cycles at the high qbar
    // of a diving missile and diverges.
    const double rollScale = std::min(1.0, c_.qbarRef / std::max(air.qbar, 1.0));

    // ---- Pitch: state feedback with integral tracking ----
    const double thetaCmd = cmd.pitch ? *cmd.pitch : theta;
    const double eTheta = theta - thetaCmd;
    const double uElev = -(kA * air.alpha + kQ * q + kT * eTheta + kI * ziTheta_);
    out.elevator = std::clamp(uElev, -c_.maxFin, c_.maxFin);
    // Conditional integration: freeze when saturated (anti-windup).
    if (std::abs(uElev) < c_.maxFin) ziTheta_ += eTheta * dt;

    // Near vertical, heading/roll Euler angles are ill-conditioned: rate-damp.
    // (-kQ is the positive damping magnitude: the pitch law damps q via -(kQ*q).)
    if (std::abs(theta) > c_.verticalGuard) {
        out.rudder  = std::clamp(-kQ * r, -c_.maxFin, c_.maxFin);
        out.aileron = std::clamp(rollScale * (-c_.rollKd * p), -c_.maxFin, c_.maxFin);
        return out;
    }

    // ---- Yaw: mirror the pitch feedback by axisymmetry ----
    // Sideslip beta plays alpha's role; heading error plays theta's. Both
    // +rudder->nose-LEFT and +elevator->nose-DOWN drive their attitude the same
    // way, so the yaw law has the SAME form as pitch (no extra negation) -- this
    // matches the sign structure of the validated PID RocketController.
    const double psiCmd = cmd.heading ? *cmd.heading : psi;
    const double ePsi = units::wrapAngle(psi - psiCmd);
    const double uRud = -(kA * air.beta + kQ * r + kT * ePsi + kI * ziPsi_);
    out.rudder = std::clamp(uRud, -c_.maxFin, c_.maxFin);
    if (std::abs(uRud) < c_.maxFin) ziPsi_ += ePsi * dt;

    // ---- Roll: PD hold wings level (qbar-normalized, see above) ----
    const double phiCmd = cmd.roll ? *cmd.roll : 0.0;
    const double uAil = c_.rollKp * units::wrapAngle(phiCmd - phi) - c_.rollKd * p;
    out.aileron = std::clamp(rollScale * uAil, -c_.maxFin, c_.maxFin);
    return out;
}
