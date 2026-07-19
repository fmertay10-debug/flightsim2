#include "gnc/control/laws/ScheduledLaw.h"

#include <algorithm>
#include <filesystem>
#include <vector>

#include "io/CsvReader.h"
#include "math/Units.h"

std::unique_ptr<ControlLaw> ScheduledLaw::fromJson(const json::Value& cfg,
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

void ScheduledLaw::bindComponents(
    const std::vector<std::unique_ptr<ForceComponent>>& components) {
    components_.clear();
    for (const auto& c : components) components_.push_back(c.get());
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

    // The roll axis has tiny inertia and huge fin authority, so its control
    // power scales with dynamic pressure. Attenuate the roll command above a
    // reference qbar (never amplify below it) to keep the roll-loop bandwidth
    // bounded -- otherwise a fixed-gain roll loop limit-cycles at the high qbar
    // of a diving missile and diverges.
    const double rollScale = std::min(1.0, c_.qbarRef / std::max(air.qbar, 1.0));

    // ---- Pitch: state feedback with integral tracking ----
    const double thetaCmd = cmd.pitch ? *cmd.pitch : theta;
    const double eTheta = theta - thetaCmd;
    const double uElevRaw = -(kA * air.alpha + kQ * q + kT * eTheta + kI * ziTheta_);
    const double uElev = std::clamp(uElevRaw, -c_.maxFin, c_.maxFin);
    // Conditional integration: freeze when saturated (anti-windup).
    if (std::abs(uElevRaw) < c_.maxFin) ziTheta_ += eTheta * dt;

    double uRud, uAil;
    if (std::abs(theta) > c_.verticalGuard) {
        // Near vertical, heading/roll Euler angles are ill-conditioned:
        // rate-damp. (-kQ is the positive damping magnitude: the pitch law
        // damps q via -(kQ*q).)
        uRud = std::clamp(-kQ * r, -c_.maxFin, c_.maxFin);
        uAil = std::clamp(rollScale * (-c_.rollKd * p), -c_.maxFin, c_.maxFin);
    } else {
        // ---- Yaw: mirror the pitch feedback by axisymmetry ----
        // Sideslip beta plays alpha's role; heading error plays theta's. Both
        // +rudder->nose-LEFT and +elevator->nose-DOWN drive their attitude the
        // same way, so the yaw law has the SAME form as pitch (no extra
        // negation) -- matches the sign structure of the validated PID law.
        const double psiCmd = cmd.heading ? *cmd.heading : psi;
        const double ePsi = units::wrapAngle(psi - psiCmd);
        const double uRudRaw = -(kA * air.beta + kQ * r + kT * ePsi + kI * ziPsi_);
        uRud = std::clamp(uRudRaw, -c_.maxFin, c_.maxFin);
        if (std::abs(uRudRaw) < c_.maxFin) ziPsi_ += ePsi * dt;

        // ---- Roll: PD hold wings level (qbar-normalized, see above) ----
        const double phiCmd = cmd.roll ? *cmd.roll : 0.0;
        const double raw = c_.rollKp * units::wrapAngle(phiCmd - phi) - c_.rollKd * p;
        uAil = std::clamp(rollScale * raw, -c_.maxFin, c_.maxFin);
    }

    // ---- Currency conversion (ADR-0004 Option B): the designed deflections
    // become a WrenchCommand through the components' effectiveness columns,
    // and the Allocator maps it back onto the channels. One effector per axis
    // makes the round trip ~exact (see the class comment).
    ControlEffect fx[ChannelTable::kMaxChannels];
    int n = 0;
    const ComponentContext cctx{ gc.state, air, gc.state.altitude(), dt, gc.mass.xcg };
    for (const ForceComponent* comp : components_)
        n += comp->controlEffectiveness(cctx, fx + n,
                                        ChannelTable::kMaxChannels - n);

    WrenchCommand nu;
    for (int k = 0; k < n; ++k) {
        const int idx = fx[k].channel.index;
        double u = 0.0;
        if      (idx == elevator_.index) u = uElev;
        else if (idx == rudder_.index)   u = uRud;
        else if (idx == aileron_.index)  u = uAil;
        nu.force.x  += fx[k].dForce.x  * u;
        nu.force.y  += fx[k].dForce.y  * u;
        nu.force.z  += fx[k].dForce.z  * u;
        nu.moment.x += fx[k].dMoment.x * u;
        nu.moment.y += fx[k].dMoment.y * u;
        nu.moment.z += fx[k].dMoment.z * u;
    }
    allocator_.allocate(nu, fx, n, table_, out);
}
