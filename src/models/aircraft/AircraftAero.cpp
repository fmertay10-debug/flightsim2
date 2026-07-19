#include "models/aircraft/AircraftAero.h"

#include <cmath>

std::unique_ptr<AeroModel> AircraftAero::fromJson(const json::Value& cfg) {
    AeroReference ref;
    ref.sref = cfg.num("sref_m2");
    ref.cbar = cfg.num("cbar_m");
    ref.bref = cfg.num("bspan_m");

    Derivatives d;
    d.cl0  = cfg.num("cl0", 0.0);
    d.cla  = cfg.num("cla");
    d.clq  = cfg.num("clq", 0.0);
    d.clde = cfg.num("clde", 0.0);
    d.cd0  = cfg.num("cd0");
    d.kInduced = cfg.num("k_induced", 0.0);
    d.cyb  = cfg.num("cyb", 0.0);
    d.cydr = cfg.num("cydr", 0.0);
    d.clb  = cfg.num("clb", 0.0);
    d.clp  = cfg.num("clp", 0.0);
    d.clr  = cfg.num("clr", 0.0);
    d.clda = cfg.num("clda", 0.0);
    d.cldr = cfg.num("cldr", 0.0);
    d.cm0  = cfg.num("cm0", 0.0);
    d.cma  = cfg.num("cma");
    d.cmq  = cfg.num("cmq", 0.0);
    d.cmde = cfg.num("cmde", 0.0);
    d.cnb  = cfg.num("cnb", 0.0);
    d.cnp  = cfg.num("cnp", 0.0);
    d.cnr  = cfg.num("cnr", 0.0);
    d.cnda = cfg.num("cnda", 0.0);
    d.cndr = cfg.num("cndr", 0.0);

    return std::make_unique<AircraftAero>(ref, d);
}

void AircraftAero::declareChannels(ChannelTable& table) {
    // Declared travel is metadata; enforcement stays with the actuator config.
    constexpr double lim = 0.7854;   // 45 deg
    if (d_.clde != 0.0 || d_.cmde != 0.0)
        elevator_ = table.add({channels::kElevator, ChannelKind::Surface, -lim, lim});
    if (d_.clda != 0.0 || d_.cnda != 0.0)
        aileron_ = table.add({channels::kAileron, ChannelKind::Surface, -lim, lim});
    if (d_.cydr != 0.0 || d_.cldr != 0.0 || d_.cndr != 0.0)
        rudder_ = table.add({channels::kRudder, ChannelKind::Surface, -lim, lim});
}

int AircraftAero::controlEffectiveness(const AirData& air, double /*xcg*/,
                                       ControlEffect* out, int maxOut) const {
    const double qSc = air.qbar * ref_.sref * ref_.cbar;
    const double qSb = air.qbar * ref_.sref * ref_.bref;
    int n = 0;
    if (elevator_.valid() && n < maxOut)
        out[n++] = { elevator_, Vector3(0.0, d_.cmde * qSc, 0.0) };
    if (aileron_.valid() && n < maxOut)
        out[n++] = { aileron_, Vector3(d_.clda * qSb, 0.0, d_.cnda * qSb) };
    if (rudder_.valid() && n < maxOut)
        out[n++] = { rudder_, Vector3(d_.cldr * qSb, 0.0, d_.cndr * qSb) };
    return n;
}

AeroForces AircraftAero::compute(const State& state, const AirData& air,
                                 const ChannelValues& u) const {
    AeroForces out;
    const double V = air.airspeed;
    if (V < 1e-6 || air.qbar <= 0.0) return out;

    const double de = u.get(elevator_);
    const double da = u.get(aileron_);
    const double dr = u.get(rudder_);

    const double alpha = air.alpha;
    const double beta  = air.beta;
    const double qS    = air.qbar * ref_.sref;

    // Normalized body rates.
    const double phat = state.angularRate.x * ref_.bref / (2.0 * V);
    const double qhat = state.angularRate.y * ref_.cbar / (2.0 * V);
    const double rhat = state.angularRate.z * ref_.bref / (2.0 * V);

    // --- Force coefficients ---
    const double CL = d_.cl0 + d_.cla * alpha + d_.clq * qhat + d_.clde * de;
    const double CD = d_.cd0 + d_.kInduced * CL * CL;
    const double CY = d_.cyb * beta + d_.cydr * dr;

    // Stability axes -> body axes (rotate by alpha): drag along -x_s, lift along -z_s.
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    out.force.x = (-CD * ca + CL * sa) * qS;
    out.force.y = CY * qS;
    out.force.z = (-CD * sa - CL * ca) * qS;

    // --- Moment coefficients ---
    const double Cl = d_.clb * beta + d_.clp * phat + d_.clr * rhat
                    + d_.clda * da + d_.cldr * dr;
    const double Cm = d_.cm0 + d_.cma * alpha + d_.cmq * qhat + d_.cmde * de;
    const double Cn = d_.cnb * beta + d_.cnp * phat + d_.cnr * rhat
                    + d_.cnda * da + d_.cndr * dr;

    out.moment.x = Cl * qS * ref_.bref;
    out.moment.y = Cm * qS * ref_.cbar;
    out.moment.z = Cn * qS * ref_.bref;
    return out;
}
