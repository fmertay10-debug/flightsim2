#include "aero/AircraftAero.h"

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

AeroForces AircraftAero::compute(const State& state, const AirData& air,
                                 const ControlInput& u) const {
    AeroForces out;
    const double V = air.airspeed;
    if (V < 1e-6 || air.qbar <= 0.0) return out;

    const double alpha = air.alpha;
    const double beta  = air.beta;
    const double qS    = air.qbar * ref_.sref;

    // Normalized body rates.
    const double phat = state.angularRate.x * ref_.bref / (2.0 * V);
    const double qhat = state.angularRate.y * ref_.cbar / (2.0 * V);
    const double rhat = state.angularRate.z * ref_.bref / (2.0 * V);

    // --- Force coefficients ---
    const double CL = d_.cl0 + d_.cla * alpha + d_.clq * qhat + d_.clde * u.elevator;
    const double CD = d_.cd0 + d_.kInduced * CL * CL;
    const double CY = d_.cyb * beta + d_.cydr * u.rudder;

    // Stability axes -> body axes (rotate by alpha): drag along -x_s, lift along -z_s.
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    out.force.x = (-CD * ca + CL * sa) * qS;
    out.force.y = CY * qS;
    out.force.z = (-CD * sa - CL * ca) * qS;

    // --- Moment coefficients ---
    const double Cl = d_.clb * beta + d_.clp * phat + d_.clr * rhat
                    + d_.clda * u.aileron + d_.cldr * u.rudder;
    const double Cm = d_.cm0 + d_.cma * alpha + d_.cmq * qhat + d_.cmde * u.elevator;
    const double Cn = d_.cnb * beta + d_.cnp * phat + d_.cnr * rhat
                    + d_.cnda * u.aileron + d_.cndr * u.rudder;

    out.moment.x = Cl * qS * ref_.bref;
    out.moment.y = Cm * qS * ref_.cbar;
    out.moment.z = Cn * qS * ref_.bref;
    return out;
}
