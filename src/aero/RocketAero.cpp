#include "aero/RocketAero.h"

std::unique_ptr<AeroModel> RocketAero::fromJson(const json::Value& cfg) {
    AeroReference ref;
    ref.sref = cfg.num("sref_m2");
    ref.cbar = cfg.num("lref_m");    // body length: pitch/yaw reference
    ref.bref = cfg.num("dref_m");    // diameter: roll reference

    Derivatives d;
    d.ca0  = cfg.num("ca0");
    d.cna  = cfg.num("cna");
    d.cnde = cfg.num("cnde", 0.0);
    d.cma  = cfg.num("cma");
    d.cmq  = cfg.num("cmq", 0.0);
    d.cmde = cfg.num("cmde", 0.0);
    d.clp  = cfg.num("clp", 0.0);
    d.clda = cfg.num("clda", 0.0);

    // Axisymmetric mirror defaults; override in config for asymmetric vehicles.
    d.cyb  = cfg.num("cyb",  -d.cna);
    d.cydr = cfg.num("cydr", -d.cnde);
    d.cnb  = cfg.num("cnb",  -d.cma);
    d.cnr  = cfg.num("cnr",   d.cmq);
    d.cndr = cfg.num("cndr",  d.cmde);

    return std::make_unique<RocketAero>(ref, d);
}

AeroForces RocketAero::compute(const State& state, const AirData& air,
                               const ControlInput& u) const {
    AeroForces out;
    const double V = air.airspeed;
    if (V < 1e-6 || air.qbar <= 0.0) return out;

    const double alpha = air.alpha;
    const double beta  = air.beta;
    const double qS    = air.qbar * ref_.sref;

    // Normalized body rates (roll uses diameter, pitch/yaw use body length).
    const double phat = state.angularRate.x * ref_.bref / (2.0 * V);
    const double qhat = state.angularRate.y * ref_.cbar / (2.0 * V);
    const double rhat = state.angularRate.z * ref_.cbar / (2.0 * V);

    // --- Forces ---
    const double CA = d_.ca0;
    const double CN = d_.cna * alpha + d_.cnde * u.elevator;
    const double CY = d_.cyb * beta + d_.cydr * u.rudder;
    out.force.x = -CA * qS;
    out.force.y =  CY * qS;
    out.force.z = -CN * qS;

    // --- Moments ---
    const double Cl = d_.clp * phat + d_.clda * u.aileron;
    const double Cm = d_.cma * alpha + d_.cmq * qhat + d_.cmde * u.elevator;
    const double Cn = d_.cnb * beta  + d_.cnr * rhat + d_.cndr * u.rudder;
    out.moment.x = Cl * qS * ref_.bref;
    out.moment.y = Cm * qS * ref_.cbar;
    out.moment.z = Cn * qS * ref_.cbar;
    return out;
}
