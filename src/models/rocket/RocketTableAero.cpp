#include "models/rocket/RocketTableAero.h"

#include <cmath>
#include <filesystem>

#include "io/CsvReader.h"

std::unique_ptr<AeroModel> RocketTableAero::fromJson(const json::Value& cfg,
                                                     const std::string& baseDir) {
    AeroReference ref;
    ref.sref = cfg.num("sref_m2");
    ref.cbar = cfg.num("cbar_m");    // body length (DATCOM CBARR)
    ref.bref = cfg.num("bref_m");    // span

    const auto resolve = [&](const std::string& p) {
        const std::filesystem::path fp(p);
        return fp.is_absolute() ? p : (std::filesystem::path(baseDir) / fp).string();
    };

    const csv::Table stat = csv::read(resolve(cfg.str("tables_csv")));
    const csv::Table ctrl = csv::read(resolve(cfg.str("control_csv")));

    Tables t;
    t.cn  = csv::buildTable2D(stat, "alpha_rad", "mach", "CN");
    t.ca  = csv::buildTable2D(stat, "alpha_rad", "mach", "CA");
    t.cm  = csv::buildTable2D(stat, "alpha_rad", "mach", "CM");
    t.cmq = csv::buildTable2D(stat, "alpha_rad", "mach", "CMQ");
    t.cnr = csv::buildTable2D(stat, "alpha_rad", "mach", "CNR");
    t.clp = csv::buildTable2D(stat, "alpha_rad", "mach", "CLP");
    t.cnb = csv::buildTable2D(stat, "alpha_rad", "mach", "CNB");
    t.cyb = csv::buildTable2D(stat, "alpha_rad", "mach", "CYB");

    t.dcmCtrl = csv::buildTable2D(ctrl, "delta_rad", "mach", "dCM_sym");
    t.dclCtrl = csv::buildTable2D(ctrl, "delta_rad", "mach", "dCL_sym");
    t.clRoll  = csv::buildTable2D(ctrl, "delta_rad", "mach", "Cl_roll");

    const double xref = cfg.has("xref_m") ? cfg.num("xref_m") : std::nan("");
    return std::make_unique<RocketTableAero>(ref, std::move(t), xref);
}

void RocketTableAero::declareChannels(ChannelTable& table) {
    // Declared travel is metadata; enforcement stays with the actuator config.
    constexpr double lim = 0.7854;   // 45 deg
    elevator_ = table.add({channels::kElevator, ChannelKind::Surface, -lim, lim});
    aileron_  = table.add({channels::kAileron,  ChannelKind::Surface, -lim, lim});
    rudder_   = table.add({channels::kRudder,   ChannelKind::Surface, -lim, lim});
}

AeroForces RocketTableAero::compute(const State& state, const AirData& air,
                                    const ChannelValues& u) const {
    AeroForces out;
    const double V = air.airspeed;
    if (V < 1e-6 || air.qbar <= 0.0) return out;

    const double alpha = air.alpha;
    const double beta  = air.beta;
    const double mach  = air.mach;
    const double qS    = air.qbar * ref_.sref;

    // Control deflections produce FORCE as well as moment: elevator fin lift
    // joins the normal force; by cruciform mirror symmetry the same table
    // gives the rudder side force.
    // The DATCOM pitch tables are mirrored onto the yaw channel with the v1
    // convention +delta -> nose RIGHT; this project defines +rudder = nose
    // LEFT (aircraft-style, see ControlInput.h), so the rudder is looked up
    // negated in both the force and moment tables.
    const double de = u.get(elevator_);
    const double da = u.get(aileron_);
    const double dr = -u.get(rudder_);

    const double CN = t_.cn.eval(alpha, mach) + t_.dclCtrl.eval(de, mach);
    const double CA = t_.ca.eval(alpha, mach);
    const double CM = t_.cm.eval(alpha, mach);
    const double CY = t_.cyb.eval(alpha, mach) * beta
                    + t_.dclCtrl.eval(dr, mach);

    // Nondimensional body rates.
    const double qhat = state.angularRate.y * ref_.cbar / (2.0 * V);
    const double phat = state.angularRate.x * ref_.bref / (2.0 * V);
    const double rhat = state.angularRate.z * ref_.bref / (2.0 * V);

    const double CM_total = CM + t_.cmq.eval(alpha, mach) * qhat
                          + t_.dcmCtrl.eval(de, mach);

    const double Cl_total = t_.clp.eval(alpha, mach) * phat
                          + t_.clRoll.eval(da, mach);

    // Weathercock is stabilizing with the leading minus; cnb and the reused
    // pitch control table are per-cbar quantities in a per-bref channel ->
    // cbar/bref conversion (cnr is natively per-bref, not scaled).
    const double c2b = ref_.cbar / ref_.bref;
    const double Cn_total = (-t_.cnb.eval(alpha, mach) * beta
                             - t_.dcmCtrl.eval(dr, mach)) * c2b
                          + t_.cnr.eval(alpha, mach) * rhat;

    // Body axes: +beta (velocity from the right) pushes the vehicle left.
    out.force  = Vector3(-CA * qS, -CY * qS, -CN * qS);
    out.moment = Vector3(Cl_total * qS * ref_.bref,
                         CM_total * qS * ref_.cbar,
                         Cn_total * qS * ref_.bref);
    return out;
}

int RocketTableAero::controlEffectiveness(const AirData& air, double xcg,
                                          ControlEffect* out, int maxOut) const {
    const double qS = air.qbar * ref_.sref;
    const double mach = air.mach;
    const double h = 0.0349;   // 2 deg central-difference step

    // Per-rad slopes of the control tables about zero deflection.
    const double dcm = (t_.dcmCtrl.eval(h, mach) - t_.dcmCtrl.eval(-h, mach)) / (2.0 * h);
    const double dcl = (t_.dclCtrl.eval(h, mach) - t_.dclCtrl.eval(-h, mach)) / (2.0 * h);
    const double dclr = (t_.clRoll.eval(h, mach) - t_.clRoll.eval(-h, mach)) / (2.0 * h);

    // The compute() sign structure gives dMy_ref/de = qS*cbar*dcm and (via the
    // mirrored, negated rudder lookup) dMz_ref/dr = qS*cbar*dcm; the fin force
    // (dFz/de = -qS*dcl, dFy/dr = +qS*dcl) moves those moments to the CG the
    // same way the Entity transfers the full wrench. Cruciform symmetry makes
    // both axes come out identical.
    const double dx = (std::isfinite(xref_) && std::isfinite(xcg)) ? (xcg - xref_) : 0.0;
    const double dM = qS * (ref_.cbar * dcm + dx * dcl);

    int n = 0;
    if (elevator_.valid() && n < maxOut)
        out[n++] = { elevator_, Vector3(0.0, dM, 0.0) };
    if (rudder_.valid() && n < maxOut)
        out[n++] = { rudder_, Vector3(0.0, 0.0, dM) };
    if (aileron_.valid() && n < maxOut)
        out[n++] = { aileron_, Vector3(qS * ref_.bref * dclr, 0.0, 0.0) };
    return n;
}
