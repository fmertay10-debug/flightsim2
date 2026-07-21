#include "models/f16/F16Aero.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <utility>
#include <vector>

#include "io/CsvReader.h"

F16Aero::F16Aero(F16Tables tables, AeroReference ref, double xcgrCbar)
    : t_(std::move(tables)), ref_(ref), xcgr_(xcgrCbar) {}

void F16Aero::declareChannels(ChannelTable& table) {
    // Declared travel = the S&L surface limits (25 / 21.5 / 30 deg).
    elevator_ = table.add({channels::kElevator, ChannelKind::Surface, -0.4363, 0.4363});
    aileron_  = table.add({channels::kAileron,  ChannelKind::Surface, -0.3752, 0.3752});
    rudder_   = table.add({channels::kRudder,   ChannelKind::Surface, -0.5236, 0.5236});
}

Wrench F16Aero::computeWrench(const ComponentContext& ctx, const ChannelValues& u,
                              const double* /*x: stateless*/) const {
    const State&   state = ctx.state;
    const AirData& air   = ctx.air;
    Wrench out;
    const double V = air.airspeed;
    if (V < 1e-6 || air.qbar <= 0.0) return out;

    const double elevator = u.get(elevator_);

    // alpha from body velocity (AirData); beta per S&L definition asin(v/V).
    const double alpha = air.alpha;
    const double beta  = std::asin(std::clamp(air.velocityBody.y / V, -1.0, 1.0));
    const double qS    = air.qbar * ref_.sref;

    // The S&L closed-form terms are historically in DEGREES; aileron/rudder
    // enter normalized by their travel limits (20 / 30 deg).
    constexpr double R2D = 180.0 / 3.14159265358979323846;
    const double betaDeg = beta * R2D;
    const double elDeg   = elevator * R2D;
    const double dail    = u.get(aileron_) * R2D / 20.0;
    const double drdr    = u.get(rudder_)  * R2D / 30.0;

    const double p = state.angularRate.x, q = state.angularRate.y,
                 r = state.angularRate.z;
    const double cq  = ref_.cbar * q / (2.0 * V);
    const double b2v = ref_.bref / (2.0 * V);

    // --- The 6-coefficient buildup (S&L Appendix A) ---
    const double CXT = t_.cx.eval(alpha, elevator) + cq * t_.cxq.eval(alpha);

    const double CYT = -0.02 * betaDeg + 0.021 * dail + 0.086 * drdr
                     + b2v * (t_.cyr.eval(alpha) * r + t_.cyp.eval(alpha) * p);

    const double CZT = t_.cz.eval(alpha) * (1.0 - (betaDeg / 57.3) * (betaDeg / 57.3))
                     - 0.19 * (elDeg / 25.0)
                     + cq * t_.czq.eval(alpha);

    const double CLT = t_.cl.eval(alpha, beta)
                     + t_.dlda.eval(alpha, beta) * dail
                     + t_.dldr.eval(alpha, beta) * drdr
                     + b2v * (t_.clr.eval(alpha) * r + t_.clp.eval(alpha) * p);

    const double CMT = t_.cm.eval(alpha, elevator) + cq * t_.cmq.eval(alpha);

    const double CNT = t_.cn.eval(alpha, beta)
                     + t_.dnda.eval(alpha, beta) * dail
                     + t_.dndr.eval(alpha, beta) * drdr
                     + b2v * (t_.cnr.eval(alpha) * r + t_.cnp.eval(alpha) * p);

    // Direct body-axis coefficients: no leading minus. Moments about xcgr;
    // the Entity transfers them to the current CG.
    out.force  = Vector3(CXT * qS, CYT * qS, CZT * qS);
    out.moment = Vector3(CLT * qS * ref_.bref,
                         CMT * qS * ref_.cbar,
                         CNT * qS * ref_.bref);
    return out;
}

int F16Aero::controlEffectiveness(const ComponentContext& ctx,
                                  ControlEffect* out, int maxOut) const {
    const AirData& air = ctx.air;
    const double   xcg = ctx.xcg;
    const double V = air.airspeed;
    if (V < 1e-6 || air.qbar <= 0.0) return 0;

    const double alpha = air.alpha;
    const double beta  = std::asin(std::clamp(air.velocityBody.y / V, -1.0, 1.0));
    const double qS    = air.qbar * ref_.sref;
    constexpr double R2D = 180.0 / 3.14159265358979323846;
    const double h = 0.0349;   // 2 deg central-difference step (elevator table)

    // CG transfer arm, same convention as the Entity: My -= dx*Fz, Mz += dx*Fy.
    const double xref = xcgr_ * ref_.cbar;
    const double dx = std::isfinite(xcg) ? (xcg - xref) : 0.0;

    int n = 0;
    if (elevator_.valid() && n < maxOut) {
        const double dcm = (t_.cm.eval(alpha, h) - t_.cm.eval(alpha, -h)) / (2.0 * h);
        const double dcz = -0.19 * R2D / 25.0;   // analytic CZ-per-rad of elevator
        out[n++] = { elevator_,
                     Vector3(0.0, qS * (ref_.cbar * dcm - dx * dcz), 0.0) };
    }
    if (aileron_.valid() && n < maxOut) {
        const double s   = R2D / 20.0;           // dail per rad of aileron channel
        const double dcl = t_.dlda.eval(alpha, beta) * s;
        const double dcn = t_.dnda.eval(alpha, beta) * s;
        const double dcy = 0.021 * s;
        out[n++] = { aileron_,
                     Vector3(qS * ref_.bref * dcl, 0.0,
                             qS * (ref_.bref * dcn + dx * dcy)) };
    }
    if (rudder_.valid() && n < maxOut) {
        const double s   = R2D / 30.0;           // drdr per rad of rudder channel
        const double dcl = t_.dldr.eval(alpha, beta) * s;
        const double dcn = t_.dndr.eval(alpha, beta) * s;
        const double dcy = 0.086 * s;
        out[n++] = { rudder_,
                     Vector3(qS * ref_.bref * dcl, 0.0,
                             qS * (ref_.bref * dcn + dx * dcy)) };
    }
    return n;
}

namespace {

LookupTable1D col1d(const csv::Table& t, const std::string& xcol,
                    const std::string& ycol) {
    const std::size_t xi = t.col(xcol), yi = t.col(ycol);
    std::vector<double> x, y;
    x.reserve(t.rows.size());
    y.reserve(t.rows.size());
    for (const auto& row : t.rows) { x.push_back(row[xi]); y.push_back(row[yi]); }
    return LookupTable1D(std::move(x), std::move(y));
}

} // namespace

std::unique_ptr<F16Aero> F16Aero::fromJson(const json::Value& cfg,
                                             const std::string& baseDir) {
    const std::filesystem::path dir =
        std::filesystem::path(cfg.str("dir")).is_absolute()
            ? std::filesystem::path(cfg.str("dir"))
            : std::filesystem::path(baseDir) / cfg.str("dir");
    const std::string d = dir.string();

    F16Tables tb;
    csv::Table t = csv::read(d + "/cx.csv");
    tb.cx = csv::buildTable2D(t, "alpha_rad", "el_rad", "CX");
    t = csv::read(d + "/cm.csv");
    tb.cm = csv::buildTable2D(t, "alpha_rad", "el_rad", "CM");
    t = csv::read(d + "/cl.csv");
    tb.cl = csv::buildTable2D(t, "alpha_rad", "beta_rad", "CL");
    t = csv::read(d + "/cn.csv");
    tb.cn = csv::buildTable2D(t, "alpha_rad", "beta_rad", "CN");
    t = csv::read(d + "/dlda.csv");
    tb.dlda = csv::buildTable2D(t, "alpha_rad", "beta_rad", "DLDA");
    t = csv::read(d + "/dldr.csv");
    tb.dldr = csv::buildTable2D(t, "alpha_rad", "beta_rad", "DLDR");
    t = csv::read(d + "/dnda.csv");
    tb.dnda = csv::buildTable2D(t, "alpha_rad", "beta_rad", "DNDA");
    t = csv::read(d + "/dndr.csv");
    tb.dndr = csv::buildTable2D(t, "alpha_rad", "beta_rad", "DNDR");

    t = csv::read(d + "/cz.csv");
    tb.cz = col1d(t, "alpha_rad", "CZ");

    t = csv::read(d + "/damping.csv");
    tb.cxq = col1d(t, "alpha_rad", "CXq");
    tb.cyr = col1d(t, "alpha_rad", "CYr");
    tb.cyp = col1d(t, "alpha_rad", "CYp");
    tb.czq = col1d(t, "alpha_rad", "CZq");
    tb.clr = col1d(t, "alpha_rad", "Clr");
    tb.clp = col1d(t, "alpha_rad", "Clp");
    tb.cmq = col1d(t, "alpha_rad", "Cmq");
    tb.cnr = col1d(t, "alpha_rad", "Cnr");
    tb.cnp = col1d(t, "alpha_rad", "Cnp");

    const auto ref = csv::readKeyValue(d + "/reference.csv");
    AeroReference r;
    r.sref = ref.at("sref_m2");
    r.bref = ref.at("bref_m");
    r.cbar = ref.at("cbar_m");
    const double xcgr = ref.at("xcgr_cbar");

    return std::make_unique<F16Aero>(std::move(tb), r, xcgr);
}
