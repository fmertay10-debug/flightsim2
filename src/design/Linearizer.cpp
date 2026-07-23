#include "design/Linearizer.h"

#include <cmath>
#include <memory>
#include <stdexcept>

#include "core/AirData.h"
#include "core/State.h"
#include "environment/Environment.h"
#include "math/Quaternion.h"

namespace design {

namespace {
// One shared ISA/flat-gravity environment for design evaluations.
const Environment& designEnv() {
    static Environment env(std::make_unique<FlatEarthGravity>(),
                           std::make_unique<NoWind>());
    return env;
}
} // namespace

Linearizer::Linearizer(Vehicle& vehicle, const ChannelTable& channels,
                       const LinearizerOptions& opt)
    : veh_(vehicle), channels_(channels), opt_(opt) {
    elevator_ = channels_.find(channels::kElevator);
    if (!elevator_.valid())
        throw std::invalid_argument(
            "linearizer: the vehicle declares no 'elevator' channel -- the "
            "short-period plant needs a pitch control input");

    const MassState ms = veh_.massState(opt_.time);
    mass_ = std::isfinite(opt_.mass) ? opt_.mass : ms.mass;
    iyy_  = std::isfinite(opt_.iyy)  ? opt_.iyy  : ms.inertia(1, 1);
    xcg_  = std::isfinite(opt_.xcg)  ? opt_.xcg  : ms.xcg;

    // Freeze the external component state at its initial values (engine spool
    // at power0, etc.) -- design evaluates f at a POINT, it does not integrate.
    auto& comps = veh_.components();
    int total = 0;
    for (auto& c : comps) {
        compOffsets_.push_back(total);
        total += c->numStates();
    }
    compState_.assign(static_cast<std::size_t>(total), 0.0);
    for (std::size_t i = 0; i < comps.size(); ++i)
        if (comps[i]->numStates() > 0)
            comps[i]->initializeState(compState_.data() + compOffsets_[i]);
}

void Linearizer::wrench(double V, double alpha, double q, double de,
                        double& Fz, double& My) const {
    // Level flight path, nose pitched up by alpha: velocity NED = (V, 0, 0),
    // attitude pitch = alpha -> air.alpha = alpha, beta = 0 (same construction
    // the Entity uses, so every component sees exactly what it sees in flight).
    State s;
    s.time = opt_.time;
    s.position = Vector3(0.0, 0.0, -opt_.altitude);
    s.velocity = Vector3(V, 0.0, 0.0);
    s.attitude = Quaternion::fromEuler(0.0, alpha, 0.0);
    s.angularRate = Vector3(0.0, q, 0.0);

    AirData air;
    air.atmosphere = designEnv().atmosphere(opt_.altitude);
    air.velocityBody = s.attitude.rotate(s.velocity);
    air.airspeed = air.velocityBody.norm();
    air.alpha = std::atan2(air.velocityBody.z, air.velocityBody.x);
    air.beta  = std::atan2(air.velocityBody.y, air.velocityBody.x);
    air.mach = air.airspeed / air.atmosphere.soundSpeed;
    air.qbar = 0.5 * air.atmosphere.density * air.airspeed * air.airspeed;

    ChannelValues u(channels_);
    u.set(elevator_, de);

    const ComponentContext ctx{ s, air, opt_.altitude, 0.0, xcg_ };
    Fz = 0.0;
    My = 0.0;
    auto& comps = veh_.components();
    for (std::size_t i = 0; i < comps.size(); ++i) {
        const int ns = comps[i]->numStates();
        const double* x = ns > 0 ? compState_.data() + compOffsets_[i] : nullptr;
        Wrench w = comps[i]->computeWrench(ctx, u, x);
        // Same reference-to-CG transfer as the Entity.
        const double xref = comps[i]->momentReferenceStation();
        if (std::isfinite(xref) && std::isfinite(xcg_))
            w.moment.y -= (xcg_ - xref) * w.force.z;
        Fz += w.force.z;
        My += w.moment.y;
    }
}

Linearizer::Rates Linearizer::rates(double V, double alpha, double q,
                                    double de) const {
    double Fz, My;
    wrench(V, alpha, q, de, Fz, My);
    return { q + Fz / (mass_ * V), My / iyy_ };
}

PlantPoint Linearizer::at(double mach) const {
    const AtmosphereState atm = designEnv().atmosphere(opt_.altitude);
    const double V = mach * atm.soundSpeed;

    PlantPoint p;
    p.mach = mach;
    p.V = V;
    p.qbar = 0.5 * atm.density * V * V;

    double a0 = 0.0, d0 = 0.0;
    if (opt_.trim) {
        // Newton on (alpha, de): level flight -- aero+propulsive lift carries
        // the weight (gravity enters HERE, not in the derivatives) and the
        // pitch moment balances.
        const double g = designEnv().gravity(opt_.altitude);
        const double h = 1e-4;
        for (int it = 0; it < 40; ++it) {
            const auto res = [&](double a, double d) {
                double Fz, My;
                wrench(V, a, 0.0, d, Fz, My);
                const double r1 = (Fz + mass_ * g * std::cos(a)) / (mass_ * V);
                return std::pair<double, double>(r1, My / iyy_);
            };
            const auto [r1, r2] = res(a0, d0);
            if (std::abs(r1) < 1e-10 && std::abs(r2) < 1e-10) {
                p.trimmed = true;
                break;
            }
            const auto [r1a, r2a] = res(a0 + h, d0);
            const auto [r1d, r2d] = res(a0, d0 + h);
            const double j11 = (r1a - r1) / h, j12 = (r1d - r1) / h;
            const double j21 = (r2a - r2) / h, j22 = (r2d - r2) / h;
            const double det = j11 * j22 - j12 * j21;
            if (std::abs(det) < 1e-14) break;   // singular: give up untrimmed
            a0 -= ( j22 * r1 - j12 * r2) / det;
            d0 -= (-j21 * r1 + j11 * r2) / det;
        }
        p.alphaTrim = a0;
        p.deTrim = d0;
    }

    // Central differences about the reference point.
    const double ha = 0.0349;   // 2 deg in alpha / deflection
    const double hq = 0.05;     // rad/s in pitch rate

    p.Za = (rates(V, a0 + ha, 0.0, d0).alphaDot -
            rates(V, a0 - ha, 0.0, d0).alphaDot) / (2.0 * ha);
    p.Zde = (rates(V, a0, 0.0, d0 + ha).alphaDot -
             rates(V, a0, 0.0, d0 - ha).alphaDot) / (2.0 * ha);
    p.Ma = (rates(V, a0 + ha, 0.0, d0).qDot -
            rates(V, a0 - ha, 0.0, d0).qDot) / (2.0 * ha);
    const Rates rqp = rates(V, a0, hq, d0);
    const Rates rqm = rates(V, a0, -hq, d0);
    p.Mq = (rqp.qDot - rqm.qDot) / (2.0 * hq);
    p.Aq = (rqp.alphaDot - rqm.alphaDot) / (2.0 * hq);
    p.Mde = (rates(V, a0, 0.0, d0 + ha).qDot -
             rates(V, a0, 0.0, d0 - ha).qDot) / (2.0 * ha);
    return p;
}

} // namespace design
