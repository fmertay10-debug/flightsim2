#include <cmath>

#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "component/Propulsor.h"
#include "gnc/control/Allocator.h"
#include "io/Json.h"
#include "models/aircraft/AircraftAero.h"
#include "models/f16/F16Aero.h"
#include "models/rocket/RocketAero.h"
#include "test_util.h"

// Central-difference d(moment)/d(channel) of an AeroModel's own compute() --
// the ground truth every effectiveness column must match.
static Vector3 fdMomentSlope(const AeroModel& m, const ChannelTable& t,
                             ChannelHandle ch, const State& s, const AirData& air,
                             double h = 0.0349) {
    ChannelValues up(t), dn(t);
    up.set(ch, h);
    dn.set(ch, -h);
    const AeroForces a = m.compute(s, air, up);
    const AeroForces b = m.compute(s, air, dn);
    return Vector3((a.moment.x - b.moment.x) / (2.0 * h),
                   (a.moment.y - b.moment.y) / (2.0 * h),
                   (a.moment.z - b.moment.z) / (2.0 * h));
}

struct FixedThrust : PropulsionModel {
    double T = 0.0;
    double thrustFromState(const PropulsionContext&, const double*) const override { return T; }
};

int main() {
    const Allocator alloc;

    // --- Single effector: u = nu / authority ---
    {
        ChannelTable t;
        const ChannelHandle e = t.add({"elevator", ChannelKind::Surface, -0.5, 0.5});
        const ControlEffect fx[] = { { e, Vector3(0.0, 1000.0, 0.0) } };
        ChannelValues u(t);
        alloc.allocate({ Vector3(), Vector3(0.0, 250.0, 0.0) }, fx, 1, t, u);
        CHECK_NEAR(u.get(e), 0.25, 1e-6);
    }

    // --- Redundant effectors share by authority (min-norm least squares) ---
    {
        ChannelTable t;
        const ChannelHandle fin = t.add({"elevator", ChannelKind::Surface, -0.5, 0.5});
        const ChannelHandle tvc = t.add({"tvc_pitch", ChannelKind::Gimbal, -0.5, 0.5});
        const ControlEffect fx[] = { { fin, Vector3(0.0, 1000.0, 0.0) },
                                     { tvc, Vector3(0.0, 4000.0, 0.0) } };
        ChannelValues u(t);
        alloc.allocate({ Vector3(), Vector3(0.0, 1700.0, 0.0) }, fx, 2, t, u);
        // u = B^T(BB^T)^-1 nu: y = 1700/17e6 = 1e-4 -> u_fin=0.1, u_tvc=0.4
        CHECK_NEAR(u.get(fin), 0.1, 1e-4);
        CHECK_NEAR(u.get(tvc), 0.4, 1e-4);
        // ...and the achieved moment matches the demand.
        CHECK_NEAR(1000.0 * u.get(fin) + 4000.0 * u.get(tvc), 1700.0, 1e-3);
    }

    // --- No authority at ALL (pad step of a pure-TVC vehicle): no throw,
    //     outputs untouched ---
    {
        ChannelTable t;
        const ChannelHandle g = t.add({"tvc_pitch", ChannelKind::Gimbal, -0.1, 0.1});
        const ControlEffect fx[] = { { g, Vector3() } };   // thrust lag -> zero column
        ChannelValues u(t);
        alloc.allocate({ Vector3(), Vector3(0.0, 5000.0, 0.0) }, fx, 1, t, u);
        CHECK_NEAR(u.get(g), 0.0, 1e-15);
        alloc.allocate({ Vector3(), Vector3(0.0, 5000.0, 0.0) }, nullptr, 0, t, u);
        CHECK_NEAR(u.get(g), 0.0, 1e-15);
    }

    // --- Zero-authority axis: damped inverse gives zeros, never NaN ---
    {
        ChannelTable t;
        const ChannelHandle e = t.add({"elevator", ChannelKind::Surface, -0.5, 0.5});
        const ControlEffect fx[] = { { e, Vector3(0.0, 1000.0, 0.0) } };
        ChannelValues u(t);
        alloc.allocate({ Vector3(), Vector3(500.0, 0.0, 0.0) }, fx, 1, t, u);   // roll demand
        CHECK(std::isfinite(u.get(e)));
        CHECK_NEAR(u.get(e), 0.0, 1e-9);
    }

    // --- Declared channel limits clamp the allocation ---
    {
        ChannelTable t;
        const ChannelHandle e = t.add({"elevator", ChannelKind::Surface, -0.1, 0.1});
        const ControlEffect fx[] = { { e, Vector3(0.0, 1000.0, 0.0) } };
        ChannelValues u(t);
        alloc.allocate({ Vector3(), Vector3(0.0, 5000.0, 0.0) }, fx, 1, t, u);
        CHECK_NEAR(u.get(e), 0.1, 1e-9);
    }

    // --- Force allocation (6-DOF path): a lateral-force effector ---
    {
        ChannelTable t;
        const ChannelHandle j = t.add({"tvc_yaw", ChannelKind::Gimbal, -0.5, 0.5});
        ControlEffect fx[1];
        fx[0].channel = j;
        fx[0].dForce  = Vector3(0.0, 2000.0, 0.0);   // +Y force per unit deflection
        fx[0].dMoment = Vector3();
        ChannelValues u(t);
        alloc.allocate({ Vector3(0.0, 500.0, 0.0), Vector3() }, fx, 1, t, u);
        CHECK(std::isfinite(u.get(j)));
        CHECK_NEAR(u.get(j), 0.25, 1e-6);            // 500 / 2000
    }

    // --- Combined force+moment demand solved together (6x6) ---
    {
        ChannelTable t;
        const ChannelHandle a = t.add({"chan_a", ChannelKind::Surface, -2.0, 2.0});
        const ChannelHandle b = t.add({"chan_b", ChannelKind::Surface, -2.0, 2.0});
        ControlEffect fx[2];
        fx[0].channel = a; fx[0].dForce = Vector3(100.0, 0.0, 0.0); fx[0].dMoment = Vector3(0.0, 50.0, 0.0);
        fx[1].channel = b; fx[1].dForce = Vector3();                fx[1].dMoment = Vector3(0.0, 80.0, 0.0);
        ChannelValues u(t);
        alloc.allocate({ Vector3(30.0, 0.0, 0.0), Vector3(0.0, 100.0, 0.0) }, fx, 2, t, u);
        const double Fx = 100.0 * u.get(a);
        const double My = 50.0 * u.get(a) + 80.0 * u.get(b);
        CHECK_NEAR(Fx, 30.0,  1e-1);   // low damping -> achieved ~= demanded
        CHECK_NEAR(My, 100.0, 1e-1);
    }

    // --- RocketAero effectiveness matches its derivatives ---
    {
        const json::Value cfg = json::Value::parse(R"({
            "sref_m2": 0.05, "lref_m": 3.0, "dref_m": 0.25,
            "ca0": 0.3, "cna": 10.0, "cma": -12.0, "cmq": -120.0,
            "clp": -6.0, "cnde": 1.5, "cmde": -8.0, "clda": 3.0
        })");
        auto aero = RocketAero::fromJson(cfg);
        ChannelTable t;
        aero->declareChannels(t);
        AirData air;
        air.qbar = 10000.0;
        ControlEffect fx[8];
        const int n = aero->controlEffectiveness(air, std::nan(""), fx, 8);
        CHECK(n == 3);
        const double qS = 10000.0 * 0.05;
        CHECK_NEAR(fx[0].dMoment.y, -8.0 * qS * 3.0, 1e-6);    // elevator, cmde
        CHECK_NEAR(fx[1].dMoment.z, -8.0 * qS * 3.0, 1e-6);    // rudder, cndr=cmde mirror
        CHECK_NEAR(fx[2].dMoment.x, 3.0 * qS * 0.25, 1e-6);    // aileron, clda
    }

    // --- AircraftAero effectiveness matches finite differences of compute() ---
    {
        const json::Value cfg = json::Value::parse(R"({
            "sref_m2": 16.2, "cbar_m": 1.5, "bspan_m": 11.0,
            "cla": 5.0, "cd0": 0.03,
            "cmde": -1.2, "clde": 0.4, "cma": -0.9,
            "clda": 0.08, "cnda": 0.01,
            "cndr": -0.07, "cldr": 0.015, "cydr": 0.1
        })");
        auto aero = AircraftAero::fromJson(cfg);
        ChannelTable t;
        aero->declareChannels(t);
        State s;
        AirData air;
        air.airspeed = 60.0;
        air.qbar = 2200.0;
        air.alpha = 0.05;
        air.beta = 0.02;
        ControlEffect fx[8];
        const int n = aero->controlEffectiveness(air, std::nan(""), fx, 8);
        CHECK(n == 3);
        for (int k = 0; k < n; ++k) {
            const Vector3 fd = fdMomentSlope(*aero, t, fx[k].channel, s, air);
            CHECK_NEAR(fx[k].dMoment.x, fd.x, 1e-6);
            CHECK_NEAR(fx[k].dMoment.y, fd.y, 1e-6);
            CHECK_NEAR(fx[k].dMoment.z, fd.z, 1e-6);
        }
    }

    // --- F16Aero effectiveness matches finite differences of the real tables
    //     (xcg NaN -> columns about the table reference, same as compute) ---
    {
        const json::Value cfg = json::Value::parse(R"({"dir": "vehicles/f16"})");
        auto aero = F16Aero::fromJson(cfg, ".");
        ChannelTable t;
        aero->declareChannels(t);
        State s;
        AirData air;
        air.airspeed = 150.0;
        air.qbar = 0.5 * 1.225 * 150.0 * 150.0;
        air.alpha = 0.05;
        air.velocityBody = Vector3(149.6, 3.0, 7.5);   // beta = asin(3/150)
        ControlEffect fx[8];
        const int n = aero->controlEffectiveness(air, std::nan(""), fx, 8);
        CHECK(n == 3);
        for (int k = 0; k < n; ++k) {
            const Vector3 fd = fdMomentSlope(*aero, t, fx[k].channel, s, air);
            CHECK_NEAR(fx[k].dMoment.x, fd.x, 1e-3);
            CHECK_NEAR(fx[k].dMoment.y, fd.y, 1e-3);
            CHECK_NEAR(fx[k].dMoment.z, fd.z, 1e-3);
        }
        // The F-16's inverted aileron convention must come out of the DATA:
        // +aileron -> LEFT roll (negative dMx), and +elevator -> nose down.
        CHECK(fx[1].dMoment.x < 0.0);   // aileron column
        CHECK(fx[0].dMoment.y < 0.0);   // elevator column
    }

    // --- Propulsor gimbal effectiveness: L*T on pitch and yaw ---
    {
        auto model = std::make_unique<FixedThrust>();
        FixedThrust* thrust = model.get();
        Propulsor prop(std::move(model), Propulsor::Gimbal{6.0, 0.15});
        ChannelTable t;
        prop.declareChannels(t);
        State s;
        AirData air;
        const ComponentContext ctx{ s, air, 0.0, 0.01, 3.5 };
        ControlEffect fx[8];
        CHECK(prop.controlEffectiveness(ctx, fx, 8) == 0);   // no thrust yet
        thrust->T = 10000.0;
        prop.computeWrench(ctx, ChannelValues(t), nullptr);                 // sets lastThrust_
        const int n = prop.controlEffectiveness(ctx, fx, 8);
        CHECK(n == 2);
        CHECK_NEAR(fx[0].dMoment.y, 2.5 * 10000.0, 1e-6);
        CHECK_NEAR(fx[1].dMoment.z, 2.5 * 10000.0, 1e-6);
    }

    std::printf("test_allocator: all checks passed\n");
    return 0;
}
