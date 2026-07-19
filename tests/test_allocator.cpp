#include <cmath>

#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "component/Propulsor.h"
#include "gnc/control/Allocator.h"
#include "io/Json.h"
#include "models/rocket/RocketAero.h"
#include "test_util.h"

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
