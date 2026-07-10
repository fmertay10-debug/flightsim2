#pragma once

#include "component/AeroModel.h"
#include "io/Json.h"

// Axisymmetric rocket/missile aero: linear derivative model in missile
// coefficient convention. Derivatives per radian; rates normalized.
//
//   CA = ca0                                    axial force,  Fx = -CA qS
//   CN = cna*alpha + cnde*de                    normal force, Fz = -CN qS
//   CY = cyb*beta + cydr*dr                     side force,   Fy = +CY qS
//   Cl = clp*phat + clda*da                     roll,  Mx = Cl qS dref
//   Cm = cma*alpha + cmq*qhat + cmde*de         pitch, My = Cm qS lref
//   Cn = cnb*beta  + cnr*rhat + cndr*dr         yaw,   Mz = Cn qS lref
//
// Axisymmetry defaults (config may override): cyb = -cna, cnb = -cma,
// cnr = cmq, cndr = cmde, cydr = -cnde. Stable vehicle: cma < 0.
class RocketAero : public AeroModel {
public:
    struct Derivatives {
        double ca0 = 0;
        double cna = 0, cnde = 0;
        double cyb = 0, cydr = 0;
        double clp = 0, clda = 0;
        double cma = 0, cmq = 0, cmde = 0;
        double cnb = 0, cnr = 0, cndr = 0;
    };

    RocketAero(const AeroReference& ref, const Derivatives& d)
        : ref_(ref), d_(d) {}

    // Builder for aero::Factory: reads the "aero" config block of a vehicle file.
    static std::unique_ptr<AeroModel> fromJson(const json::Value& cfg);

    // Declares only fins with authority (nonzero control derivatives) -- a
    // TVC airframe with no fin derivatives declares no fin channels at all.
    void declareChannels(ChannelTable& table) override;

    AeroForces compute(const State& state, const AirData& air,
                       const ChannelValues& control) const override;

    // Fin moment sensitivities from the control derivatives (already about
    // the CG -- derivative models report there).
    int controlEffectiveness(const AirData& air, double xcg,
                             ControlEffect* out, int maxOut) const override;

private:
    AeroReference ref_;   // cbar = body length (pitch/yaw), bref = diameter (roll)
    Derivatives   d_;
    ChannelHandle elevator_, aileron_, rudder_;
};
