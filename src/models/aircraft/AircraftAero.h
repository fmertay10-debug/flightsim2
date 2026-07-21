#pragma once

#include <memory>

#include "component/AeroReference.h"
#include "component/ForceComponent.h"
#include "io/Json.h"

// Fixed-wing aircraft aero: classic linear stability-derivative model.
// All derivatives are per radian; rates are normalized (phat = p*b/2V, etc.).
//
//   CL = cl0 + cla*alpha + clq*qhat + clde*de          (lift)
//   CD = cd0 + kInduced*CL^2                           (drag polar)
//   CY = cyb*beta + cydr*dr                            (side force)
//   Cl = clb*beta + clp*phat + clr*rhat + clda*da + cldr*dr
//   Cm = cm0 + cma*alpha + cmq*qhat + cmde*de
//   Cn = cnb*beta + cnp*phat + cnr*rhat + cnda*da + cndr*dr
//
// Lift/drag act in stability axes and are rotated to the body frame by alpha.
class AircraftAero : public ForceComponent {
public:
    struct Derivatives {
        // Lift & drag
        double cl0 = 0, cla = 0, clq = 0, clde = 0;
        double cd0 = 0, kInduced = 0;
        // Side force
        double cyb = 0, cydr = 0;
        // Roll moment
        double clb = 0, clp = 0, clr = 0, clda = 0, cldr = 0;
        // Pitch moment
        double cm0 = 0, cma = 0, cmq = 0, cmde = 0;
        // Yaw moment
        double cnb = 0, cnp = 0, cnr = 0, cnda = 0, cndr = 0;
    };

    AircraftAero(const AeroReference& ref, const Derivatives& d)
        : ref_(ref), d_(d) {}

    // Builder for component::Factory: reads a "components" entry of a vehicle file.
    static std::unique_ptr<AircraftAero> fromJson(const json::Value& cfg);

    // Declares only the surfaces with authority (nonzero control derivatives).
    void declareChannels(ChannelTable& table) override;

    Wrench computeWrench(const ComponentContext& ctx, const ChannelValues& u,
                         const double* /*x: stateless*/) const override;

    // Analytic control slope cards for allocation (moments are already about
    // the CG -- derivative model, no reference transfer). Aileron and rudder
    // carry their roll/yaw cross terms (cnda, cldr) in the same column.
    int controlEffectiveness(const ComponentContext& ctx,
                             ControlEffect* out, int maxOut) const override;

private:
    AeroReference ref_;
    Derivatives   d_;
    ChannelHandle elevator_, aileron_, rudder_;
};
