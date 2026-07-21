#pragma once

#include <memory>
#include <string>

#include "component/AeroReference.h"
#include "component/ForceComponent.h"
#include "io/Json.h"
#include "math/LookupTable1D.h"
#include "math/LookupTable2D.h"

// Stevens & Lewis F-16 low-fidelity aerodynamics (NASA TP-1538), ported to
// SI/radian CSVs in vehicles/f16/ (see its README for lineage). Static tables
// are functions of (alpha, elevator) or (alpha, beta); damping derivatives are
// 1-D in alpha. No Mach dependence -- credible subsonic only (~M 0.6).
//
// Direct body-axis coefficients (X fwd, Y right, Z DOWN): force = qS*(CX,CY,CZ)
// with NO leading minus (CZ is negative when lifting). Sideslip uses the S&L
// definition beta = asin(v/V). Moments are reported about the table reference
// CG (xcgr, a fraction of cbar); the Entity transfers them to the actual CG
// via momentReferenceStation() -- so a variable-CG aircraft needs no change.
struct F16Tables {
    LookupTable2D cx, cm;                       // (alpha, elevator)
    LookupTable2D cl, cn;                       // (alpha, beta)
    LookupTable2D dlda, dldr, dnda, dndr;       // control derivatives (alpha, beta)
    LookupTable1D cz;                           // static normal force (alpha)
    LookupTable1D cxq, cyr, cyp, czq,           // damping derivatives (alpha),
                  clr, clp, cmq, cnr, cnp;      // per radian
};

class F16Aero : public ForceComponent {
public:
    F16Aero(F16Tables tables, AeroReference ref, double xcgrCbar);

    static std::unique_ptr<F16Aero> fromJson(const json::Value& cfg,
                                             const std::string& baseDir);

    // The S&L tables always carry elevator/aileron/rudder authority.
    void declareChannels(ChannelTable& table) override;

    Wrench computeWrench(const ComponentContext& ctx, const ChannelValues& u,
                         const double* /*x: stateless*/) const override;

    // Tables are referenced to xcgr (fraction of cbar). As a station in meters
    // (increasing aft, MAC-LE datum) that is xcgr*cbar -- matching the mass
    // model's xcg = (cg fraction)*cbar.
    double momentReferenceStation() const override { return xcgr_ * ref_.cbar; }

    // Slope cards for allocation, CG-referenced: elevator by central-differencing
    // the CM table about zero deflection (+ the analytic CZ-per-elevator term
    // transferred over the xcg-xcgr arm); aileron/rudder from the S&L control-
    // derivative tables at the current (alpha, beta), de-normalized to per-rad
    // (their inputs are fractions of the 20/30 deg travels). The F-16's
    // INVERTED aileron (+da -> LEFT roll) comes out of the dlda data.
    int controlEffectiveness(const ComponentContext& ctx,
                             ControlEffect* out, int maxOut) const override;

private:
    F16Tables     t_;
    AeroReference ref_;
    double        xcgr_;   // table reference CG [fraction of cbar]
    ChannelHandle elevator_, aileron_, rudder_;
};
