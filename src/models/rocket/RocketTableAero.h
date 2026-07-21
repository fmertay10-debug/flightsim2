#pragma once

#include <memory>
#include <string>

#include "component/AeroReference.h"
#include "component/ForceComponent.h"
#include "io/Json.h"
#include "math/LookupTable2D.h"

// DATCOM table-lookup rocket aero: bilinear interpolation over the full
// (alpha, Mach) envelope instead of point derivatives -- coefficients vary
// with flight condition, valid to alpha = +/-180 deg.
//
// Data comes from two tidy CSVs produced by tools/datcom_export.py:
//   aero_tables.csv    alpha_rad, mach, CN, CA, CM, CMQ, CNR, CLP, CNB, CYB
//   control_tables.csv delta_rad, mach, dCM_sym, dCL_sym, Cl_roll
//
// Conventions ported verbatim from flightsim v1 LinearAero (validated there):
//   - forces map as Fx=-CA qS, Fy=-CY qS, Fz=-CN qS
//   - elevator fin lift (dCL_sym) joins CN; by cruciform symmetry the same
//     table gives the rudder side force and the dCM table the rudder moment
//   - cbar = BODY LENGTH (pitch/yaw reference), bref = span; cnb and the
//     reused pitch control table are per-cbar, so the yaw channel applies a
//     cbar/bref conversion. cnr is natively per-bref: NOT scaled. Do not "fix".
class RocketTableAero : public ForceComponent {
public:
    struct Tables {
        // Static + damping vs (alpha [rad], Mach).
        LookupTable2D cn, ca, cm, cmq, cnr, clp, cnb, cyb;
        // Control vs (deflection [rad], Mach).
        LookupTable2D dcmCtrl;   // pitch moment increment (also rudder, mirrored)
        LookupTable2D dclCtrl;   // fin lift increment (elevator CN / rudder CY)
        LookupTable2D clRoll;    // roll moment from differential deflection
    };

    // xrefStation: moment reference station [m from nose]; NaN = report about
    // CG (no transfer). When set, the Entity transfers the moment to the
    // current CG using the mass model's xcg -- this is how CG travel changes
    // the effective static margin of a burning rocket.
    RocketTableAero(const AeroReference& ref, Tables tables, double xrefStation)
        : ref_(ref), t_(std::move(tables)), xref_(xrefStation) {}

    // Builder for component::Factory. Reads sref_m2/cbar_m/bref_m, the
    // tables_csv/control_csv paths (relative to baseDir), and optional xref_m.
    static std::unique_ptr<RocketTableAero> fromJson(const json::Value& cfg,
                                                     const std::string& baseDir);

    // DATCOM control tables always carry all three fin channels.
    void declareChannels(ChannelTable& table) override;

    Wrench computeWrench(const ComponentContext& ctx, const ChannelValues& u,
                         const double* /*x: stateless*/) const override;

    double momentReferenceStation() const override { return xref_; }

    // Fin sensitivities by central-differencing the control tables about zero
    // deflection at the current Mach, transferred from xref to the CG.
    int controlEffectiveness(const ComponentContext& ctx,
                             ControlEffect* out, int maxOut) const override;

private:
    AeroReference ref_;
    Tables        t_;
    double        xref_;
    ChannelHandle elevator_, aileron_, rudder_;
};
