#pragma once

#include <cmath>
#include <vector>

#include "core/Channel.h"
#include "vehicle/Vehicle.h"

// The offline-design seam ADR-0003 was built for: evaluate the vehicle's PURE
// dynamics f(x,u) -- the real ForceComponents at a frozen flight condition,
// external component state at its initial values -- and extract the
// short-period pitch plant by central differences. This replaces the
// hand-assembled DATCOM-derivative plant in tools/design_autopilot.py: any
// vehicle the sim can fly (derivative aero, DATCOM tables, wind-tunnel
// tables), the tool can now linearize, because it IS the sim's force stack.
//
// Reduced dynamics linearized (about alpha = de = 0, or about level-flight
// trim with --trim):
//   alpha_dot = q + Fz_body(alpha, de) / (m V)      [aero+propulsion, no
//   q_dot     = My_cg(alpha, q, de) / Iyy            gravity: constant terms
//                                                    drop out of differences;
//                                                    gravity enters the TRIM
//                                                    residual only]
namespace design {

struct PlantPoint {
    double mach = 0.0, V = 0.0, qbar = 0.0;
    double alphaTrim = 0.0, deTrim = 0.0;   // [rad]; zero when not trimmed
    bool   trimmed = false;                 // trim requested AND converged
    // Short-period sensitivities (the designer's plant entries):
    double Za = 0.0, Zde = 0.0;             // d(alpha_dot)/d(alpha, de) [1/s, 1/s]
    double Ma = 0.0, Mq = 0.0, Mde = 0.0;   // d(q_dot)/d(alpha, q, de)
    // d(alpha_dot)/dq = 1 + Zq/V: pitch rate makes lift (CZq), which feeds
    // alpha_dot -- 0.905 for the F-16, not 1. The proper 2-state A is
    // [[Za, Aq], [Ma, Mq]]; aero with no q force dependence gives exactly 1.
    double Aq = 1.0;
};

struct LinearizerOptions {
    double altitude = 1000.0;   // design altitude [m]
    double time     = 0.0;      // component evaluation time (burn clock) [s]
    bool   trim     = false;    // Newton-trim (alpha, de) for level flight
    // NaN = take from the vehicle's mass model at `time`.
    double mass = std::nan(""), iyy = std::nan(""), xcg = std::nan("");
};

class Linearizer {
public:
    // The vehicle and its declared channel table (loader-built); both must
    // outlive the Linearizer. Non-owning by design: the caller (CLI, test)
    // owns the vehicle it loaded.
    Linearizer(Vehicle& vehicle, const ChannelTable& channels,
               const LinearizerOptions& opt);

    // Plant at one Mach number (V from ISA sound speed at the altitude).
    PlantPoint at(double mach) const;

private:
    struct Rates { double alphaDot, qDot; };
    // Aero+propulsion rates at the frozen condition (no gravity; see header).
    Rates rates(double V, double alpha, double q, double de) const;
    // Body-frame Fz and CG-referenced My from the component stack.
    void wrench(double V, double alpha, double q, double de,
                double& Fz, double& My) const;

    Vehicle&            veh_;
    const ChannelTable& channels_;
    LinearizerOptions   opt_;
    ChannelHandle       elevator_;
    double mass_ = 0.0, iyy_ = 0.0, xcg_ = std::nan("");
    std::vector<double> compState_;         // frozen external component state
    std::vector<int>    compOffsets_;
};

} // namespace design
