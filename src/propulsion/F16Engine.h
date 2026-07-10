#pragma once

#include <string>

#include "math/LookupTable2D.h"
#include "propulsion/PropulsionModel.h"

// Stevens & Lewis F-16 turbofan (Appendix A): thrust from three (altitude,
// Mach) tables -- idle / military / max -- blended by an internal power state
// P in [0,100] % with its own first-order dynamics:
//
//   Pcmd = tgear(throttle):  64.94*thr           (thr <= 0.77)
//                            217.38*thr - 117.38  (thr >  0.77)
//   Pdot = powerRate(P, Pcmd)   [afterburner gate at P = 50: crossings route
//          through 60 (up) / 40 (down); rate 5/s in afterburner, 0.1..1/s
//          spool below]
//   T    = idle + (mil-idle) * P/50     (P <  50)
//          mil  + (max-mil) * (P-50)/50 (P >= 50)
//
// Stateful and self-integrating: P advances by ctx.dt each thrust() call.
class F16Engine : public PropulsionModel {
public:
    F16Engine(LookupTable2D idle, LookupTable2D mil, LookupTable2D mx,
              double power0 = 0.0);

    static F16Engine fromCsv(const std::string& dir);   // reads f16/thrust.csv

    double thrust(const PropulsionContext& ctx) override;   // advances P

    double power() const { return power_; }
    // Steady-state thrust at a held throttle/condition (P -> tgear), no state.
    double steadyThrust(double throttle, double altitude, double mach) const;
    static double commandedPower(double throttle);          // tgear [%]

private:
    static double rateGain(double dp);
    static double powerRate(double p, double pcmd);
    double blend(double p, double alt, double mach) const;

    LookupTable2D idle_, mil_, max_;
    double        power_;
};
