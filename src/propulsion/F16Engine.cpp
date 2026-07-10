#include "propulsion/F16Engine.h"

#include <algorithm>
#include <utility>

#include "io/CsvReader.h"

F16Engine::F16Engine(LookupTable2D idle, LookupTable2D mil, LookupTable2D mx,
                     double power0)
    : idle_(std::move(idle)), mil_(std::move(mil)), max_(std::move(mx)),
      power_(std::clamp(power0, 0.0, 100.0)) {}

double F16Engine::commandedPower(double throttle) {
    const double thr = std::clamp(throttle, 0.0, 1.0);
    return (thr <= 0.77) ? 64.94 * thr : 217.38 * thr - 117.38;
}

double F16Engine::rateGain(double dp) {
    if (dp <= 25.0) return 1.0;      // small change: 1 s time constant
    if (dp >= 50.0) return 0.1;      // big spool-up: slow (10 s)
    return 1.9 - 0.036 * dp;         // linear bridge
}

double F16Engine::powerRate(double p, double pcmd) {
    // Afterburner gate at P = 50: commands crossing it route through an
    // intermediate setpoint (60 up, 40 down) so spool and light-off sequence.
    double target, gain;
    if (pcmd >= 50.0) {
        if (p >= 50.0) { target = pcmd; gain = 5.0; }
        else           { target = 60.0; gain = rateGain(target - p); }
    } else {
        if (p >= 50.0) { target = 40.0; gain = 5.0; }
        else           { target = pcmd; gain = rateGain(target - p); }
    }
    return gain * (target - p);
}

double F16Engine::blend(double p, double alt, double mach) const {
    const double ti = idle_.eval(alt, mach);
    const double tm = mil_.eval(alt, mach);
    if (p < 50.0) return ti + (tm - ti) * p * 0.02;
    return tm + (max_.eval(alt, mach) - tm) * (p - 50.0) * 0.02;
}

double F16Engine::thrust(const PropulsionContext& ctx) {
    const double pcmd = commandedPower(ctx.throttle);
    power_ = std::clamp(power_ + powerRate(power_, pcmd) * ctx.dt, 0.0, 100.0);
    return blend(power_, ctx.altitude, ctx.mach);
}

double F16Engine::steadyThrust(double throttle, double altitude, double mach) const {
    return blend(commandedPower(throttle), altitude, mach);
}

F16Engine F16Engine::fromCsv(const std::string& dir) {
    const csv::Table t = csv::read(dir + "/thrust.csv");
    return F16Engine(csv::buildTable2D(t, "alt_m", "mach", "idle_N"),
                     csv::buildTable2D(t, "alt_m", "mach", "mil_N"),
                     csv::buildTable2D(t, "alt_m", "mach", "max_N"));
}
