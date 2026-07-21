#pragma once

#include <algorithm>
#include <limits>

// A running integral with the two anti-windup mechanisms the control laws use,
// composable in a single call:
//   * hold  -- skip accumulation this step (conditional integration; e.g. while
//              the downstream angular-acceleration demand or a PID output is
//              saturated), and
//   * limit -- clamp the accumulated magnitude to +/-limit (integral clamp).
// Both were previously open-coded three slightly different ways across the
// laws; this is the one primitive they now share.
//
// Read order is the caller's choice: laws that feed the OLD integral into their
// output and then update (Scheduled, Pid) read value() first; laws that update
// and then use the NEW integral (AllocatedAttitude, aircraft inner loop) use
// the value accumulate() returns.
class Integrator {
public:
    // Add error*dt unless held, then clamp to +/-limit (default: no clamp).
    double accumulate(double error, double dt, bool hold = false,
                      double limit = std::numeric_limits<double>::infinity()) {
        if (!hold) value_ += error * dt;
        value_ = std::clamp(value_, -limit, limit);   // no-op when limit = inf
        return value_;
    }

    double value() const { return value_; }
    void reset() { value_ = 0.0; }

private:
    double value_ = 0.0;
};
