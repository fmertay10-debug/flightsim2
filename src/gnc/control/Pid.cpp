#include "gnc/control/Pid.h"

#include <algorithm>

double Pid::update(double error, double rate, double dt) {
    const double raw = kp_ * error + ki_ * integ_.value() - kd_ * rate;
    const double out = std::clamp(raw, outMin_, outMax_);

    // Conditional integration: freeze only when the clamped output is saturated
    // AND the error would push it further into saturation. (Equivalent, case for
    // case, to the previous open-coded "integrate unless saturated-and-pushing".)
    const bool pushingIntoSat = (raw > outMax_ && error >= 0.0) ||
                                (raw < outMin_ && error <= 0.0);
    integ_.accumulate(error, dt, /*hold=*/pushingIntoSat);
    return out;
}
