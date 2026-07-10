#include "control/Pid.h"

#include <algorithm>

double Pid::update(double error, double rate, double dt) {
    const double raw = kp_ * error + ki_ * integral_ - kd_ * rate;
    const double out = std::clamp(raw, outMin_, outMax_);

    // Conditional integration: only integrate when not pushing further into
    // saturation, so the integrator cannot wind up.
    const bool saturated = (raw != out);
    if (!saturated || (raw > outMax_ && error < 0.0) || (raw < outMin_ && error > 0.0))
        integral_ += error * dt;

    return out;
}
