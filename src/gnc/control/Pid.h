#pragma once

#include "gnc/control/Integrator.h"

// Small PID building block with output clamping and conditional-integration
// anti-windup (the integrator freezes while the output is saturated). Derivative
// is taken on a measured rate supplied by the caller (e.g. q from the gyro)
// rather than differentiating the error -- standard flight-control practice,
// avoids derivative kick. The integral + anti-windup are delegated to the shared
// Integrator primitive (freeze when the clamped output saturates further).
class Pid {
public:
    Pid() = default;
    Pid(double kp, double ki, double kd, double outMin, double outMax)
        : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax) {}

    // error = command - measurement; rate = d(measurement)/dt.
    double update(double error, double rate, double dt);

    void reset() { integ_.reset(); }

private:
    double kp_ = 0.0, ki_ = 0.0, kd_ = 0.0;
    double outMin_ = -1.0, outMax_ = 1.0;
    Integrator integ_;
};
