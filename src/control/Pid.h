#pragma once

// Small PID building block with output clamping and conditional-integration
// anti-windup (the integrator freezes while the output is saturated).
// Derivative is taken on a measured rate supplied by the caller (e.g. q from
// the gyro) rather than differentiating the error -- standard flight-control
// practice, avoids derivative kick.
class Pid {
public:
    Pid() = default;
    Pid(double kp, double ki, double kd, double outMin, double outMax)
        : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax) {}

    // error = command - measurement; rate = d(measurement)/dt.
    double update(double error, double rate, double dt);

    void reset() { integral_ = 0.0; }

private:
    double kp_ = 0.0, ki_ = 0.0, kd_ = 0.0;
    double outMin_ = -1.0, outMax_ = 1.0;
    double integral_ = 0.0;
};
