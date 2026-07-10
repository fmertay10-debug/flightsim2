#include "dynamics/KinematicEom.h"

#include <cmath>

State KinematicEom::solve(const State& state,
                          const Vector3&, const Vector3&,
                          double, const Matrix3x3&, double dt) const {
    State next = state;

    if (turnRate_ != 0.0) {
        // Rotate the horizontal velocity by turnRate*dt; keep vertical rate.
        const double dpsi = turnRate_ * dt;
        const double c = std::cos(dpsi), s = std::sin(dpsi);
        const Vector3 v = state.velocity;
        next.velocity = { c * v.x - s * v.y,
                          s * v.x + c * v.y,
                          v.z };
        // Keep the attitude aligned with the (turning) velocity heading.
        const Vector3 euler = state.eulerAngles();
        next.attitude = Quaternion::fromEuler(euler.x, euler.y, euler.z + dpsi);
    }

    next.position = state.position + state.velocity * dt;
    next.time = state.time + dt;
    return next;
}
