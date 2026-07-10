#pragma once

#include "dynamics/EquationsOfMotion.h"

// Kinematic mover: ignores all forces, flies at constant velocity with an
// optional constant turn rate (heading rate, rad/s). Used for targets and
// traffic that other vehicles react to but that need no physics of their own.
class KinematicEom : public EquationsOfMotion {
public:
    explicit KinematicEom(double turnRate = 0.0) : turnRate_(turnRate) {}

    State solve(const State& state,
                const Vector3& forceBody, const Vector3& momentBody,
                double mass, const Matrix3x3& inertia, double dt) const override;

private:
    double turnRate_;   // psi_dot [rad/s], + = clockwise viewed from above
};
