#pragma once

#include "dynamics/EquationsOfMotion.h"

// Full rigid-body 6-DOF equations of motion.
//  - Translation integrated in the inertial (NED) frame.
//  - Rotation: full Euler equation I*omega_dot = M - omega x (I*omega)
//    (captures gyroscopic coupling; works for non-diagonal inertia).
//  - Attitude: quaternion kinematics, renormalized every step (mandatory).
class SixDofEom : public EquationsOfMotion {
public:
    State solve(const State& state,
                const Vector3& forceBody, const Vector3& momentBody,
                double mass, const Matrix3x3& inertia, double dt) const override;
};
