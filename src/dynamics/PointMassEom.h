#pragma once

#include "dynamics/EquationsOfMotion.h"

// 3-DOF point mass: translation only. Moments are ignored; attitude and
// angular rate pass through unchanged. Useful for trajectory-only studies.
//
// No shipped config uses this EOM (all use six_dof or kinematic); kept
// deliberately (decision 2026-07-19) as the simplest EOM example --
// registered as "point_mass".
class PointMassEom : public EquationsOfMotion {
public:
    State solve(const State& state,
                const Vector3& forceBody, const Vector3& momentBody,
                double mass, const Matrix3x3& inertia, double dt) const override;
};
