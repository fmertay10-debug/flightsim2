#include "dynamics/PointMassEom.h"

State PointMassEom::solve(const State& state,
                          const Vector3& forceBody, const Vector3& /*momentBody*/,
                          double mass, const Matrix3x3& /*inertia*/, double dt) const {
    State next = state;

    const Vector3 forceInertial = state.attitude.rotateBack(forceBody);
    const Vector3 accel = forceInertial / mass;
    next.position = state.position + state.velocity * dt;
    next.velocity = state.velocity + accel * dt;

    next.time = state.time + dt;
    return next;
}
