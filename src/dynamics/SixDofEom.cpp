#include "dynamics/SixDofEom.h"

State SixDofEom::solve(const State& state,
                       const Vector3& forceBody, const Vector3& momentBody,
                       double mass, const Matrix3x3& inertia, double dt) const {
    State next = state;

    // ---- Translation (inertial frame) ----
    const Vector3 forceInertial = state.attitude.rotateBack(forceBody);
    const Vector3 accel = forceInertial / mass;
    next.position = state.position + state.velocity * dt;
    next.velocity = state.velocity + accel * dt;

    // ---- Rotation: I * omega_dot = M - omega x (I * omega) ----
    const Vector3 omega    = state.angularRate;
    const Vector3 gyro     = omega.cross(inertia * omega);
    const Vector3 omegaDot = inertia.inverse() * (momentBody - gyro);
    next.angularRate = omega + omegaDot * dt;

    // ---- Attitude kinematics: q_dot = 1/2 * q (x) (0, p, q, r) ----
    const Quaternion omegaQ(0.0, omega.x, omega.y, omega.z);
    const Quaternion qdot = state.attitude * omegaQ;

    Quaternion qn(state.attitude.w + 0.5 * dt * qdot.w,
                  state.attitude.x + 0.5 * dt * qdot.x,
                  state.attitude.y + 0.5 * dt * qdot.y,
                  state.attitude.z + 0.5 * dt * qdot.z);
    qn.normalize();                                      // MANDATORY every step
    next.attitude = qn;

    next.time = state.time + dt;
    return next;
}
