#pragma once

#include "math/Vector3.h"
#include "math/Quaternion.h"

// The complete kinematic state of a vehicle at one instant.
// Shared currency across dynamics, control, and logging -- belongs to no one module.
//
// Conventions (locked):
//   - Inertial frame: NED (north, east, down). Altitude = -position.z.
//   - Body frame: x forward, y right, z down.
//   - attitude: quaternion, INERTIAL -> BODY (single source of truth, no gimbal lock).
//   - angularRate: body frame (p, q, r) [rad/s].
struct State {
    double     time = 0.0;        // simulation time [s]
    Vector3    position;          // inertial NED [m]
    Vector3    velocity;          // inertial NED [m/s]
    Quaternion attitude;          // inertial -> body (defaults to identity)
    Vector3    angularRate;       // body (p, q, r) [rad/s]

    double altitude() const { return -position.z; }

    // Human-readable attitude: (phi, theta, psi) in radians. Derived view only.
    Vector3 eulerAngles() const { return attitude.toEuler(); }
};
