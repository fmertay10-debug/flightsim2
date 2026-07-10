#pragma once

#include "core/State.h"
#include "math/Matrix3x3.h"
#include "math/Vector3.h"

// Strategy: propagate a State one timestep under given body-frame loads.
// Pure function of its inputs -- implementations hold no mutable state,
// so the two-phase (snapshot -> propagate -> commit) sim loop stays deterministic.
//
// Integration is forward Euler with a small fixed dt (aero damping dominates;
// quaternion renormalized every step).
class EquationsOfMotion {
public:
    virtual ~EquationsOfMotion() = default;

    virtual State solve(const State&     state,
                        const Vector3&   forceBody,    // [N]  body frame
                        const Vector3&   momentBody,   // [Nm] body frame
                        double           mass,         // [kg]
                        const Matrix3x3& inertia,      // body frame [kg m^2]
                        double           dt) const = 0;
};
