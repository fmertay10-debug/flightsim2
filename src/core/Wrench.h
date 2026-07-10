#pragma once

#include "math/Vector3.h"

// A force + moment pair in the BODY frame, moment taken about the CG.
// The shared currency for anything that pushes on the vehicle -- effectors
// return one, the Entity sums them.
struct Wrench {
    Vector3 force;    // [N]
    Vector3 moment;   // [Nm], about the CG

    Wrench operator+(const Wrench& r) const {
        return { force + r.force, moment + r.moment };
    }
};
