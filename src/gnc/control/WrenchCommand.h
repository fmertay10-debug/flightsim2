#pragma once

#include "math/Vector3.h"

// The virtual control output of a control law (ADR-0004): a desired net wrench
// on the body about the CG -- force + moment -- before any decision about which
// effectors produce it. The Allocator maps it onto channel commands using each
// ForceComponent's queried effectiveness at the current flight condition.
//
// Structurally a force+moment pair like core/Wrench, but semantically distinct:
// a Wrench is what a component PRODUCES; a WrenchCommand is what control DEMANDS.
struct WrenchCommand {
    Vector3 force;    // desired body force about the CG [N]
    Vector3 moment;   // desired body moment about the CG [Nm]
};
