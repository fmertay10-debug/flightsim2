#pragma once

#include "component/ForceComponent.h"
#include "core/Channel.h"
#include "gnc/control/WrenchCommand.h"
#include "math/Vector3.h"

// Control allocation: distribute a desired body WrenchCommand (force + moment
// about the CG) over the channels the vehicle's components declare, using their
// queried effectiveness columns (B = d[F;M]/du). Damped minimum-norm least
// squares:
//
//   u = B^T (B B^T + lambda*I)^-1  nu
//
// so redundant effectors share the load in proportion to their authority --
// on the pad the fin columns are ~zero (qbar ~ 0) and the gimbal does the
// work; as qbar builds the fins take over; after burnout the gimbal column
// dies. No mode switching: the blend falls out of B changing with flight
// condition. Axes with no authority at all get (damped) zero, not a blow-up.
//
// The wrench is up to 6-dimensional. When no force is demanded AND no component
// reports force effectiveness (the attitude-only case), allocation runs on the
// 3x3 moment Gram matrix exactly as before -- byte-identical. Force enters the
// 6x6 path only when a WrenchCommand or a component actually carries it.
//
// Each channel is clamped to its DECLARED limits from the ChannelTable (the
// mechanical stop the owning component stated). Saturation redistribution
// (daisy-chaining) is future work; the ActuatorBank clamps again downstream.
class Allocator {
public:
    // damping: relative ridge on B*B^T (fraction of its mean authority).
    explicit Allocator(double damping = 1e-6) : damping_(damping) {}

    // Writes the allocated values into `out` through the effects' handles.
    void allocate(const WrenchCommand& desired,
                  const ControlEffect* effects, int count,
                  const ChannelTable& table, ChannelValues& out) const;

private:
    double damping_;
};
