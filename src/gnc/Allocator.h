#pragma once

#include "component/ForceComponent.h"
#include "core/Channel.h"
#include "math/Vector3.h"

// Control allocation: distribute a desired body moment over the channels the
// vehicle's components declare, using their queried effectiveness columns
// (B = dM/du). Damped minimum-norm least squares:
//
//   u = B^T (B B^T + lambda*I)^-1  nu
//
// so redundant effectors share the load in proportion to their authority --
// on the pad the fin columns are ~zero (qbar ~ 0) and the gimbal does the
// work; as qbar builds the fins take over; after burnout the gimbal column
// dies. No mode switching: the blend falls out of B changing with flight
// condition. Axes with no authority at all get (damped) zero, not a blow-up.
//
// Each channel is clamped to its DECLARED limits from the ChannelTable (the
// mechanical stop the owning component stated). Saturation redistribution
// (daisy-chaining) is future work; the ActuatorBank clamps again downstream.
class Allocator {
public:
    // damping: relative ridge on B*B^T (fraction of its mean diagonal).
    explicit Allocator(double damping = 1e-6) : damping_(damping) {}

    // Writes the allocated values into `out` through the effects' handles.
    void allocate(const Vector3& desiredMoment,
                  const ControlEffect* effects, int count,
                  const ChannelTable& table, ChannelValues& out) const;

private:
    double damping_;
};
