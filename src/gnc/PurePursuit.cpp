#include "gnc/PurePursuit.h"

#include <cmath>

CommandSet PurePursuit::update(const State& self, const WorldView& world,
                               double /*dt*/) {
    if (world.isAlive(targetId_)) {
        const Vector3 r = world.stateOf(targetId_).position - self.position;
        const double rho = std::hypot(r.x, r.y);
        lastHeading_ = std::atan2(r.y, r.x);
        lastPitch_   = std::atan2(-r.z, rho);
    }
    CommandSet out;
    out.pitch   = lastPitch_;
    out.heading = lastHeading_;
    return out;
}
