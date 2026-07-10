#pragma once

#include "guidance/GuidanceLaw.h"

// Pure pursuit: always point at the target's CURRENT position -- pitch
// command = LOS elevation, heading command = LOS azimuth. Simple and robust;
// ends in a tail chase against moving targets (use ProNav3D to lead).
class PurePursuit : public GuidanceLaw {
public:
    explicit PurePursuit(int targetId) : targetId_(targetId) {}

    CommandSet update(const State& self, const WorldView& world,
                      double dt) override;

private:
    int    targetId_;
    double lastPitch_   = 0.0;
    double lastHeading_ = 0.0;
};
