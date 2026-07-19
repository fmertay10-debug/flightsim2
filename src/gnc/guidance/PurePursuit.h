#pragma once

#include "gnc/guidance/GuidanceLaw.h"

// Pure pursuit: always point at the target's CURRENT position -- pitch
// command = LOS elevation, heading command = LOS azimuth. Simple and robust;
// ends in a tail chase against moving targets (use ProNav3D to lead).
//
// No shipped scenario uses this law; kept deliberately (decision 2026-07-19)
// as the simplest guidance example -- registered as "pure_pursuit", covered
// by test_guidance.
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
