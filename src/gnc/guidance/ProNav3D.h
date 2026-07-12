#pragma once

#include "gnc/guidance/GuidanceLaw.h"

// 3D proportional navigation, skid-to-turn: the LOS rate is split into an
// elevation channel (-> pitch command) and an azimuth channel (-> heading
// command), each integrated with navigation gain N. Ported from flightsim v1
// (validated on the 2D/3D intercept scenarios there).
//
// Inside blindRange the LOS rate diverges as range -> 0, so the law holds
// its final collision-course commands instead of chasing it into the clamps.
class ProNav3D : public GuidanceLaw {
public:
    ProNav3D(int targetId, double navGain,
             double maxPitch, double maxYaw, double blindRange)
        : targetId_(targetId), N_(navGain), maxPitch_(maxPitch),
          maxYaw_(maxYaw), blindRange_(blindRange) {}

    CommandSet update(const State& self, const WorldView& world,
                      double dt) override;

    double lastRange() const { return lastRange_; }

private:
    int    targetId_;
    double N_;
    double maxPitch_;      // pitch command clamp [rad]
    double maxYaw_;        // heading command clamp about the seed [rad]
    double blindRange_;    // [m]

    bool   seeded_   = false;
    double gammaCmd_ = 0.0;
    double psiCmd_   = 0.0;
    double psiSeed_  = 0.0;
    double lastRange_ = 1e30;
};
