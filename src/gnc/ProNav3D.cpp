#include "gnc/ProNav3D.h"

#include <algorithm>
#include <cmath>

CommandSet ProNav3D::update(const State& self, const WorldView& world,
                            double dt) {
    CommandSet out;
    out.pitch   = gammaCmd_;
    out.heading = psiCmd_;

    if (!world.isAlive(targetId_)) return out;   // hold last commands
    const State& tgt = world.stateOf(targetId_);

    // Relative geometry (NED: up = -z).
    const Vector3 r    = tgt.position - self.position;
    const Vector3 vrel = tgt.velocity - self.velocity;
    lastRange_ = r.norm();

    // Endgame: hold the collision course inside the blind range.
    if (seeded_ && blindRange_ > 0.0 && lastRange_ < blindRange_) return out;

    const double rup  = -r.z;
    const double vup  = -vrel.z;
    const double rho2 = r.x * r.x + r.y * r.y;      // horizontal range^2
    const double rho  = std::sqrt(rho2);

    // AZIMUTH LOS rate: d/dt atan2(ry, rx) in the horizontal plane.
    const double losAz = (rho2 > 1e-9)
                             ? (r.x * vrel.y - r.y * vrel.x) / rho2
                             : 0.0;

    // ELEVATION LOS rate: d/dt atan2(rup, rho); rhoDot projects the
    // horizontal closing velocity onto the horizontal LOS direction.
    double losEl = 0.0;
    const double den = rho2 + rup * rup;
    if (den > 1e-9 && rho > 1e-9) {
        const double rhoDot = (r.x * vrel.x + r.y * vrel.y) / rho;
        losEl = (rho * vup - rup * rhoDot) / den;
    }

    // Seed to the current course, then apply the PN turns.
    if (!seeded_) {
        gammaCmd_ = std::atan2(-self.velocity.z,
                               std::hypot(self.velocity.x, self.velocity.y));
        psiCmd_   = std::atan2(self.velocity.y, self.velocity.x);
        psiSeed_  = psiCmd_;
        seeded_   = true;
    }
    gammaCmd_ = std::clamp(gammaCmd_ + N_ * losEl * dt, -maxPitch_, maxPitch_);
    psiCmd_   = std::clamp(psiCmd_ + N_ * losAz * dt,
                           psiSeed_ - maxYaw_, psiSeed_ + maxYaw_);

    out.pitch   = gammaCmd_;
    out.heading = psiCmd_;
    return out;
}
