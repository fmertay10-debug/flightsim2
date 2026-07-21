#pragma once

#include <memory>
#include <vector>

#include "core/Channel.h"
#include "gnc/control/Allocator.h"
#include "gnc/control/ControlLaw.h"
#include "math/Vector3.h"

class ForceComponent;

// Base for every control law that drives channels through the Allocator
// (ADR-0004). It owns the machinery all such laws share -- the (non-owning)
// component references, the Allocator, and a copy of the channel table for the
// declared limits -- so a concrete law contributes ONLY its distinct logic:
// how it turns commands + state into a desired body angular acceleration.
// That result funnels through commandAngularAccel(), the one shared back-end.
//
// A subclass:
//   - forwards the allocation damping to this ctor,
//   - sets table_ = t inside its bindChannels() (allocation needs the limits),
//   - computes (aRoll, aPitch, aYaw) each step and calls commandAngularAccel().
class AllocatingLaw : public ControlLaw {
public:
    explicit AllocatingLaw(double allocDamping = 1e-6)
        : allocator_(allocDamping) {}

    bool allocates() const override { return true; }

    // Keep non-owning references to the vehicle's components (the Vehicle owns
    // them and outlives the law) so effectiveness can be queried each step.
    void bindComponents(
        const std::vector<std::unique_ptr<ForceComponent>>& components) override {
        components_.clear();
        for (const auto& c : components) components_.push_back(c.get());
    }

protected:
    // Desired body angular acceleration (x=roll, y=pitch, z=yaw) [rad/s^2] ->
    // pseudo-control moment nu = I * a (attitude-only: no net force demand) ->
    // Allocator distributes it over the components' LIVE effectiveness, honoring
    // each channel's declared limits. This is the shared tail every allocating
    // law ends with.
    void commandAngularAccel(const Vector3& angAccel, const GncContext& gc,
                             ChannelValues& out) const;

    // Near vertical (|theta| > guardAngle) the heading and roll Euler angles are
    // ill-conditioned, so the yaw and roll loops rate-damp instead of tracking
    // them: aYaw = -yawRateDamp*r, aRoll = -rollRateDamp*p, each clamped to
    // +/-maxAngAccel. Returns true and fills aRoll/aYaw when the guard is active;
    // returns false (leaving them untouched) so the law runs its normal yaw/roll
    // control. Shared by every attitude law that launches near-vertical.
    static bool rateDampIfNearVertical(double theta, double guardAngle,
                                       double p, double r,
                                       double rollRateDamp, double yawRateDamp,
                                       double maxAngAccel,
                                       double& aRoll, double& aYaw);

    ChannelTable table_;                            // declared limits for allocation
    std::vector<const ForceComponent*> components_; // non-owning
    Allocator allocator_;
};
