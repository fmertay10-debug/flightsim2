#include "gnc/control/laws/AllocatingLaw.h"

#include "component/ForceComponent.h"
#include "gnc/control/WrenchCommand.h"
#include "math/Matrix3x3.h"

void AllocatingLaw::commandAngularAccel(const Vector3& angAccel,
                                        const GncContext& gc,
                                        ChannelValues& out) const {
    // Scale the demanded accelerations by the current inertia (diagonal terms)
    // into a body-moment WrenchCommand; no net force is demanded.
    const Matrix3x3& I = gc.mass.inertia;
    const WrenchCommand nu{ Vector3(),
                            Vector3(I(0, 0) * angAccel.x,
                                    I(1, 1) * angAccel.y,
                                    I(2, 2) * angAccel.z) };

    ControlEffect effects[ChannelTable::kMaxChannels];
    int n = 0;
    const ComponentContext cctx{ gc.state, gc.air, gc.state.altitude(),
                                 gc.dt, gc.mass.xcg };
    for (const ForceComponent* c : components_)
        n += c->controlEffectiveness(cctx, effects + n,
                                     ChannelTable::kMaxChannels - n);
    allocator_.allocate(nu, effects, n, table_, out);
}
