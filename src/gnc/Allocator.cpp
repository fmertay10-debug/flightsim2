#include "gnc/Allocator.h"

#include <algorithm>

#include "math/Matrix3x3.h"

void Allocator::allocate(const Vector3& desiredMoment,
                         const ControlEffect* effects, int count,
                         const ChannelTable& table, ChannelValues& out) const {
    // Gram matrix G = B B^T (3x3), built column by column.
    Matrix3x3 G;
    for (int k = 0; k < count; ++k) {
        const Vector3& b = effects[k].dMoment;
        G(0, 0) += b.x * b.x; G(0, 1) += b.x * b.y; G(0, 2) += b.x * b.z;
        G(1, 0) += b.y * b.x; G(1, 1) += b.y * b.y; G(1, 2) += b.y * b.z;
        G(2, 0) += b.z * b.x; G(2, 1) += b.z * b.y; G(2, 2) += b.z * b.z;
    }

    // Ridge: relative to the mean authority, with an absolute floor so a
    // zero-authority axis inverts cleanly to zero output.
    const double lambda =
        damping_ * (G(0, 0) + G(1, 1) + G(2, 2)) / 3.0 + 1e-12;
    G(0, 0) += lambda;
    G(1, 1) += lambda;
    G(2, 2) += lambda;

    const Vector3 y = G.inverse() * desiredMoment;   // (BB^T+lI)^-1 nu

    for (int k = 0; k < count; ++k) {
        const ControlEffect& e = effects[k];
        const Vector3& b = e.dMoment;
        double u = b.x * y.x + b.y * y.y + b.z * y.z;   // B^T y
        const ChannelDef& def = table.def(e.channel.index);
        u = std::clamp(u, def.minValue, def.maxValue);
        out.set(e.channel, u);
    }
}
