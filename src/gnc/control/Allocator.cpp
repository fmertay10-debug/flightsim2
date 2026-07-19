#include "gnc/control/Allocator.h"

#include <algorithm>
#include <cmath>

#include "math/Matrix3x3.h"

namespace {

inline bool nonzero(const Vector3& v) {
    return v.x != 0.0 || v.y != 0.0 || v.z != 0.0;
}

// Solve A y = b for a small dense NxN system by Gaussian elimination with
// partial pivoting. A is row-major, both A and b are overwritten (b <- y).
// Returns false if the system is singular.
bool solveDense(int n, double* A, double* b) {
    for (int col = 0; col < n; ++col) {
        int piv = col;
        double best = std::fabs(A[col * n + col]);
        for (int r = col + 1; r < n; ++r) {
            const double v = std::fabs(A[r * n + col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best == 0.0) return false;
        if (piv != col) {
            for (int c = 0; c < n; ++c) std::swap(A[col * n + c], A[piv * n + c]);
            std::swap(b[col], b[piv]);
        }
        const double d = A[col * n + col];
        for (int r = col + 1; r < n; ++r) {
            const double f = A[r * n + col] / d;
            if (f == 0.0) continue;
            for (int c = col; c < n; ++c) A[r * n + c] -= f * A[col * n + c];
            b[r] -= f * b[col];
        }
    }
    for (int r = n - 1; r >= 0; --r) {
        double s = b[r];
        for (int c = r + 1; c < n; ++c) s -= A[r * n + c] * b[c];
        b[r] = s / A[r * n + r];
    }
    return true;
}

}  // namespace

void Allocator::allocate(const WrenchCommand& desired,
                         const ControlEffect* effects, int count,
                         const ChannelTable& table, ChannelValues& out) const {
    // Does force enter at all? If neither the demand nor any effector carries
    // force, the moment-only 3x3 path is exact -- and byte-identical to the
    // pre-ADR-0004 allocator.
    bool hasForce = nonzero(desired.force);
    for (int k = 0; !hasForce && k < count; ++k)
        hasForce = nonzero(effects[k].dForce);

    if (!hasForce) {
        // ---- Attitude-only: 3x3 moment Gram G = B B^T (unchanged) ----
        Matrix3x3 G;
        for (int k = 0; k < count; ++k) {
            const Vector3& b = effects[k].dMoment;
            G(0, 0) += b.x * b.x; G(0, 1) += b.x * b.y; G(0, 2) += b.x * b.z;
            G(1, 0) += b.y * b.x; G(1, 1) += b.y * b.y; G(1, 2) += b.y * b.z;
            G(2, 0) += b.z * b.x; G(2, 1) += b.z * b.y; G(2, 2) += b.z * b.z;
        }
        // No moment authority anywhere (count == 0 or all-zero columns --
        // e.g. a pure-TVC vehicle's first pad step, before the gimbal's
        // one-step thrust lag fills in): leave the outputs untouched, same
        // contract as the 6x6 path's singular case.
        const double trace = G(0, 0) + G(1, 1) + G(2, 2);
        if (trace == 0.0) return;

        const double lambda = damping_ * trace / 3.0 + 1e-12;
        G(0, 0) += lambda;
        G(1, 1) += lambda;
        G(2, 2) += lambda;

        // Mirror Matrix3x3::inverse()'s ABSOLUTE epsilon: with authority this
        // small (e.g. the last milliseconds of a burnout ramp, LT -> 0) the
        // damped det underflows 1e-12 and inverse() would throw. Commanding
        // nothing there is physically right -- the effector is dead anyway.
        if (std::abs(G.det()) < 1e-12) return;

        const Vector3 y = G.inverse() * desired.moment;
        for (int k = 0; k < count; ++k) {
            const ControlEffect& e = effects[k];
            const Vector3& b = e.dMoment;
            double u = b.x * y.x + b.y * y.y + b.z * y.z;
            const ChannelDef& def = table.def(e.channel.index);
            u = std::clamp(u, def.minValue, def.maxValue);
            out.set(e.channel, u);
        }
        return;
    }

    // ---- Full 6-DOF wrench: B column k = [dForce; dMoment], G = B B^T (6x6) ----
    double b6[ChannelTable::kMaxChannels][6];
    for (int k = 0; k < count; ++k) {
        b6[k][0] = effects[k].dForce.x;
        b6[k][1] = effects[k].dForce.y;
        b6[k][2] = effects[k].dForce.z;
        b6[k][3] = effects[k].dMoment.x;
        b6[k][4] = effects[k].dMoment.y;
        b6[k][5] = effects[k].dMoment.z;
    }
    double G[36] = {0.0};
    for (int k = 0; k < count; ++k)
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                G[i * 6 + j] += b6[k][i] * b6[k][j];

    // Ridge relative to the mean authority over axes that HAVE authority, so
    // the moment-only sub-case matches the 3x3 path's normalization.
    double diagSum = 0.0;
    int nActive = 0;
    for (int i = 0; i < 6; ++i) {
        diagSum += G[i * 6 + i];
        if (G[i * 6 + i] > 0.0) ++nActive;
    }
    const double lambda = damping_ * diagSum / std::max(nActive, 1) + 1e-12;
    for (int i = 0; i < 6; ++i) G[i * 6 + i] += lambda;

    double y[6] = { desired.force.x, desired.force.y, desired.force.z,
                    desired.moment.x, desired.moment.y, desired.moment.z };
    if (!solveDense(6, G, y)) return;   // singular: leave outputs untouched

    for (int k = 0; k < count; ++k) {
        double u = 0.0;
        for (int i = 0; i < 6; ++i) u += b6[k][i] * y[i];
        const ChannelDef& def = table.def(effects[k].channel.index);
        u = std::clamp(u, def.minValue, def.maxValue);
        out.set(effects[k].channel, u);
    }
}
