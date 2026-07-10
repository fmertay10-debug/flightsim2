#include "test_util.h"

#include "math/Matrix3x3.h"
#include "math/Quaternion.h"
#include "math/Units.h"
#include "math/Vector3.h"
#include "math/LookupTable1D.h"
#include "math/LookupTable2D.h"

int main() {
    // --- Vector3 ---
    {
        const Vector3 a(1, 2, 3), b(4, 5, 6);
        CHECK_NEAR(a.dot(b), 32.0, 1e-12);
        const Vector3 c = a.cross(b);
        CHECK_NEAR(c.x, -3.0, 1e-12);
        CHECK_NEAR(c.y,  6.0, 1e-12);
        CHECK_NEAR(c.z, -3.0, 1e-12);
        CHECK_NEAR(Vector3(3, 4, 0).norm(), 5.0, 1e-12);
    }

    // --- Matrix3x3 inverse ---
    {
        const Matrix3x3 M(2, 0, 1,
                          0, 3, 0,
                          1, 0, 2);
        const Matrix3x3 I = M * M.inverse();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                CHECK_NEAR(I(i, j), (i == j) ? 1.0 : 0.0, 1e-12);
    }

    // --- Quaternion: Euler roundtrip ---
    {
        const double phi = units::deg2rad(10), theta = units::deg2rad(-20),
                     psi = units::deg2rad(135);
        const Quaternion q = Quaternion::fromEuler(phi, theta, psi);
        CHECK_NEAR(q.norm(), 1.0, 1e-12);
        const Vector3 e = q.toEuler();
        CHECK_NEAR(e.x, phi,   1e-10);
        CHECK_NEAR(e.y, theta, 1e-10);
        CHECK_NEAR(e.z, psi,   1e-10);
    }

    // --- Quaternion rotate: 90 deg yaw maps north (inertial) to -y (body) ---
    {
        const Quaternion q = Quaternion::fromEuler(0, 0, units::deg2rad(90));
        const Vector3 vb = q.rotate(Vector3(1, 0, 0));   // north, inertial
        CHECK_NEAR(vb.x, 0.0, 1e-12);
        CHECK_NEAR(vb.y, -1.0, 1e-12);
        // rotateBack is the inverse.
        const Vector3 vi = q.rotateBack(vb);
        CHECK_NEAR(vi.x, 1.0, 1e-12);
        CHECK_NEAR(vi.y, 0.0, 1e-12);
    }

    // --- LookupTable1D: interpolation and clamping ---
    {
        const LookupTable1D t({0, 1, 2}, {0, 10, 40});
        CHECK_NEAR(t.eval(0.5), 5.0, 1e-12);
        CHECK_NEAR(t.eval(1.5), 25.0, 1e-12);
        CHECK_NEAR(t.eval(-5), 0.0, 1e-12);
        CHECK_NEAR(t.eval(99), 40.0, 1e-12);
    }

    // --- LookupTable2D: bilinear ---
    {
        const LookupTable2D t({0, 1}, {0, 1}, {0, 1, 2, 3});   // z(x,y)=2x+y
        CHECK_NEAR(t.eval(0.5, 0.5), 1.5, 1e-12);
        CHECK_NEAR(t.eval(0.25, 0.75), 1.25, 1e-12);
        CHECK_NEAR(t.eval(-1, 2), 1.0, 1e-12);   // clamped corner (0,1)
    }

    // --- wrapAngle ---
    CHECK_NEAR(units::wrapAngle(units::deg2rad(350)), units::deg2rad(-10), 1e-12);
    CHECK_NEAR(units::wrapAngle(units::deg2rad(-190)), units::deg2rad(170), 1e-12);

    std::printf("test_math: all checks passed\n");
    return 0;
}
