#include "test_util.h"

#include "dynamics/SixDofEom.h"
#include "math/Units.h"

// Free fall: no aero, gravity only -> after 1 s, vz = g*t, z = g*t^2/2.
static void testFreeFall() {
    const SixDofEom eom;
    const double dt = 0.001, g = 9.80665, mass = 10.0;
    const Matrix3x3 I = Matrix3x3::diagonal(1, 1, 1);

    State s;
    for (int i = 0; i < 1000; ++i) {
        // Body aligned with inertial (identity attitude): force = weight, +z down.
        s = eom.solve(s, Vector3(0, 0, mass * g), Vector3(), mass, I, dt);
    }
    CHECK_NEAR(s.time, 1.0, 1e-9);
    CHECK_NEAR(s.velocity.z, g * 1.0, 1e-6);
    CHECK_NEAR(s.position.z, 0.5 * g * 1.0, 0.01);   // forward-Euler bias ~ g*dt/2*t
    CHECK_NEAR(s.attitude.norm(), 1.0, 1e-12);
}

// Constant roll rate, no external moment, symmetric inertia:
// attitude integrates to phi = p * t.
static void testPureRoll() {
    const SixDofEom eom;
    const double dt = 0.0005, p = units::deg2rad(90);   // 90 deg/s
    const Matrix3x3 I = Matrix3x3::diagonal(2, 2, 2);

    State s;
    s.angularRate = Vector3(p, 0, 0);
    for (int i = 0; i < 2000; ++i)                       // 1 second
        s = eom.solve(s, Vector3(), Vector3(), 1.0, I, dt);

    const Vector3 e = s.eulerAngles();
    CHECK_NEAR(e.x, units::deg2rad(90), 1e-3);
    CHECK_NEAR(e.y, 0.0, 1e-9);
    CHECK_NEAR(s.angularRate.x, p, 1e-12);               // torque-free, symmetric
    CHECK_NEAR(s.attitude.norm(), 1.0, 1e-12);
}

// Gyroscopic coupling: spin about x with asymmetric inertia and a transverse
// rate -> the INERTIAL-frame angular momentum VECTOR is conserved
// (torque-free). The vector matters: |I*omega| alone is conserved under
// either sign of the omega x (I*omega) term (the cross product is
// perpendicular to I*omega), so a magnitude-only check cannot catch a
// flipped gyroscopic sign -- only the frame-transformed vector can.
static void testTorqueFreeMomentum() {
    const SixDofEom eom;
    const double dt = 0.0002;
    const Matrix3x3 I = Matrix3x3::diagonal(1, 5, 5);

    State s;
    s.angularRate = Vector3(5.0, 0.5, 0.0);
    const double  h0  = (I * s.angularRate).norm();
    const Vector3 Li0 = s.attitude.rotateBack(I * s.angularRate);

    for (int i = 0; i < 5000; ++i)                       // 1 second
        s = eom.solve(s, Vector3(), Vector3(), 1.0, I, dt);

    const double h1 = (I * s.angularRate).norm();
    CHECK_NEAR(h1, h0, h0 * 1e-3);
    const Vector3 Li1 = s.attitude.rotateBack(I * s.angularRate);
    CHECK_NEAR((Li1 - Li0).norm(), 0.0, h0 * 5e-3);
}

int main() {
    testFreeFall();
    testPureRoll();
    testTorqueFreeMomentum();
    std::printf("test_sixdof: all checks passed\n");
    return 0;
}
