#include "test_util.h"

#include <memory>

#include "environment/Environment.h"

int main() {
    Environment env(std::make_unique<FlatEarthGravity>(),
                    std::make_unique<NoWind>());

    // Sea level (ISA reference values).
    const AtmosphereState sl = env.atmosphere(0.0);
    CHECK_NEAR(sl.temperature, 288.15, 0.01);
    CHECK_NEAR(sl.pressure, 101325.0, 1.0);
    CHECK_NEAR(sl.density, 1.225, 0.001);
    CHECK_NEAR(sl.soundSpeed, 340.29, 0.1);

    // 11 km (tropopause).
    const AtmosphereState tp = env.atmosphere(11000.0);
    CHECK_NEAR(tp.temperature, 216.65, 0.5);
    CHECK_NEAR(tp.pressure, 22632.0, 150.0);

    // 20 km (isothermal layer).
    const AtmosphereState s20 = env.atmosphere(20000.0);
    CHECK_NEAR(s20.temperature, 216.65, 0.5);
    CHECK_NEAR(s20.pressure, 5474.9, 60.0);

    // Gravity models.
    CHECK_NEAR(env.gravity(0.0), 9.80665, 1e-9);
    const SphericalEarthGravity sph;
    CHECK_NEAR(sph.gravity(0.0), 9.80665, 1e-9);
    CHECK(sph.gravity(100000.0) < 9.6);

    // Constant wind strategy.
    Environment windy(std::make_unique<FlatEarthGravity>(),
                      std::make_unique<ConstantWind>(Vector3(5, -2, 0)));
    const Vector3 w = windy.wind(Vector3(), 0.0);
    CHECK_NEAR(w.x, 5.0, 1e-12);
    CHECK_NEAR(w.y, -2.0, 1e-12);

    std::printf("test_atmosphere: all checks passed\n");
    return 0;
}
