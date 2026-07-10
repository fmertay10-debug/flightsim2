#pragma once

// Unit conversion helpers. Config files use degrees; the sim core is
// strictly SI radians -- convert at the boundary, never inside.
namespace units {

constexpr double PI = 3.14159265358979323846;

constexpr double deg2rad(double deg) { return deg * PI / 180.0; }
constexpr double rad2deg(double rad) { return rad * 180.0 / PI; }

// Wrap an angle to (-pi, pi] -- for heading error arithmetic.
inline double wrapAngle(double a) {
    while (a >  PI) a -= 2.0 * PI;
    while (a <= -PI) a += 2.0 * PI;
    return a;
}

} // namespace units
