#pragma once

// Strategy: gravity magnitude as a function of altitude.
// Returns magnitude only; callers build the NED vector (0, 0, +g).
class GravityModel {
public:
    virtual ~GravityModel() = default;
    virtual double gravity(double altitude) const = 0;   // [m/s^2]
};

// Constant standard gravity -- fine for low-altitude flight.
class FlatEarthGravity : public GravityModel {
public:
    double gravity(double) const override { return 9.80665; }
};

// Inverse-square law -- matters for high-altitude rockets.
class SphericalEarthGravity : public GravityModel {
public:
    double gravity(double altitude) const override {
        constexpr double G0 = 9.80665;
        constexpr double RE = 6371000.0;   // mean Earth radius [m]
        const double r = RE + altitude;
        return G0 * (RE / r) * (RE / r);
    }
};
