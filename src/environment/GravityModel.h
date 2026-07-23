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

// Inverse-square GRAVITATION mu/r^2 about a sphere of the given radius,
// with an optional J2 oblateness term (the NESC 6-DOF check-case models,
// NASA/TM-2015-218675 Vol. II B.4.2/B.4.3). No centrifugal contribution --
// this is attraction on a non-rotating planet, ~9.82 m/s^2 at the surface,
// not the familiar 9.81. The scalar J2 form is exact on the equator, where
// J2 gravitation is purely radial; the model is not latitude-aware.
class InverseSquareGravity : public GravityModel {
public:
    InverseSquareGravity(double mu, double radius, double j2)
        : mu_(mu), radius_(radius), j2_(j2) {}
    double gravity(double altitude) const override {
        const double r  = radius_ + altitude;
        const double rr = radius_ / r;
        return mu_ / (r * r) * (1.0 + 1.5 * j2_ * rr * rr);
    }

private:
    double mu_, radius_, j2_;
};
