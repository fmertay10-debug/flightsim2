#pragma once

#include <memory>

#include "core/AirData.h"
#include "environment/GravityModel.h"
#include "environment/WindModel.h"

// Provides atmospheric, gravitational, and wind data to the simulation.
// Gravity and wind are swappable Strategies injected at construction.
class Environment {
public:
    Environment(std::unique_ptr<GravityModel> gravity,
                std::unique_ptr<WindModel>    wind);

    // ISA standard atmosphere at a geometric altitude [m].
    AtmosphereState atmosphere(double altitude) const;

    double  gravity(double altitude) const;                        // [m/s^2]
    Vector3 wind(const Vector3& positionNed, double time) const;   // NED [m/s]

private:
    std::unique_ptr<GravityModel> gravity_;
    std::unique_ptr<WindModel>    wind_;
};
