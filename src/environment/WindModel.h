#pragma once

#include "math/Vector3.h"

// Strategy: wind velocity (NED, m/s) as a function of position and time.
// Positive components mean air moving north/east/down.
class WindModel {
public:
    virtual ~WindModel() = default;
    virtual Vector3 wind(const Vector3& positionNed, double time) const = 0;
};

class NoWind : public WindModel {
public:
    Vector3 wind(const Vector3&, double) const override { return {0.0, 0.0, 0.0}; }
};

// Spatially and temporally constant wind field.
class ConstantWind : public WindModel {
public:
    explicit ConstantWind(const Vector3& windNed) : wind_(windNed) {}
    Vector3 wind(const Vector3&, double) const override { return wind_; }

private:
    Vector3 wind_;
};
