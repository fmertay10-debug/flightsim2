#pragma once

#include <cmath>

#include "core/AirData.h"
#include "core/ControlInput.h"
#include "core/State.h"
#include "math/Vector3.h"

// Aerodynamic loads in the BODY frame. Reported about the model's moment
// reference station (see momentReferenceStation); the Entity transfers them
// to the current CG.
struct AeroForces {
    Vector3 force;    // [N]
    Vector3 moment;   // [Nm]
};

// Reference geometry every aero model normalizes its coefficients by.
struct AeroReference {
    double sref = 1.0;   // reference area [m^2]
    double cbar = 1.0;   // longitudinal reference length (chord / body length) [m]
    double bref = 1.0;   // lateral reference length (span / diameter) [m]
};

// Strategy: aerodynamic model. One concrete class per VEHICLE TYPE
// (AircraftAero, RocketAero, ...). To support a new vehicle type, subclass
// this and register a builder in aero::Factory -- nothing else changes.
class AeroModel {
public:
    virtual ~AeroModel() = default;

    virtual AeroForces compute(const State& state,
                               const AirData& air,
                               const ControlInput& control) const = 0;

    // Station (meters, increasing aft, same datum as MassState::xcg) that the
    // reported moments are taken about. The Entity transfers them from here to
    // the current CG. Return NaN (the default) to opt out -- the moments are
    // already about the CG and no transfer is applied (derivative models).
    virtual double momentReferenceStation() const { return std::nan(""); }
};
