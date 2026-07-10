#pragma once

#include "math/Vector3.h"

// Atmospheric state at one altitude (ISA standard atmosphere).
struct AtmosphereState {
    double density     = 0.0;   // rho [kg/m^3]
    double pressure    = 0.0;   // P   [Pa]
    double temperature = 0.0;   // T   [K]
    double soundSpeed  = 0.0;   // a   [m/s]
};

// Air-relative flight condition, computed once per step by the Entity and
// consumed by aero models and controllers. Includes wind.
struct AirData {
    AtmosphereState atmosphere;
    Vector3 velocityBody;        // air-relative velocity, body frame [m/s]
    double  airspeed = 0.0;      // |velocityBody| [m/s]
    double  alpha    = 0.0;      // angle of attack  atan2(w, u) [rad]
    double  beta     = 0.0;      // sideslip         atan2(v, u) [rad]
    double  mach     = 0.0;      // airspeed / soundSpeed
    double  qbar     = 0.0;      // dynamic pressure 0.5*rho*V^2 [Pa]
};
