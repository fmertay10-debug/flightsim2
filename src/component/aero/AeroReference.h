#pragma once

// Reference geometry an aero component normalizes its coefficients by.
struct AeroReference {
    double sref = 1.0;   // reference area [m^2]
    double cbar = 1.0;   // longitudinal reference length (chord / body length) [m]
    double bref = 1.0;   // lateral reference length (span / diameter) [m]
};
