#pragma once

#include <cmath>

#include "math/Matrix3x3.h"

// A vehicle's mass properties at one instant. Inertia is about the CG in the
// body frame. xcg is the CG station used for the aerodynamic moment transfer;
// it must share the datum and direction (increasing aft) with the aero
// model's momentReferenceStation() -- meters from the nose for rockets, or
// (cg fraction x cbar) for the F-16. Only the DIFFERENCE (xcg - xref) is used,
// so the datum cancels. Default NaN means "CG at the aero reference": no
// transfer (the common constant-CG case).
struct MassState {
    double    mass    = 1.0;                          // [kg]
    Matrix3x3 inertia = Matrix3x3::identity();        // about CG, body [kg m^2]
    double    xcg     = std::nan("");                 // CG station [m]
};

// Strategy: mass properties as a function of time. The Lego "mass block" --
// swap ConstantMassModel for TabulatedMassModel to get burn-time variation of
// mass, inertia, and CG travel without touching aero or control.
class MassModel {
public:
    virtual ~MassModel() = default;
    virtual MassState at(double time) const = 0;
};
