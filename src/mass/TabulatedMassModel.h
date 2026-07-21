#pragma once

#include <string>

#include "math/LookupTable1D.h"
#include "mass/MassModel.h"

// Mass properties from a lookup table vs time: mass, inertia, and CG station
// interpolated linearly and held at the endpoints outside the table. This is
// THE mass model -- a constant-mass vehicle is simply a two-row table, and a
// burning rocket's propellant drain, shrinking inertia, and CG travel are the
// general case.
//
// CSV columns (header names): time_s, mass_kg, ixx, iyy, izz
// Optional: xcg_m (absent = NaN: CG at the aero reference, no moment
// transfer) and the products of inertia ixy, ixz, iyz (absent = 0; they
// enter the tensor as negative off-diagonals, same convention as the old
// vehicle-json "inertia" block).
class TabulatedMassModel : public MassModel {
public:
    TabulatedMassModel(LookupTable1D mass, LookupTable1D ixx,
                       LookupTable1D iyy, LookupTable1D izz,
                       LookupTable1D ixy, LookupTable1D ixz,
                       LookupTable1D iyz, LookupTable1D xcg)
        : mass_(std::move(mass)), ixx_(std::move(ixx)), iyy_(std::move(iyy)),
          izz_(std::move(izz)), ixy_(std::move(ixy)), ixz_(std::move(ixz)),
          iyz_(std::move(iyz)), xcg_(std::move(xcg)) {}

    static TabulatedMassModel fromCsv(const std::string& path);

    MassState at(double time) const override {
        MassState s;
        s.mass = mass_.eval(time);
        const double ixy = ixy_.eval(time), ixz = ixz_.eval(time),
                     iyz = iyz_.eval(time);
        s.inertia = Matrix3x3(ixx_.eval(time), -ixy,            -ixz,
                              -ixy,            iyy_.eval(time), -iyz,
                              -ixz,            -iyz,            izz_.eval(time));
        s.xcg = xcg_.eval(time);
        return s;
    }

private:
    LookupTable1D mass_, ixx_, iyy_, izz_, ixy_, ixz_, iyz_, xcg_;
};
