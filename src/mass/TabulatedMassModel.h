#pragma once

#include <string>

#include "math/LookupTable1D.h"
#include "mass/MassModel.h"

// Variable mass properties from a lookup table vs time: mass, diagonal
// inertia (Ixx, Iyy, Izz), and CG station all interpolated linearly and
// held at the endpoints outside the table. This is how a real rocket is
// modeled -- propellant burn drops mass, shrinks inertia, and shifts the CG.
//
// CSV columns (header names): time_s, mass_kg, ixx, iyy, izz, xcg_m
class TabulatedMassModel : public MassModel {
public:
    TabulatedMassModel(LookupTable1D mass, LookupTable1D ixx,
                       LookupTable1D iyy, LookupTable1D izz,
                       LookupTable1D xcg)
        : mass_(std::move(mass)), ixx_(std::move(ixx)), iyy_(std::move(iyy)),
          izz_(std::move(izz)), xcg_(std::move(xcg)) {}

    static TabulatedMassModel fromCsv(const std::string& path);

    MassState at(double time) const override {
        MassState s;
        s.mass    = mass_.eval(time);
        s.inertia = Matrix3x3::diagonal(ixx_.eval(time), iyy_.eval(time),
                                        izz_.eval(time));
        s.xcg     = xcg_.eval(time);
        return s;
    }

private:
    LookupTable1D mass_, ixx_, iyy_, izz_, xcg_;
};
