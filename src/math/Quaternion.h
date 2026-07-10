#pragma once

#include <ostream>

#include "math/Vector3.h"
#include "math/Matrix3x3.h"

// Unit quaternion, scalar-first: q = w + x*i + y*j + z*k.
// Convention: 3-2-1 (yaw -> pitch -> roll) Euler sequence, INERTIAL -> BODY.
// rotate() / toDcm() assume the quaternion is normalized.
struct Quaternion {
    double w, x, y, z;

    Quaternion();                                       // identity (1,0,0,0)
    Quaternion(double w, double x, double y, double z);

    static Quaternion fromEuler(double phi, double theta, double psi);

    Quaternion  operator*(const Quaternion& rhs) const; // Hamilton product

    Quaternion  conjugate() const;
    double      norm()      const;
    Quaternion& normalize();                            // in place, returns *this
    Vector3     rotate(const Vector3& v) const;         // v_body = q* (0,v) q
    Vector3     rotateBack(const Vector3& v) const;     // body -> inertial
    Matrix3x3   toDcm()   const;                        // inertial -> body DCM
    Vector3     toEuler() const;                        // -> (phi, theta, psi)

    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);
};
