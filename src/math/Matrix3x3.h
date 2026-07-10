#pragma once

#include <ostream>

#include "math/Vector3.h"

struct Matrix3x3 {
    double m[3][3];   // row-major: m[row][col]

    Matrix3x3();      // zero matrix
    Matrix3x3(double a, double b, double c,
              double d, double e, double f,
              double g, double h, double i);

    static Matrix3x3 identity();
    static Matrix3x3 diagonal(double xx, double yy, double zz);

    double&       operator()(int i, int j);
    const double& operator()(int i, int j) const;

    Matrix3x3 operator+(const Matrix3x3& rhs) const;
    Matrix3x3 operator-(const Matrix3x3& rhs) const;
    Matrix3x3 operator*(const Matrix3x3& rhs) const;
    Vector3   operator*(const Vector3& v)     const;
    Matrix3x3 operator*(double scalar)        const;

    Matrix3x3 transpose() const;
    double    det()       const;
    Matrix3x3 inverse()   const;   // throws std::runtime_error if singular

    friend std::ostream& operator<<(std::ostream& os, const Matrix3x3& M);
};
