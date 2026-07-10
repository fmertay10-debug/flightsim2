#include "math/Matrix3x3.h"

#include <cmath>
#include <iomanip>
#include <stdexcept>

Matrix3x3::Matrix3x3() : m{} {}

Matrix3x3::Matrix3x3(double a, double b, double c,
                     double d, double e, double f,
                     double g, double h, double i)
    : m{ {a, b, c},
         {d, e, f},
         {g, h, i} } {}

Matrix3x3 Matrix3x3::identity() {
    return Matrix3x3(1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);
}

Matrix3x3 Matrix3x3::diagonal(double xx, double yy, double zz) {
    return Matrix3x3(xx, 0,  0,
                     0,  yy, 0,
                     0,  0,  zz);
}

double&       Matrix3x3::operator()(int i, int j)       { return m[i][j]; }
const double& Matrix3x3::operator()(int i, int j) const { return m[i][j]; }

Matrix3x3 Matrix3x3::operator+(const Matrix3x3& rhs) const {
    Matrix3x3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = m[i][j] + rhs.m[i][j];
    return r;
}

Matrix3x3 Matrix3x3::operator-(const Matrix3x3& rhs) const {
    Matrix3x3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = m[i][j] - rhs.m[i][j];
    return r;
}

Matrix3x3 Matrix3x3::operator*(const Matrix3x3& rhs) const {
    Matrix3x3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                r.m[i][j] += m[i][k] * rhs.m[k][j];
    return r;
}

Vector3 Matrix3x3::operator*(const Vector3& v) const {
    return { m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
             m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
             m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z };
}

Matrix3x3 Matrix3x3::operator*(double s) const {
    Matrix3x3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = m[i][j] * s;
    return r;
}

Matrix3x3 Matrix3x3::transpose() const {
    Matrix3x3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = m[j][i];
    return r;
}

double Matrix3x3::det() const {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
         - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
         + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

Matrix3x3 Matrix3x3::inverse() const {
    const double d = det();
    if (std::abs(d) < 1e-12)
        throw std::runtime_error("Matrix3x3::inverse() - matrix is singular");
    const double invDet = 1.0 / d;

    Matrix3x3 r;
    r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;

    r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;

    r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
    r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;
    return r;
}

std::ostream& operator<<(std::ostream& os, const Matrix3x3& M) {
    os << std::fixed << std::setprecision(4);
    for (int i = 0; i < 3; ++i) {
        os << "[ ";
        for (int j = 0; j < 3; ++j)
            os << std::setw(10) << M.m[i][j] << " ";
        os << "]\n";
    }
    return os;
}
